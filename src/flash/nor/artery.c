/***************************************************************************
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
***************************************************************************/

/* Only tested on AT32F415 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

#define MCU_DEVICE_ID_ADDR      0xE0042000
#define FLASH_SIZE_ADDR         0x1FFFF7E0
#define DEVICE_UID_ADDR         0x1FFFF7E8
#define MASK_VERSION_ADDR       0x1FFFF7F1
#define FLASH_BASE_ADDR         0x08000000
#define FLASH_BASE_BANK2_4032K	0x08200000
#define FLASH_BASE_BANK2_1024K	0x08080000

#define OTP_BANK_BASE_ADDR		0x1FFFC000

#define EFC_BASE				0x40023C00
#define EFC_BASE_BANK2			(EFC_BASE + 0x40)

#define EFC_CTRL_REG            0x10
#define EFC_PRGM_BIT            (1<<0)
#define EFC_PGERS_BIT           (1<<1)
#define EFC_BANKERS_BIT			(1<<2)
#define EFC_USD_PRGM_BIT		(1<<4)
#define EFC_USD_ERS_BIT			(1<<5)
#define EFC_RSTR_BIT            (1<<6)
#define EFC_LOCK_BIT            (1<<7)
#define EFC_USD_UNLOCK_BIT		(1<<9)
#define EFC_FCKEY_REG           0x04
#define EFC_USD_UNLOCK_REG		0x08
#define EFC_KEY1                0x45670123
#define EFC_KEY2                0xCDEF89AB
#define EFC_RDPRTEN             0x00A5
#define EFC_STS_REG             0x0C
#define EFC_BSY_BIT             (1<<0)
#define EFC_PRGMERR_BIT         (1<<2)	/* when the programming address is not 0xFFFF */
#define EFC_EPPERR_BIT          (1<<4)	/* Erase/program protection error */
#define EFC_PRCDN_BIT           (1<<5)
#define EFC_ADDR_REG            0x14

#define FLASH_ERASE_TIMEOUT 		100     /* 10ms actually required */
#define FLSAH_MASS_ERASE_TIMEOUT	(100 * 1000)	/* max 64s for 4Mb package */
#define FLASH_WRITE_TIMEOUT 		10      /* 42us actually required */

struct artery_flash_bank {
	bool probed;
	uint32_t flash_regs_base;    /* Address of flash reg controller */
};

struct artery_chip_info {
	uint32_t chip_id;
	uint32_t flash_size_kB;
	uint32_t sector_size;
	char *chip_name;
};

static const struct artery_chip_info known_artery_chips[] = {
{ 0xF0050340	, 1024	, 2048	, "AR8F403CGT6-A"	},
{ 0xF0050340	, 1024	, 2048	, "AR8F403CGT6"     },
{ 0x70050242	, 256	, 2048	, "AT32F403ACCT7"	},
{ 0x70050243	, 256	, 2048	, "AT32F403ACCU7"	},
{ 0x700502CF	, 512	, 2048	, "AT32F403ACET7"	},
{ 0x700502D0	, 512	, 2048	, "AT32F403ACEU7"	},
{ 0x70050346	, 1024	, 2048	, "AT32F403ACGT7"	},
{ 0x70050347	, 1024	, 2048	, "AT32F403ACGU7"	},
{ 0x70050241	, 256	, 2048	, "AT32F403ARCT7"	},
{ 0x700502CE	, 512	, 2048	, "AT32F403ARET7"	},
{ 0x70050345	, 1024	, 2048	, "AT32F403ARGT7"	},
{ 0x70050240	, 256	, 2048	, "AT32F403AVCT7"	},
{ 0x700502CD	, 512	, 2048	, "AT32F403AVET7"	},
{ 0x70050344	, 1024	, 2048	, "AT32F403AVGT7"	},
{ 0xF0050355	, 1024	, 2048	, "AT32F403AVGW"	},
{ 0x700301CF	, 128	, 1024	, "AT32F403CBT6"	},
{ 0x70050243	, 256	, 2048	, "AT32F403CCT6"	},
{ 0x7005024E	, 256	, 2048	, "AT32F403CCU6"	},
{ 0x700502CB	, 512	, 2048	, "AT32F403CET6"	},
{ 0x700502CD	, 512	, 2048	, "AT32F403CEU6"	},
{ 0x70050347	, 1024	, 2048	, "AT32F403CGT6"	},
{ 0x7005034C	, 1024	, 2048	, "AT32F403CGU6"	},
{ 0x70050242	, 256	, 2048	, "AT32F403RCT6"	},
{ 0x700502CA	, 512	, 2048	, "AT32F403RET6"	},
{ 0x70050346	, 1024	, 2048	, "AT32F403RGT6"	},
{ 0x70050241	, 256	, 2048	, "AT32F403VCT6"	},
{ 0x700502C9	, 512	, 2048	, "AT32F403VET6"	},
{ 0x70050345	, 1024	, 2048	, "AT32F403VGT6"	},
{ 0x70050240	, 256	, 2048	, "AT32F403ZCT6"	},
{ 0x700502C8	, 512	, 2048	, "AT32F403ZET6"	},
{ 0x70050344	, 1024	, 2048	, "AT32F403ZGT6"	},
{ 0x70050254	, 256	, 2048	, "AT32F407AVCT7"	},
{ 0x70050353	, 1024	, 2048	, "AT32F407AVGT7"	},
{ 0x7005024A	, 256	, 2048	, "AT32F407RCT7"	},
{ 0x700502D2	, 512	, 2048	, "AT32F407RET7"	},
{ 0x7005034C	, 1024	, 2048	, "AT32F407RGT7"	},
{ 0x70050249	, 256	, 2048	, "AT32F407VCT7"	},
{ 0x700502D1	, 512	, 2048	, "AT32F407VET7"	},
{ 0x7005034B	, 1024	, 2048	, "AT32F407VGT7"	},
{ 0x70030106	, 64	, 1024	, "AT32F413C8T7"	},
{ 0x700301C3	, 128	, 1024	, "AT32F413CBT7"	},
{ 0x700301CA	, 128	, 1024	, "AT32F413CBU7"	},
{ 0x70030242	, 256	, 2048	, "AT32F413CCT7"	},
{ 0x70030247	, 256	, 2048	, "AT32F413CCU7"	},
{ 0x700301C5	, 128	, 1024	, "AT32F413KBU7-4"	},
{ 0x70030244	, 256	, 2048	, "AT32F413KCU7-4"	},
{ 0x700301C1	, 128	, 1024	, "AT32F413RBT7"	},
{ 0x70030240	, 256	, 2048	, "AT32F413RCT7"	},
{ 0x700301CB	, 128	, 1024	, "AT32F413TBU7"	},
{ 0x70030109	, 64	, 1024	, "AT32F415C8T7"	},
{ 0x700301C5	, 128	, 1024	, "AT32F415CBT7"	},
{ 0x700301CD	, 128	, 1024	, "AT32F415CBU7"	},
{ 0x70030241	, 256	, 2048	, "AT32F415CCT7"	},
{ 0x7003024C	, 256	, 2048	, "AT32F415CCU7"	},
{ 0x7003010A	, 64	, 1024	, "AT32F415K8U7-4"	},
{ 0x700301C6	, 128	, 1024	, "AT32F415KBU7-4"	},
{ 0x70030242	, 256	, 2048	, "AT32F415KCU7-4"	},
{ 0x7003010B	, 64	, 1024	, "AT32F415R8T7-7"	},
{ 0x70030108	, 64	, 1024	, "AT32F415R8T7"	},
{ 0x700301C7	, 128	, 1024	, "AT32F415RBT7-7"	},
{ 0x700301C4	, 128	, 1024	, "AT32F415RBT7"	},
{ 0x700301CF	, 128	, 1024	, "AT32F415RBW"	},
{ 0x70030243	, 256	, 2048	, "AT32F415RCT7-7"	},
{ 0x70030240	, 256	, 2048	, "AT32F415RCT7"	},
{ 0x7003024E	, 256	, 2048	, "AT32F415RCW"	},
{ 0x5001000C	, 16	, 1024	, "AT32F421C4T7"	},
{ 0x50020086	, 32	, 1024	, "AT32F421C6T7"	},
{ 0x50020100	, 64	, 1024	, "AT32F421C8T7"	},
{ 0xD0020100	, 64	, 1024	, "AT32F421C8W-YY"	},
{ 0x50020117	, 64	, 1024	, "AT32F421C8W"	},
{ 0x50010011	, 16	, 1024	, "AT32F421F4P7"	},
{ 0x50010010	, 16	, 1024	, "AT32F421F4U7"	},
{ 0x5002008B	, 32	, 1024	, "AT32F421F6P7"	},
{ 0x5002008A	, 32	, 1024	, "AT32F421F6U7"	},
{ 0x50020105	, 64	, 1024	, "AT32F421F8P7"	},
{ 0x50020104	, 64	, 1024	, "AT32F421F8U7"	},
{ 0x50010014	, 16	, 1024	, "AT32F421G4U7"	},
{ 0x50020093	, 32	, 1024	, "AT32F421G6U7"	},
{ 0x50020112	, 64	, 1024	, "AT32F421G8U7"	},
{ 0x5001000D	, 16	, 1024	, "AT32F421K4T7"	},
{ 0x5001000F	, 16	, 1024	, "AT32F421K4U7-4"	},
{ 0x5001000E	, 16	, 1024	, "AT32F421K4U7"	},
{ 0x50020087	, 32	, 1024	, "AT32F421K6T7"	},
{ 0x50020089	, 32	, 1024	, "AT32F421K6U7-4"	},
{ 0x50020088	, 32	, 1024	, "AT32F421K6U7"	},
{ 0x50020101	, 64	, 1024	, "AT32F421K8T7"	},
{ 0x50020103	, 64	, 1024	, "AT32F421K8U7-4"	},
{ 0x50020102	, 64	, 1024	, "AT32F421K8U7"	},
{ 0x50010016	, 16	, 1024	, "AT32F421PF4P7"	},
{ 0x50020115	, 64	, 1024	, "AT32F421PF8P7"	},
{ 0x7003210B	, 64	, 1024	, "AT32F423C8T7"	},
{ 0x7003210E	, 64	, 1024	, "AT32F423C8U7"	},
{ 0x700A21CA	, 128	, 1024	, "AT32F423CBT7"	},
{ 0x700A21CD	, 128	, 1024	, "AT32F423CBU7"	},
{ 0x700A3249	, 256	, 2048	, "AT32F423CCT7"	},
{ 0x700A324C	, 256	, 2048	, "AT32F423CCU7"	},
{ 0x70032115	, 64	, 1024	, "AT32F423K8U7-4"	},
{ 0x700A21D4	, 128	, 1024	, "AT32F423KBU7-4"	},
{ 0x700A3253	, 256	, 2048	, "AT32F423KCU7-4"	},
{ 0x70032108	, 64	, 1024	, "AT32F423R8T7-7"	},
{ 0x70032105	, 64	, 1024	, "AT32F423R8T7"	},
{ 0x700A21C7	, 128	, 1024	, "AT32F423RBT7-7"	},
{ 0x700A21C4	, 128	, 1024	, "AT32F423RBT7"	},
{ 0x700A3246	, 256	, 2048	, "AT32F423RCT7-7"	},
{ 0x700A3243	, 256	, 2048	, "AT32F423RCT7"	},
{ 0x70032112	, 64	, 1024	, "AT32F423T8U7"	},
{ 0x700A21D1	, 128	, 1024	, "AT32F423TBU7"	},
{ 0x700A3250	, 256	, 2048	, "AT32F423TCU7"	},
{ 0x70032102	, 64	, 1024	, "AT32F423V8T7"	},
{ 0x700A21C1	, 128	, 1024	, "AT32F423VBT7"	},
{ 0x700A3240	, 256	, 2048	, "AT32F423VCT7"	},
{ 0x50092087	, 32	, 1024	, "AT32F425C6T7"	},
{ 0x5009208A	, 32	, 1024	, "AT32F425C6U7"	},
{ 0x50092106	, 64	, 1024	, "AT32F425C8T7"	},
{ 0x50092109	, 64	, 1024	, "AT32F425C8U7"	},
{ 0x50092093	, 32	, 1024	, "AT32F425F6P7"	},
{ 0x50092112	, 64	, 1024	, "AT32F425F8P7"	},
{ 0x50092096	, 32	, 1024	, "AT32F425G6U7"	},
{ 0x50092115	, 64	, 1024	, "AT32F425G8U7"	},
{ 0x5009208D	, 32	, 1024	, "AT32F425K6T7"	},
{ 0x50092090	, 32	, 1024	, "AT32F425K6U7-4"	},
{ 0x5009210C	, 64	, 1024	, "AT32F425K8T7"	},
{ 0x5009210F	, 64	, 1024	, "AT32F425K8U7-4"	},
{ 0x50092084	, 32	, 1024	, "AT32F425R6T7-7"	},
{ 0x50092081	, 32	, 1024	, "AT32F425R6T7"	},
{ 0x50092103	, 64	, 1024	, "AT32F425R8T7-7"	},
{ 0x50092100	, 64	, 1024	, "AT32F425R8T7"	},
{ 0x7008449A	, 192	, 4096	, "AT32F435CCT7-W"	},
{ 0x7008324B	, 256	, 2048	, "AT32F435CCT7"	},
{ 0x7008449D	, 192	, 4096	, "AT32F435CCU7-W"	},
{ 0x7008324E	, 256	, 2048	, "AT32F435CCU7"	},
{ 0x700844D9	, 960	, 4096	, "AT32F435CGT7-W"	},
{ 0x7008334A	, 1024	, 2048	, "AT32F435CGT7"	},
{ 0x700844DC	, 960	, 4096	, "AT32F435CGU7-W"	},
{ 0x7008334D	, 1024	, 2048	, "AT32F435CGU7"	},
{ 0x70084558	, 4032	, 4096	, "AT32F435CMT7-E"	},
{ 0x70084549	, 4032	, 4096	, "AT32F435CMT7"	},
{ 0x7008455B	, 4032	, 4096	, "AT32F435CMU7-E"	},
{ 0x7008454C	, 4032	, 4096	, "AT32F435CMU7"	},
{ 0x70083248	, 256	, 2048	, "AT32F435RCT7"	},
{ 0x70083347	, 1024	, 2048	, "AT32F435RGT7"	},
{ 0x70084546	, 4032	, 4096	, "AT32F435RMT7"	},
{ 0x70083245	, 256	, 2048	, "AT32F435VCT7"	},
{ 0x70083344	, 1024	, 2048	, "AT32F435VGT7"	},
{ 0x70084543	, 4032	, 4096	, "AT32F435VMT7"	},
{ 0x70083242	, 256	, 2048	, "AT32F435ZCT7"	},
{ 0x70083341	, 1024	, 2048	, "AT32F435ZGT7"	},
{ 0x70084540	, 4032	, 4096	, "AT32F435ZMT7"	},
{ 0x70083257	, 256	, 2048	, "AT32F437RCT7"	},
{ 0x70083356	, 1024	, 2048	, "AT32F437RGT7"	},
{ 0x70084555	, 4032	, 4096	, "AT32F437RMT7"	},
{ 0x70083254	, 256	, 2048	, "AT32F437VCT7"	},
{ 0x70083353	, 1024	, 2048	, "AT32F437VGT7"	},
{ 0x70084552	, 4032	, 4096	, "AT32F437VMT7"	},
{ 0x70083251	, 256	, 2048	, "AT32F437ZCT7"	},
{ 0x70083350	, 1024	, 2048	, "AT32F437ZGT7"	},
{ 0x7008454F	, 4032	, 4096	, "AT32F437ZMT7"	},
{ 0x70030109	, 64	, 1024	, "AT32FEBKC8T7"	},
{ 0x10012006	, 16	, 1024	, "AT32L021C4T7"	},
{ 0x1001208D	, 32	, 1024	, "AT32L021C6T7"	},
{ 0x10012114	, 64	, 1024	, "AT32L021C8T7"	},
{ 0x10012001	, 16	, 1024	, "AT32L021F4P7"	},
{ 0x10012002	, 16	, 1024	, "AT32L021F4U7"	},
{ 0x10012088	, 32	, 1024	, "AT32L021F6P7"	},
{ 0x10012089	, 32	, 1024	, "AT32L021F6U7"	},
{ 0x1001210F	, 64	, 1024	, "AT32L021F8P7"	},
{ 0x10012110	, 64	, 1024	, "AT32L021F8U7"	},
{ 0x10012000	, 16	, 1024	, "AT32L021G4U7"	},
{ 0x10012087	, 32	, 1024	, "AT32L021G6U7"	},
{ 0x1001210E	, 64	, 1024	, "AT32L021G8U7"	},
{ 0x10012005	, 16	, 1024	, "AT32L021K4T7"	},
{ 0x10012003	, 16	, 1024	, "AT32L021K4U7-4"	},
{ 0x10012004	, 16	, 1024	, "AT32L021K4U7"	},
{ 0x1001208C	, 32	, 1024	, "AT32L021K6T7"	},
{ 0x1001208A	, 32	, 1024	, "AT32L021K6U7-4"	},
{ 0x1001208B	, 32	, 1024	, "AT32L021K6U7"	},
{ 0x10012113	, 64	, 1024	, "AT32L021K8T7"	},
{ 0x10012111	, 64	, 1024	, "AT32L021K8U7-4"	},
{ 0x10012112	, 64	, 1024	, "AT32L021K8U7"	},
{ 0x70030250	, 256	, 2048	, "AT32WB415CCU7-7"	},
{ 0xF00301C2	, 128	, 1024	, "KC9060"          },
{ 0             , 0     , 0     , NULL              }
};

static int artery_find_chip_from_id(uint32_t id, const struct artery_chip_info **chip_info)
{
	const struct artery_chip_info *curr_chip = known_artery_chips;
	while (curr_chip->chip_id != 0 || curr_chip->chip_name != NULL) {
		if (curr_chip->chip_id == id) {
			if (chip_info)
				*chip_info = curr_chip;
			return ERROR_OK;
		}
		curr_chip += 1;
	}
	return ERROR_FAIL;
}

static int artery_guess_sector_size_from_flash_size(int flash_size_kb)
{
	/* According to device DB, devices with 4096 byte sectors do not have a power of 2 kB of FLASH */
	if ((flash_size_kb & (flash_size_kb - 1)) != 0)
		return 4096;

	/* According to AT32F415 code examples, FLASH <= 128kB means 1024 bytes setor size */
	if (flash_size_kb <= 128)
		return 1024;

	/* Other devices have 2048 bytes per sectors */
	return 2048;
}

static bool artery_is_otp(struct flash_bank *bank)
{
	return (bank->base == OTP_BANK_BASE_ADDR);
}

static int artery_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct artery_flash_bank *artery_bank_info = bank->driver_priv;
	uint32_t device_id, sector_size;
	uint16_t read_flash_size_in_kb;
	unsigned int bank_size;
	const struct artery_chip_info *chip_info = 0;
	bool flash_is_unknown = false;
	int retval;

	artery_bank_info->probed = false;

	free(bank->sectors);
	bank->num_sectors = 0;
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->num_prot_blocks = 0;
	bank->prot_blocks = NULL;

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	/* Read device ID */
	retval = target_read_u32(target, MCU_DEVICE_ID_ADDR, &device_id);
	if (retval != ERROR_OK) {
		LOG_WARNING("Cannot read device ID.");
		return retval;
	}

	/* get flash size from target. */
	retval = target_read_u16(target, FLASH_SIZE_ADDR, &read_flash_size_in_kb);
	if (retval != ERROR_OK || read_flash_size_in_kb == 0xffff || read_flash_size_in_kb == 0) {
		LOG_WARNING("Cannot read flash size.");
		flash_is_unknown = true;
		//return ERROR_FAIL;
	}

	/* look up chip id in known chip db */
	retval = artery_find_chip_from_id(device_id, &chip_info);
	if (retval == ERROR_OK) {
		if (flash_is_unknown) {
			read_flash_size_in_kb = chip_info->flash_size_kB;
		}
		/* known flash size matches read flash size. Trust known sector size */
		if (read_flash_size_in_kb == chip_info->flash_size_kB) {
			sector_size = chip_info->sector_size;
			LOG_INFO("Chip: %s, %" PRIi32 "kB FLASH, %" PRIi32 " bytes sectors",
						chip_info->chip_name, read_flash_size_in_kb, sector_size);
		}

		/* Known flash size does not match read flash size. Guess sector size */
		else {
			sector_size = artery_guess_sector_size_from_flash_size(read_flash_size_in_kb);
			LOG_INFO("Chip: %s, %" PRIi32 "kB FLASH expected, but %" PRIi32 \
						"kB detected. Guessing %" PRIi32 " bytes sectors",
						chip_info->chip_name, chip_info->flash_size_kB,
						read_flash_size_in_kb, sector_size);
		}
	}

	/* Unknown chip. Guess sector size */
	else {
		sector_size = artery_guess_sector_size_from_flash_size(read_flash_size_in_kb);
		LOG_INFO("Unknown chip id: %" PRIi32 ", %" PRIi32 \
					"kB FLASH detected. Guessing %" PRIi32 " bytes sectors",
					device_id, read_flash_size_in_kb, sector_size);
	}

	if (bank->base == 0x0)
		bank->base = FLASH_BASE_ADDR;

	if (bank->base == OTP_BANK_BASE_ADDR) {
		/* User area/option bytes */
		artery_bank_info->flash_regs_base = EFC_BASE;
		if ((read_flash_size_in_kb == 4032) || (read_flash_size_in_kb == 448))
			sector_size = bank_size = 4 << 10;
		else
			/* for 1024K and 256K */
			sector_size = bank_size = 512;
		LOG_INFO("User system area: %" PRIi32 " bytes", bank_size);
	} else if (bank->base == FLASH_BASE_ADDR) {
		/* Bank 1 */
		artery_bank_info->flash_regs_base = EFC_BASE;
		if (read_flash_size_in_kb == 4032)
			bank_size = 2048 << 10;
		else if (read_flash_size_in_kb == 1024)
			bank_size = 512 << 10;
		else {
			/* For 448K and 256K */
			bank_size = read_flash_size_in_kb;
		}
		LOG_INFO("Bank 1: %" PRIi32 "kB", bank_size >> 10);
	} else if ((bank->base == FLASH_BASE_BANK2_4032K) || (bank->base == FLASH_BASE_BANK2_1024K)) {
		/* Bank 2 */
		artery_bank_info->flash_regs_base = EFC_BASE_BANK2;
		if (read_flash_size_in_kb == 4032) {
			bank_size = 1984 << 10;
			if (bank->base != FLASH_BASE_BANK2_4032K) {
				LOG_INFO("Fixing base address for bank 2: %" PRIx32 "", FLASH_BASE_BANK2_4032K);
				bank->base = FLASH_BASE_BANK2_4032K;
			}
		} else if (read_flash_size_in_kb == 1024) {
			bank_size = 512 << 10;
			if (bank->base != FLASH_BASE_BANK2_1024K) {
				LOG_INFO("Fixing base address for bank 2: %" PRIx32 "", FLASH_BASE_BANK2_1024K);
				bank->base = FLASH_BASE_BANK2_1024K;
			}
		}
		else {
			/* Chip has no second bank */
			bank_size = 0;
		}
		LOG_INFO("Bank 2: %" PRIi32 "kB", bank_size >> 10);
	} else {
		LOG_ERROR("Unsupported bank base address %" PRIx32 " !", (uint32_t)bank->base);
		return ERROR_FAIL;
	}

	bank->size = bank_size;
	if (bank_size != 0) {
		unsigned int num_sectors;

		num_sectors = bank_size / sector_size;
		//if ((num_sectors * sector_size) != flash_size) {
		//	LOG_ERROR("Total FLASH size does not match sector size times sectors count !");
		//	return ERROR_FAIL;
		//}

		bank->num_sectors = num_sectors;

		if (num_sectors != 0) {
			bank->sectors = calloc(num_sectors, sizeof(struct flash_sector));

			for (unsigned int i = 0; i < num_sectors; i++) {
				bank->sectors[i].offset = i * sector_size;
				bank->sectors[i].size = sector_size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = -1;
				/* Currently we simply ignore sector protection. TODO : implement protection read / write */
			}
			LOG_DEBUG("allocated %u sectors", num_sectors);
		} else {
			LOG_DEBUG("Chip has no bank 2");
		}
	}

	artery_bank_info->probed = true;
	return ERROR_OK;
}

/* flash bank stm32x <base> <size> 0 0 <target#>
*/
FLASH_BANK_COMMAND_HANDLER(artery_flash_bank_command)
{
	struct artery_flash_bank *artery_bank_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	artery_bank_info = malloc(sizeof(struct artery_flash_bank));
	bank->driver_priv = artery_bank_info;

	artery_bank_info->probed = false;
	artery_bank_info->flash_regs_base = 0;

	return ERROR_OK;
}

static inline uint32_t artery_get_flash_reg(struct flash_bank *bank, uint32_t reg_offset)
{
	struct artery_flash_bank *artery_bank_info = bank->driver_priv;
	return reg_offset + artery_bank_info->flash_regs_base;
}

static inline int artery_read_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t *value)
{
	uint32_t reg_addr = artery_get_flash_reg(bank, reg_offset);
	int retval = target_read_u32(bank->target, reg_addr, value);

	if (retval != ERROR_OK)
		LOG_ERROR("error while reading from address 0x%" PRIx32, reg_addr);

	return retval;
}

static inline int artery_write_flash_reg(struct flash_bank *bank, uint32_t reg_offset, uint32_t value)
{
	uint32_t reg_addr = artery_get_flash_reg(bank, reg_offset);
	int retval = target_write_u32(bank->target, reg_addr, value);

	if (retval != ERROR_OK)
		LOG_ERROR("error while writing to address 0x%" PRIx32, reg_addr);

	return retval;
}

static int artery_unlock_flash(struct flash_bank *bank)
{
	uint32_t ctrl;

	int retval = artery_read_flash_reg(bank, EFC_CTRL_REG, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & EFC_LOCK_BIT) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = artery_write_flash_reg(bank, EFC_FCKEY_REG, EFC_KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = artery_write_flash_reg(bank, EFC_FCKEY_REG, EFC_KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = artery_read_flash_reg(bank, EFC_CTRL_REG, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & EFC_LOCK_BIT) {
		LOG_ERROR("flash not unlocked FLASH_CTRL: 0x%" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int artery_unlock_user(struct flash_bank *bank)
{
	uint32_t ctrl;
	int timeout = 1000;

	int retval = artery_read_flash_reg(bank, EFC_CTRL_REG, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & EFC_USD_UNLOCK_BIT) == EFC_USD_UNLOCK_BIT)
		return ERROR_OK;

	/* unlock flash registers */
	retval = artery_write_flash_reg(bank, EFC_FCKEY_REG, EFC_KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = artery_write_flash_reg(bank, EFC_FCKEY_REG, EFC_KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* unlock user area registers */
	retval = artery_write_flash_reg(bank, EFC_USD_UNLOCK_REG, EFC_KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = artery_write_flash_reg(bank, EFC_USD_UNLOCK_REG, EFC_KEY2);
	if (retval != ERROR_OK)
		return retval;

	do {
		retval = artery_read_flash_reg(bank, EFC_CTRL_REG, &ctrl);
		if (retval != ERROR_OK)
			return retval;

		if ((ctrl & EFC_USD_UNLOCK_BIT) == EFC_USD_UNLOCK_BIT) {
			return ERROR_OK;
		}
	} while (timeout--);

	LOG_ERROR("user flash not unlocked FLASH_CTRL: 0x%" PRIx32, ctrl);

	return ERROR_TARGET_FAILURE;
}

static int artery_unlock(struct flash_bank *bank)
{
	if (artery_is_otp(bank))
		return artery_unlock_user(bank);
	else
		return artery_unlock_flash(bank);
}

static int artery_lock(struct flash_bank *bank)
{
	if (artery_is_otp(bank))
		return artery_write_flash_reg(bank, EFC_CTRL_REG, 0);
	else
		return artery_write_flash_reg(bank, EFC_CTRL_REG, EFC_LOCK_BIT);

}

/* only wait while busy */
static int artery_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = artery_read_flash_reg(bank, EFC_STS_REG, &status);
		if (retval != ERROR_OK)
			return retval;
		if ((status & EFC_BSY_BIT) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	return retval;
}

/* wait while busy and check STS for errors */
static int artery_wait_status_busy_and_check(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = artery_read_flash_reg(bank, EFC_STS_REG, &status);
		if (retval != ERROR_OK)
			return retval;
		if ((status & EFC_BSY_BIT) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & EFC_EPPERR_BIT) {
		LOG_ERROR("Device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & EFC_PRGMERR_BIT) {
		LOG_ERROR("Attempt to write an address that has not been erased before");
		retval = ERROR_FAIL;
	}
	return retval;
}

static int artery_mass_erase(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (artery_is_otp(bank)) {
		LOG_ERROR("Mass erase for USD area is not supported");
		return ERROR_FLASH_BANK_INVALID;
	}

	int retval;
	retval = artery_unlock(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Wait for flash not busy */
	retval = artery_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Clear status register beforehand */
	artery_write_flash_reg(bank, EFC_STS_REG, EFC_PRCDN_BIT | EFC_PRGMERR_BIT | EFC_EPPERR_BIT);

	/* Set the PGERS or USDERS bit in the FLASH_CTRLx register */
	retval = artery_write_flash_reg(bank, EFC_CTRL_REG, EFC_BANKERS_BIT);
	if (retval != ERROR_OK)
		return retval;

	/* Set the RSTR bit in the FLASH_CTRLx register */
	retval = artery_write_flash_reg(bank, EFC_CTRL_REG, EFC_BANKERS_BIT | EFC_RSTR_BIT);
	if (retval != ERROR_OK)
		return retval;

	/* Check operation status */
	retval = artery_wait_status_busy_and_check(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* Re-lock flash */
	retval = artery_lock(bank);
	return retval;
}

COMMAND_HANDLER(artery_handle_mass_erase_command)
{
	if (CMD_ARGC < 1) {
		command_print(CMD, "artery mass_erase <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = artery_mass_erase(bank);
	if (retval == ERROR_OK) {
		command_print(CMD, "artery mass erase complete");
	} else {
		command_print(CMD, "artery mass erase failed");
	}

	return retval;
}

static const struct command_registration artery_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = artery_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static int artery_auto_probe(struct flash_bank *bank)
{
	struct artery_flash_bank *artery_bank_info = bank->driver_priv;
	if (artery_bank_info->probed)
		return ERROR_OK;
	return artery_probe(bank);
}

static int artery_print_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct target *target = bank->target;
	uint32_t device_id;
	uint16_t flash_size_in_kb;
	uint8_t mask_version;
	int retval;
	const struct artery_chip_info *chip_info = 0;

	/* Read device ID */
	retval = target_read_u32(target, MCU_DEVICE_ID_ADDR, &device_id);
	if (retval != ERROR_OK) {
		LOG_WARNING("Cannot read device ID.");
		return retval;
	}

	/* Read revision */
	retval = target_read_u8(target, MASK_VERSION_ADDR, &mask_version);
	if (retval != ERROR_OK) {
		LOG_WARNING("Cannot read mask version.");
		return retval;
	}
	mask_version = ((mask_version >> 4) & 0x07) + 'A';

	/* Read Flash size */
	retval = target_read_u16(target, FLASH_SIZE_ADDR, &flash_size_in_kb);
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("Cannot read flash size.");
		return retval;
	}

	/* look up chip id in known chip db */
	retval = artery_find_chip_from_id(device_id, &chip_info);
	if (retval == ERROR_OK)
		command_print_sameline(cmd, "Chip: %s Rev. %c, %" PRIi32 "kB FLASH",
							   chip_info->chip_name, mask_version, flash_size_in_kb);
	else
		command_print_sameline(cmd, "Unknown chip, Id: 0x%08" PRIx32 \
							   ", Rev: %c, %" PRIi32 "kB FLASH",
							   device_id, mask_version, flash_size_in_kb);

	return ERROR_OK;
}

static int artery_erase(struct flash_bank *bank, unsigned int first,
						unsigned int last)
{
	assert((first <= last) && (last < bank->num_sectors));

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	retval = artery_unlock(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Wait for flash not busy */
	retval = artery_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = first; i <= last; i++) {
		/* Clear status register beforehand */
		artery_write_flash_reg(bank, EFC_STS_REG, EFC_PRCDN_BIT | EFC_PRGMERR_BIT | EFC_EPPERR_BIT);

		/* Set the PGERS or USDERS bit in the FLASH_CTRLx register */
		retval = artery_write_flash_reg(bank, EFC_CTRL_REG, artery_is_otp(bank) ? (EFC_USD_ERS_BIT | EFC_USD_UNLOCK_BIT) : EFC_PGERS_BIT);
		if (retval != ERROR_OK)
			return retval;

		if (!artery_is_otp(bank)) {
			/* Select the page to be erased with the FLASH_ADDRx register */
			target_addr_t eraseAddress = bank->base + bank->sectors[i].offset;
			retval = artery_write_flash_reg(bank, EFC_ADDR_REG, eraseAddress);
			if (retval != ERROR_OK)
				return retval;
		}

		/* Set the RSTR bit in the FLASH_CTRLx register */
		retval = artery_write_flash_reg(bank, EFC_CTRL_REG, (artery_is_otp(bank) ? (EFC_USD_ERS_BIT | EFC_USD_UNLOCK_BIT) : EFC_PGERS_BIT) | EFC_RSTR_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Check operation status */
		retval = artery_wait_status_busy_and_check(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Re-lock flash */
	retval = artery_lock(bank);
	return retval;
}

static int artery_write_user(struct flash_bank *bank, const uint8_t *buffer,
							 uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	target_addr_t write_address = bank->base + offset;
	uint32_t bytes_written = 0;
	const uint16_t *data = (const uint16_t *)buffer;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = artery_unlock(bank);
	if (retval != ERROR_OK)
		return retval;

	if ((offset & 0x01) || (count & 0x01)) {
		LOG_ERROR("Destination address or count is not aligned to two bytes");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	/* Clear status register beforehand */
	artery_write_flash_reg(bank, EFC_STS_REG, EFC_PRCDN_BIT | EFC_PRGMERR_BIT | EFC_EPPERR_BIT);

	/* First write half-word by half-word, as it provides the highest write speed */
	while (bytes_written < (count - 1)) {
		/* Set the PRGM bit = 1 in FLASH_CTRL, keep USD unlock */
		retval = artery_write_flash_reg(bank, EFC_CTRL_REG, EFC_USD_PRGM_BIT | EFC_USD_UNLOCK_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Write word to flash */
		/* TODO: replace with target_write_u16() */
		retval = target_write_u16(target, write_address, *data);
		//retval = target_write_memory(target, write_address, 2, 1, buffer + bytes_written);
		if (retval != ERROR_OK)
			return retval;

		/* Check operation status */
		retval = artery_wait_status_busy_and_check(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 2;
		write_address += 2;
		data++;
	}

	/* Re-lock flash */
	retval = artery_lock(bank);
	return retval;
}

static int artery_write(struct flash_bank *bank, const uint8_t *buffer,
						uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	target_addr_t write_address = bank->base + offset;
	uint32_t bytes_written = 0;
	int retval;

	if (artery_is_otp(bank))
		return artery_write_user(bank, buffer, offset, count);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = artery_unlock(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Write byte by byte until we align to a word */
	while ((bytes_written < count) && (write_address & 0x03)) {
		/* Set the PRGM bit = 1 in FLASH_CTRL */
		retval = artery_write_flash_reg(bank, EFC_CTRL_REG, EFC_PRGM_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Write byte to flash */
		retval = target_write_u8(target, write_address, buffer[bytes_written]);
		if (retval != ERROR_OK)
			return retval;

		/* Check operation status */
		retval = artery_wait_status_busy_and_check(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 1;
		write_address += 1;
	}

	/* First write word by word, as it provides the highest write speed */
	while (bytes_written < (count - 3)) {
		/* Set the PRGM bit = 1 in FLASH_CTRL */
		retval = artery_write_flash_reg(bank, EFC_CTRL_REG, EFC_PRGM_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Write word to flash */
		retval = target_write_memory(target, write_address, 4, 1, buffer + bytes_written);
		if (retval != ERROR_OK)
			return retval;

		/* Check operation status */
		retval = artery_wait_status_busy_and_check(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 4;
		write_address += 4;
	}

	/* Write potential last bytes */
	while (bytes_written < count) {
		/* Set the PRGM bit = 1 in FLASH_CTRL */
		retval = artery_write_flash_reg(bank, EFC_CTRL_REG, EFC_PRGM_BIT);
		if (retval != ERROR_OK)
			return retval;

		/* Write byte to flash */
		retval = target_write_u8(target, write_address, buffer[bytes_written]);
		if (retval != ERROR_OK)
			return retval;

		/* Check operation status */
		retval = artery_wait_status_busy_and_check(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 1;
		write_address += 1;
	}

	/* Re-lock flash */
	retval = artery_lock(bank);
	return retval;
}

static const struct command_registration artery_command_handlers[] = {
{
	.name = "artery",
	.mode = COMMAND_ANY,
	.help = "artery flash command group",
	.usage = "",
	.chain = artery_exec_command_handlers,
},
COMMAND_REGISTRATION_DONE
};


const struct flash_driver artery_flash = {
	.name = "artery",
	.commands = artery_command_handlers,
	.flash_bank_command = artery_flash_bank_command,
	.erase = artery_erase,
	.protect = NULL,
	.write = artery_write,
	.read = default_flash_read,
	.probe = artery_probe,
	.auto_probe = artery_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.info = artery_print_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
