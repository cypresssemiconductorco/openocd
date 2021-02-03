/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2014 by Tomas Vanek (PSoC 4 support derived from STM32) *
 *   vanekt@fbl.cz                                                         *
 *                                                                         *
 *   Copyright (C) 2020 by Bohdan Tymkiv, Andrii Lishchynskyi              *
 *   bohdan.tymkiv@cypress.com                                             *
 *   andrii.lishchynskyi@cypress.com                                       *
 *                                                                         *
 *   Copyright (C) <2019-2020> < Cypress Semiconductor Corporation >       *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <jtag/jtag.h>
#include <flash/progress.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include "time_support.h"
#include "target/breakpoints.h"
#include "target/cortex_m.h"
#include "target/register.h"

/* device documets:

 PSoC(R) 4: PSoC 4200 Family Datasheet
		Document Number: 001-87197 Rev. *B  Revised August 29, 2013

 PSoC 4100/4200 Family PSoC(R) 4 Architecture TRM
		Document No. 001-85634 Rev. *E June 28, 2016

 PSoC(R) 4 Registers TRM Spec.
		Document No. 001-85847 Rev. *A June 25, 2013

 PSoC 4000 Family PSoC(R) 4 Technical Reference Manual
		Document No. 001-89309 Rev. *B May 9, 2016

 PSoC 41XX_BLE/42XX_BLE Family PSoC 4 BLE Architecture TRM
		Document No. 001-92738 Rev. *C February 12, 2016

 PSoC 4200L Family PSoC 4 Architecture TRM
		Document No. 001-97952 Rev. *A December 15, 2015

 PSoC 4200L Family PSoC 4 Registers TRM
		Document No. 001-98126 Rev. *A December 16, 2015

 PSoC 4100M/4200M Family PSoC 4 Architecture TRM
		Document No. 001-95223 Rev. *B July 29, 2015

 PSoC 4100S Family PSoC 4 Architecture TRM
		Document No. 002-10621 Rev. *A July 29, 2016

 PSoC 4100S Family PSoC 4 Registers TRM
		Document No. 002-10523 Rev. *A July 20, 2016

 PSoC Analog Coprocessor Architecture TRM
		Document No. 002-10404 Rev. ** December 18, 2015

 CY8C4Axx PSoC Analog Coprocessor Registers TRM
		Document No. 002-10405 Rev. ** December 18, 2015

 CY8C41xx, CY8C42xx Programming Specifications
		Document No. 001-81799 Rev. *C March 4, 2014

 CYBL10x6x, CY8C4127_BL, CY8C4247_BL Programming Specifications
		Document No. 001-91508 Rev. *B September 22, 2014

 http://dmitry.gr/index.php?r=05.Projects&proj=24.%20PSoC4%20confidential
*/

/* register locations */
#define PSOC4_SFLASH_MACRO0             0x0FFFF000
#define PSOC4_SFLASH_MACRO0_v2          0x0FFFE000

#define PSOC4_CPUSS_BASE_LEGACY         0x40000000
#define PSOC4_CPUSS_SYSREQ_LEGACY       PSOC4_CPUSS_BASE_LEGACY + 0x04
#define PSOC4_CPUSS_SYSARG_LEGACY       PSOC4_CPUSS_BASE_LEGACY + 0x08
#define PSOC4_SPCIF_BASE_LEGACY         PSOC4_CPUSS_BASE_LEGACY + 0xE0000
#define PSOC4_SPCIF_GEOMETRY_LEGACY     PSOC4_SPCIF_BASE_LEGACY + 0x00

#define PSOC4_CPUSS_BASE_NEW            0x40100000
#define PSOC4_CPUSS_SYSREQ_NEW          PSOC4_CPUSS_BASE_NEW + 0x04
#define PSOC4_CPUSS_SYSARG_NEW          PSOC4_CPUSS_BASE_NEW + 0x08
#define PSOC4_SPCIF_BASE_NEW            PSOC4_CPUSS_BASE_NEW + 0x10000
#define PSOC4_SPCIF_GEOMETRY_NEW        PSOC4_SPCIF_BASE_NEW + 0x00

#define PSOC4_ROMTABLE_PID0             0xF0000FE0

/* constants */
#define PSOC4_SFLASH_MACRO_SIZE         0x800
#define PSOC4_ROWS_PER_MACRO            512

#define PSOC4_SROM_KEY1                 0xB6
#define PSOC4_SROM_KEY2                 0xD3
#define PSOC4_SROM_SYSREQ_BIT           (1u << 31)
#define PSOC4_SROM_HMASTER_BIT          (1u << 30)
#define PSOC4_SROM_PRIVILEGED_BIT       (1u << 28)
#define PSOC4_DIS_RESET_VECT_REL_BIT    (1u << 27)
#define PSOC4_SROM_STATUS_SUCCEEDED     0xA0000000
#define PSOC4_SROM_STATUS_FAILED        0xF0000000
#define PSOC4_SROM_STATUS_MASK          0xF0000000

/* System Call Error Codes */
#define PSOC4_SROM_ERR_INVALID_PROTECTION   0xF0000001
#define PSOC4_SROM_ERR_INVALID_LATCH_ADDR   0xF0000003
#define PSOC4_SROM_ERR_INVALID_ADDR         0xF0000004
#define PSOC4_SROM_ERR_ROW_PROTECTED        0xF0000005
#define PSOC4_SROM_ERR_CHECKSUM             0xF000000A
#define PSOC4_SROM_ERR_INVALID_OPCODE       0xF000000B
#define PSOC4_SROM_ERR_INVALID_KEY_OPCODE   0xF000000C
#define PSOC4_SROM_ERR_INVALID_START_ADDR   0xF000000E
#define PSOC4_SROM_ERR_INVALID_CLOCK_FREQ   0xF0000012
#define PSOC4_SROM_ERR_IMO_NOT_IMPLEM       0xF0000013

#define PSOC4_CMD_GET_SILICON_ID        0
#define PSOC4_CMD_LOAD_LATCH            4
#define PSOC4_CMD_WRITE_ROW             5
#define PSOC4_CMD_PROGRAM_ROW           6
#define PSOC4_CMD_ERASE_ALL             0x0A
#define PSOC4_CMD_CHECKSUM              0x0B
#define PSOC4_CMD_WRITE_PROTECTION      0x0D
#define PSOC4_CMD_SET_IMO48             0x15
#define PSOC4_CMD_WRITE_SFLASH_ROW      0x18
#define PSOC4_CMD_SOFT_RESET            0x1B

#define PSOC4_CHIP_PROT_VIRGIN          0x00
#define PSOC4_CHIP_PROT_OPEN            0x01
#define PSOC4_CHIP_PROT_PROTECTED       0x02
#define PSOC4_CHIP_PROT_KILL            0x04

#define PSOC4_ROMTABLE_DESIGNER_CHECK   0xB4

#define PSOC4_FAMILY_FLAG_LEGACY        (1u << 0)
#define PSOC4_FLAG_IMO_NOT_REQUIRED     (1u << 1)

struct spcif_regs {
	uint32_t geometry_addr;
	uint32_t geometry_flash_mask;
	uint32_t geometry_flash_pos;
	uint32_t geometry_sflash_mask;
	uint32_t geometry_sflash_pos;
	uint32_t geometry_flash_macro_mask;
	uint32_t geometry_flash_macro_pos;
	uint32_t geometry_flash_rows_mask;
	uint32_t geometry_flash_rows_pos;
};

struct spcif_regs spcif_v2_t = {
	.geometry_flash_mask = 0xffff << 0,         /* 0:15 */
	.geometry_flash_pos = 0,
	.geometry_sflash_mask = 0x0F << 16,         /* 16:19 */
	.geometry_sflash_pos = 16,
	.geometry_flash_macro_mask = 0x03 << 20,    /* 20:21 */
	.geometry_flash_macro_pos = 20,
	.geometry_flash_rows_mask = 0x03 << 22,     /* 22:23 */
	.geometry_flash_rows_pos = 22
};

struct spcif_regs spcif_v3_t = {
	.geometry_flash_mask = 0x3FFF << 0,         /* 0:13 */
	.geometry_flash_pos = 0,
	.geometry_sflash_mask = 0x3F << 14,         /* 14:19 */
	.geometry_sflash_pos = 14,
	.geometry_flash_macro_mask = 0x03 << 20,    /* 20:21 */
	.geometry_flash_macro_pos = 20,
	.geometry_flash_rows_mask = 0x03 << 22,     /* 22:23 */
	.geometry_flash_rows_pos = 22
};

enum spcif_ver{
	spcif_unknown, spcif_v2, spcif_v3
};

struct sflash_type {
	uint32_t flash_prot_addr_increment;
	uint32_t chip_protection_offs;
	uint32_t wounding_offs;
	uint32_t siid_offs;
};

const struct sflash_type sflash_64 = {
	.flash_prot_addr_increment = 0,
	.chip_protection_offs = 0x07C,
	.wounding_offs = 0x140,
	.siid_offs = 0x144
};

const struct sflash_type sflash_128 = {
	.flash_prot_addr_increment = 0x400,
	.chip_protection_offs = 0x07C,
	.wounding_offs = 0x140,
	.siid_offs = 0x144
};

const struct sflash_type sflash_128x4 = {
	.flash_prot_addr_increment = 0x400,
	.chip_protection_offs = 0x07C,
	.wounding_offs = 0x140,
	.siid_offs = 0x144
};

const struct sflash_type sflash_256 = {
	.flash_prot_addr_increment = 0x800,
	.chip_protection_offs = 0x0FC,
	.wounding_offs = 0x240,
	.siid_offs = 0x244
};

struct psoc4_chip_family {
	uint16_t id;
	const char *name;
	uint32_t flags;
	enum spcif_ver spcif_ver;
};

const struct psoc4_chip_family psoc4_families[] = {
	{ 0x93, "PSoC 4100/4200",                       .flags = PSOC4_FAMILY_FLAG_LEGACY | PSOC4_FLAG_IMO_NOT_REQUIRED, .spcif_ver = spcif_v2 }, /* has known SROM bug (CDT#184422). */
	{ 0x9A, "PSoC 4000",                            .flags = 0, .spcif_ver = spcif_v2 }, /* has known SROM bug (CDT#184422) */
	{ 0x9E, "PSoC 4xx7 BLE",                        .flags = 0, .spcif_ver = spcif_v2 },
	{ 0xA0, "PSoC 4200L",                           .flags = PSOC4_FLAG_IMO_NOT_REQUIRED, .spcif_ver = spcif_v2 },
	{ 0xA1, "PSoC 4100M/4200M",                     .flags = PSOC4_FLAG_IMO_NOT_REQUIRED, .spcif_ver = spcif_v2 },
	{ 0xA3, "PSoC 4xx8 BLE",                        .flags = PSOC4_FLAG_IMO_NOT_REQUIRED, .spcif_ver = spcif_v2 },
	{ 0xA7, "PSoC 4000DS/4200DS",                   .flags = 0, .spcif_ver = spcif_v2 },
	{ 0xA9, "PSoC 4000S/4700S",                     .flags = 0, .spcif_ver = spcif_v3 },
	{ 0xAA, "PSoC 4xx8 BLE",                        .flags = PSOC4_FLAG_IMO_NOT_REQUIRED, .spcif_ver = spcif_v2 },
	{ 0xAB, "PSoC 4100S",                           .flags = 0, .spcif_ver = spcif_v3 },
	{ 0xAC, "PSoC 4100PS/PSoC Analog Coprocessor",  .flags = 0, .spcif_ver = spcif_v3 },
	{ 0xAE, "PSoC 4xx8 BLE",                        .flags = PSOC4_FLAG_IMO_NOT_REQUIRED, .spcif_ver = spcif_v2 },
	{ 0xB0, "CCG3PA USB Type-C Port Controller",    .flags = 0, .spcif_ver = spcif_v3 },
	{ 0xB5, "PSoC 4100S Plus",                      .flags = 0, .spcif_ver = spcif_v3 },
	{ 0xB8, "PSoC 4100S Plus/PSoC 4500",            .flags = 0, .spcif_ver = spcif_v3 },
	{ 0xBE, "PSoC 4100S Max",                       .flags = 0, .spcif_ver = spcif_v3 },
	{ 0xC0, "CCG6DF USB Type-C Port Controller",    .flags = 0, .spcif_ver = spcif_v3 },
	{ 0xC3, "CCG6SF USB Type-C Port Controller",    .flags = 0, .spcif_ver = spcif_v3 },
	{ 0,    "Unknown",                              .flags = 0, .spcif_ver = spcif_unknown }
};

struct psoc4_flash_bank {
	uint32_t row_size;
	uint32_t num_rows;
	uint32_t die_num_rows;
	uint32_t user_bank_size;
	int chip_protection;
	unsigned int num_macros;
	bool probed;
	uint16_t family_id;
	bool legacy_family;
	bool imo_required;
	uint32_t cpuss_sysreq_addr;
	uint32_t cpuss_sysarg_addr;
	uint32_t sflash_base;
	struct spcif_regs *spcif;
	const struct sflash_type *sflash_descr;
};

static bool is_sflash_base_v2(uint16_t family_id)
{
	return family_id == 0xBE;
}

static bool is_sflash_bank(struct flash_bank *bank)
{
	return 	(bank->base & PSOC4_SFLASH_MACRO0) == PSOC4_SFLASH_MACRO0 ||
			(bank->base & PSOC4_SFLASH_MACRO0_v2) == PSOC4_SFLASH_MACRO0_v2;
}

static bool is_flash_prot_bank(struct flash_bank *bank)
{
	return bank->base == 0x90400000;
}

/**************************************************************************************************
 * @brief Iterates over all psoc4 families, and locates psoc4_chip_family within given family_id
 * @param family_id family id of the connected PSoC4
 * @return pointer to psoc4_chip_family structure
 *************************************************************************************************/
static const struct psoc4_chip_family *psoc4_family_by_id(uint16_t family_id)
{
	const struct psoc4_chip_family *p = psoc4_families;
	while (p->id && p->id != family_id)
		p++;

	return p;
}

/**************************************************************************************************
 * @brief Determines Sflash type used in the device based on the geometry
 * @param row_size flash page size
 * @param num_macros amount of flash macro on the device
 * @return pointer to the sflash_type structure
 *************************************************************************************************/
static const struct sflash_type *psoc4_sflash_by_geometry(uint32_t row_size, uint32_t num_macros)
{
	switch (row_size)
	{
		case 64:
			return &sflash_64;
		case 128:
			return num_macros <= 2 ? &sflash_128 : &sflash_128x4;
		case 256:
			return &sflash_256;
		default:
			LOG_WARNING("Devices with %" PRIu32 " is not yet supported. Some operations might not work properly", row_size);
			return &sflash_256;
	}
}

/*************************************************************************************************
 * @brief Translates chip protection value to the human-readable format
 * @param protection device protection state
 * @return string representing chip protection in a human-readable format
 *************************************************************************************************/
static const char *psoc4_decode_chip_protection(uint8_t protection)
{
	switch (protection) {
		case PSOC4_CHIP_PROT_VIRGIN:
			return "protection VIRGIN";
		case PSOC4_CHIP_PROT_OPEN:
			return "protection OPEN";
		case PSOC4_CHIP_PROT_PROTECTED:
			return "PROTECTED";
		case PSOC4_CHIP_PROT_KILL:
			return "protection KILL";
		default:
			LOG_WARNING("Unknown protection state 0x%02" PRIx8 "", protection);
			return "";
	}
}


/*************************************************************************************************
 * @brief Translates sysreq error code to the human-readable format
 * @param error_code sysreq error code
 * @return string representing sysreq code in a human-readable format
 *************************************************************************************************/
static const char *psoc4_decode_sysreq_error(uint32_t error_code)
{
	switch (error_code) {
		case PSOC4_SROM_ERR_INVALID_PROTECTION:
			return "Invalid Chip Protection Mode - This API is not available in the current "
					"chip protection mode";
		case PSOC4_SROM_ERR_INVALID_LATCH_ADDR:
			return "Invalid Page Latch Address";
		case PSOC4_SROM_ERR_INVALID_ADDR:
			return "Invalid Address - The row id or byte address provided is outside of the "
					"available memory";
		case PSOC4_SROM_ERR_ROW_PROTECTED:
			return "Row Protected - The row id provided is a protected row";
		case PSOC4_SROM_ERR_CHECKSUM:
			return "Checksum Zero Failed";
		case PSOC4_SROM_ERR_INVALID_OPCODE:
			return "Invalid Opcode - The opcode is not a valid API opcode";
		case PSOC4_SROM_ERR_INVALID_KEY_OPCODE:
			return "Key Opcode Mismatch - The opcode provided does not match key1 and key2";
		case PSOC4_SROM_ERR_INVALID_START_ADDR:
			return "Invalid start address - The start address is greater than the end address provided";
		case PSOC4_SROM_ERR_INVALID_CLOCK_FREQ:
			return "Invalid Flash Clock - set IMO to 48MHz before Write/Erase operations";
		case PSOC4_SROM_ERR_IMO_NOT_IMPLEM:
			return "Setting IMO to 48MHz is not implemented";
		default:
			LOG_WARNING("Unknown sysreq error 0x%" PRIx32, error_code);
			return "";
	}
}

/* flash bank <name> psoc <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(psoc4_flash_bank_command)
{
	struct psoc4_flash_bank *psoc4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	psoc4_info = calloc(1, sizeof(struct psoc4_flash_bank));

	bank->driver_priv = psoc4_info;
	bank->default_padded_value = bank->erased_value = 0x00;
	psoc4_info->user_bank_size = bank->size;

	return ERROR_OK;
}

/*************************************************************************************************
 * @brief Very similar functionality to psoc4_sysreq, however it executes without core interferation.
 * Functionality limits to writing to CPUSS_SYSREQ and CPUSS_SYSARG and is used in order to unlock
 * the protected device
 * @param bank current flash bank
 * @param cmd ROM command to execute
 * @param cmd_param ROM command parameters
 * @param sysreq_params not supported, added for compatibility with psoc4_sysreq
 * @param sysreq_params_size not supported, added for compatibility with psoc4_sysreq
 * @param sysarg_out if not NULL, status of the system ROM API is returned to the caller
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_sysreq_mem_ap(struct flash_bank *bank, uint8_t cmd,
	uint16_t cmd_param,
	uint32_t *sysreq_params, uint32_t sysreq_params_size,
	uint32_t *sysarg_out)
{
	int retval;
	struct working_area *sysreq_mem;
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	uint32_t param = PSOC4_SROM_KEY1
		| ((PSOC4_SROM_KEY2 + cmd) << 8)
		| (cmd_param << 16);

	/* access to SRAM is locked in PROTECTED mode, thus it is impossible to invoke ROM API requiring params in SRAM */
	if ((sysreq_params != NULL || sysreq_params_size) && psoc4_info->chip_protection == PSOC4_CHIP_PROT_PROTECTED) {
		LOG_ERROR("Given psoc4 sysreq 0x%" PRIx8 " cannot be executed. Access to SRAM is locked in PROTECTED mode", cmd);
		return ERROR_FAIL;
	}
	/* Params in SRAM, but chip is not protected, so we have SRAM access */
	else if (sysreq_params != NULL || sysreq_params_size) {
		/* Allocate memory for sysreq_params */
		retval = target_alloc_working_area(target, sysreq_params_size, &sysreq_mem);
		if (retval != ERROR_OK) {
			LOG_WARNING("no working area for sysreq parameters");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			return retval;
		}

		/* Write sysreq_params */
		target_buffer_set_u32(target, (uint8_t *)sysreq_params, param);
		retval = target_write_buffer(target, sysreq_mem->address,
				sysreq_params_size, (uint8_t *)sysreq_params);
		if (retval != ERROR_OK)
			goto cleanup_mem;

		/* Set address of sysreq parameters block */
		retval = target_write_u32(target, psoc4_info->cpuss_sysarg_addr, sysreq_mem->address);
		if (retval != ERROR_OK)
			goto cleanup_mem;

	}
	else { /* Write params to the CPUSS_SYSARG */
		retval = target_write_u32(target, psoc4_info->cpuss_sysarg_addr, param);
		if (retval != ERROR_OK)
			goto cleanup_mem;
	}

	retval = target_write_u32(target, psoc4_info->cpuss_sysreq_addr, PSOC4_SROM_SYSREQ_BIT | PSOC4_SROM_HMASTER_BIT | cmd);
	if (retval != ERROR_OK)
		goto cleanup_mem;

	uint32_t val;
	int64_t t0 = timeval_ms();
	do {
		int64_t elapsed_ms = timeval_ms() - t0;
		if (elapsed_ms > 2000)
			break;

		retval = target_read_u32(target, psoc4_info->cpuss_sysreq_addr, &val);
		if (retval != ERROR_OK)
			goto cleanup_mem;

		val &= PSOC4_SROM_SYSREQ_BIT | PSOC4_SROM_PRIVILEGED_BIT;
	} while (val != 00);

	retval = target_read_u32(target, psoc4_info->cpuss_sysarg_addr, &val);
	if (retval != ERROR_OK)
		goto cleanup_mem;

	if (sysarg_out)
		*sysarg_out = val;
	else if ((val & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		LOG_ERROR("sysreq error - \"%s\"", psoc4_decode_sysreq_error(val));
		retval = ERROR_FAIL;
	}

cleanup_mem:
	if (sysreq_params != NULL || sysreq_params_size)
		target_free_working_area(target, sysreq_mem);

	return retval;
}

/*************************************************************************************************
 * @brief PSoC 4 system ROM request. Setting SROM_SYSREQ_BIT in CPUSS_SYSREQ register runs
 * NMI service in sysrem ROM. Algorithm just waits for NMI to finish.
 * When sysreq_params_size == 0 only one parameter is passed in CPUSS_SYSARG register.
 * Otherwise address of memory parameter block is set in CPUSS_SYSARG
 * and the first parameter is written to the first word of parameter block
 * @param bank current flash bank
 * @param cmd ROM cmd to execute
 * @param cmd_param ROM cmd parameters
 * @param sysreq_params ROM cmd parameters to be written into RAM
 * @param sysreq_params_size if 0, sysreq_params are written into CPUSS_SYSARG, otherwise
 * sysreq_params are written into SRAM and address when parameters are written is set in CPUSS_SYSARG
 * @param sysarg_out if not NULL, status of the system ROM API is returned to the caller
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_sysreq(struct flash_bank *bank, uint8_t cmd,
	uint16_t cmd_param,
	uint32_t *sysreq_params, uint32_t sysreq_params_size,
	uint32_t *sysarg_out)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	struct working_area *sysreq_wait_algorithm;
	struct working_area *sysreq_mem;

	struct reg_param reg_params[1];
	struct armv7m_algorithm armv7m_info;

	if (target->coreid == 0xff)
		return psoc4_sysreq_mem_ap(bank, cmd, cmd_param, sysreq_params, sysreq_params_size, sysarg_out);

	int retval = ERROR_OK;

	uint32_t param1 = PSOC4_SROM_KEY1
		| ((PSOC4_SROM_KEY2 + cmd) << 8)
		| (cmd_param << 16);

	static uint8_t psoc4_sysreq_wait_code[] = {
		/* system request NMI is served immediately after algo run
		now we are done: break */
		0x00, 0xbe,		/* bkpt 0 */
	};

	const int code_words = (sizeof(psoc4_sysreq_wait_code) + 3) / 4;
	/* stack must be aligned */
	const int stack_size = 256;
	/* tested stack sizes on PSoC4200:
			ERASE_ALL	144
			PROGRAM_ROW	112
			other sysreq	 68
	*/

	/* allocate area for sysreq wait code and stack */
	if (target_alloc_working_area(target, code_words * 4 + stack_size,
		&sysreq_wait_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for sysreq code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Write the code */
	retval = target_write_buffer(target,
			sysreq_wait_algorithm->address,
			sizeof(psoc4_sysreq_wait_code),
			psoc4_sysreq_wait_code);
	if (retval != ERROR_OK) {
		/* we already allocated the writing code, but failed to get a
		 * buffer, free the algorithm */
		goto cleanup_algo;
	}

	if (sysreq_params_size) {
		LOG_DEBUG("SYSREQ %02" PRIx8 " %04" PRIx16 " %08" PRIx32 " size %" PRIu32,
			cmd, cmd_param, param1, sysreq_params_size);
		/* Allocate memory for sysreq_params */
		retval = target_alloc_working_area(target, sysreq_params_size, &sysreq_mem);
		if (retval != ERROR_OK) {
			LOG_WARNING("no working area for sysreq parameters");

			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto cleanup_algo;
		}

		/* Write sysreq_params */
		target_buffer_set_u32(target, (uint8_t *)sysreq_params, param1);
		retval = target_write_buffer(target, sysreq_mem->address,
				sysreq_params_size, (uint8_t *)sysreq_params);
		if (retval != ERROR_OK)
			goto cleanup_mem;

		/* Set address of sysreq parameters block */
		retval = target_write_u32(target, psoc4_info->cpuss_sysarg_addr, sysreq_mem->address);
		if (retval != ERROR_OK)
			goto cleanup_mem;

	} else {
		/* Sysreq without memory block of parameters */
		LOG_DEBUG("SYSREQ %02" PRIx8 " %04" PRIx16 " %08" PRIx32,
			cmd, cmd_param, param1);
		/* Set register parameter */
		retval = target_write_u32(target, psoc4_info->cpuss_sysarg_addr, param1);
		if (retval != ERROR_OK)
			goto cleanup_mem;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* sysreq stack */
	init_reg_param(&reg_params[0], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32,
		sysreq_wait_algorithm->address + sysreq_wait_algorithm->size);

	struct armv7m_common *armv7m = target_to_armv7m(target);
	if (armv7m == NULL) {
		/* something is very wrong if armv7m is NULL */
		LOG_ERROR("unable to get armv7m target");
		retval = ERROR_FAIL;
		goto cleanup;
	}

	/* Set SROM request */
	retval = target_write_u32(target, psoc4_info->cpuss_sysreq_addr,
			PSOC4_SROM_SYSREQ_BIT | PSOC4_SROM_HMASTER_BIT | cmd);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Execute wait code */
	retval = target_run_algorithm(target, 0, NULL,
			sizeof(reg_params) / sizeof(*reg_params), reg_params,
			sysreq_wait_algorithm->address, 0, 1000, &armv7m_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("sysreq wait code execution failed");
		goto cleanup;
	}

	uint32_t sysarg_out_tmp;
	retval = target_read_u32(target, psoc4_info->cpuss_sysarg_addr, &sysarg_out_tmp);
	if (retval != ERROR_OK)
		goto cleanup;

	if (sysarg_out) {
		*sysarg_out = sysarg_out_tmp;
		/* If result is an error, do not show now, let caller to decide */
	} else if ((sysarg_out_tmp & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		LOG_ERROR("sysreq error - \"%s\"", psoc4_decode_sysreq_error(sysarg_out_tmp));
		retval = ERROR_FAIL;
	}
cleanup:
	destroy_reg_param(&reg_params[0]);

cleanup_mem:
	if (sysreq_params_size)
		target_free_working_area(target, sysreq_mem);

cleanup_algo:
	target_free_working_area(target, sysreq_wait_algorithm);

	return retval;
}


/*************************************************************************************************
 * @brief Executes system ROM API to read silicon/family ID and protection from the device
 * @param bank current flash bank
 * @param silicon_id pointer to variable, will be populated with silicon ID
 * @param family_id pointer to variable, will be populated with family ID
 * @param protection pointer to variable, will be populated with protection state
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_get_silicon_id(struct flash_bank *bank, bool no_algo, uint32_t *silicon_id, uint16_t *family_id, uint8_t *protection)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	uint32_t part0, part1;

	int retval;
	if (no_algo) {
		retval = psoc4_sysreq_mem_ap(bank, PSOC4_CMD_GET_SILICON_ID, 0, NULL, 0, &part0);
		if (retval != ERROR_OK)
			return retval;
	}
	else {
		retval = psoc4_sysreq(bank, PSOC4_CMD_GET_SILICON_ID, 0, NULL, 0, &part0);
		if (retval != ERROR_OK)
			return retval;
	}


	if ((part0 & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		LOG_ERROR("sysreq error - \"%s\"", psoc4_decode_sysreq_error(part0));
		return ERROR_FAIL;
	}

	retval = target_read_u32(target, psoc4_info->cpuss_sysreq_addr, &part1);
	if (retval != ERROR_OK)
		return retval;

	/* build ID as Cypress sw does:
	 * bit 31..16 silicon ID
	 * bit 15..8  revision ID (so far 0x11 for all devices)
	 * bit 7..0   family ID (lowes 8 bits)
	 */
	if (silicon_id)
		*silicon_id = ((part0 & 0x0000ffff) << 16)
			| ((part0 & 0x00ff0000) >> 8)
			| (part1 & 0x000000ff);

	if (family_id)
		*family_id = part1 & 0x0fff;

	psoc4_info->chip_protection = (part1 >> 12) & 0x0f;
	if (protection)
		*protection = psoc4_info->chip_protection;

	return ERROR_OK;
}


/*************************************************************************************************
 * @brief Retrieves family and revision IDs from the ROM table
 * @param target current target
 * @param family_id pointer to variable, will be populated with family ID
 * @param revision_id pointer to variable, will be populated with revision ID
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_get_family(struct target *target, uint16_t *family_id, uint16_t *revision_id)
{
	int retval, i;
	uint32_t pidbf[4];
	uint8_t pid[4];

	retval = target_read_memory(target, PSOC4_ROMTABLE_PID0, 4, 4, (uint8_t *)pidbf);
	if (retval != ERROR_OK)
		return retval;

	for (i = 0; i < 4; i++) {
		uint32_t tmp = target_buffer_get_u32(target, (uint8_t *)(pidbf + i));
		if (tmp & 0xffffff00) {
			LOG_ERROR("Unexpected data in ROMTABLE");
			return ERROR_FAIL;
		}
		pid[i] = tmp & 0xff;
	}

	uint16_t family = pid[0] | ((pid[1] & 0xf) << 8);
	uint16_t revision = (pid[2] & 0xF0) | ((pid[3] & 0xF0) >> 4);
	uint32_t designer = ((pid[1] & 0xf0) >> 4) | ((pid[2] & 0xf) << 4);

	if (designer != PSOC4_ROMTABLE_DESIGNER_CHECK) {
		LOG_ERROR("ROMTABLE designer is not Cypress");
		return ERROR_FAIL;
	}

	if (family_id)
		*family_id = family;
	if (revision_id)
		*revision_id = revision;

	return ERROR_OK;
}


/*************************************************************************************************
 * @brief Sets required frequency for program/erase operations.
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int set_imo_48mhz(struct flash_bank *bank)
{
	uint32_t sysreq_status;
	int retval = psoc4_sysreq(bank, PSOC4_CMD_SET_IMO48, 0, NULL, 0, &sysreq_status);
	if (retval != ERROR_OK)
		return retval;

	if ((sysreq_status & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		LOG_ERROR("sysreq error - \"%s\"", psoc4_decode_sysreq_error(sysreq_status));
		return ERROR_FAIL;
	}

	return retval;
}


/*************************************************************************************************
 * @brief Prepares flash for the program/erase operations
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_flash_prepare(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint16_t family_id;
	int retval;

	/* get family ID from SROM call */
	retval = psoc4_get_silicon_id(bank, false, NULL, &family_id, NULL);
	if (retval != ERROR_OK)
		return retval;

	/* and check with family ID from ROMTABLE */
	if (family_id != psoc4_info->family_id) {
		LOG_ERROR("Family mismatch");
		return ERROR_FAIL;
	}
	if (psoc4_info->imo_required)
		return set_imo_48mhz(bank);

	return ERROR_OK;
}


/*************************************************************************************************
 * @brief Reads flash level protection as a continuous memory block. This function wraps physical
 * flash protection organization.
 * @param bank current flash bank
 * @param buffer pointer to the buffer where flash protection data will be stored
 * @param offset starting offset in flash bank
 * @param count index of flash macro to be protected
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_flash_prot_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	uint32_t prot_addr = psoc4_info->sflash_base;
	uint32_t prot_addr_incr = psoc4_info->sflash_descr->flash_prot_addr_increment;
	uint32_t bytes_per_macro = psoc4_info->die_num_rows / psoc4_info->num_macros / 8;

	if ((psoc4_info->num_rows / 8) < offset + count) {
		LOG_ERROR("invalid size");
		return ERROR_FAIL;
	}

	memset(buffer, 0, count);
	while (count)
	{
		// determine which macro to read
		uint32_t cur_macro_id = offset / bytes_per_macro;
		uint32_t offs_in_macro = offset % bytes_per_macro;
		uint32_t bytes_this_run = count < bytes_per_macro - offs_in_macro ? count : bytes_per_macro - offs_in_macro;
		uint32_t address = prot_addr + prot_addr_incr * cur_macro_id + offs_in_macro;

		int retval = target_read_buffer(target, address, bytes_this_run, buffer);
		if (retval != ERROR_OK)
			return retval;

		offset += bytes_this_run;
		buffer += bytes_this_run;
		count -=  bytes_this_run;
	}

	return ERROR_OK;
}

/*************************************************************************************************
 * @brief Determine if the specific bank is "protected" or not.
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_protect_check(struct flash_bank *bank)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	uint8_t *buf = malloc(psoc4_info->num_rows/8);
	if (buf == NULL) {
		LOG_ERROR("no memory for protection buffer");
		return ERROR_FAIL;
	}

	int retval = psoc4_flash_prot_read(bank, buf, 0, psoc4_info->num_rows/8);
	if (retval != ERROR_OK)
		goto cleanup;

	for (uint32_t i = 0; i < psoc4_info->num_rows; i++)
		bank->sectors[i].is_protected = buf[i/8] & (1 << (i%8)) ? 1 : 0;

cleanup:
	free(buf);

	return retval;
}


/*************************************************************************************************
 * @brief Function will try to determine entry point of user application. If it succeeds it will
 * set HW breakpoint at that address, issue SW Reset and remove the breakpoint afterwards.
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_reset_halt(struct target *target)
{
	int hr;
	uint32_t reset_addr;
	const int halt_timeout = 1000;

	if (target->state != TARGET_HALTED) {
		hr = target_halt(target);
		if (hr != ERROR_OK)
			return hr;

		hr = target_wait_state(target, TARGET_HALTED, halt_timeout);
		if (hr != ERROR_OK)
			return hr;
	}

	/* Read and validate Reset Vector value */
	hr = target_read_u32(target, 0x00000004, &reset_addr);
	if (hr != ERROR_OK)
		return hr;

	struct flash_bank *main_bank;
	hr = get_flash_bank_by_addr(target, 0x00000000, true, &main_bank);
	if (hr != ERROR_OK)
		return hr;

	if(reset_addr > main_bank->size) {
		LOG_INFO("Entry Point address invalid (0x%08X), reset_halt skipped", reset_addr);
		return ERROR_OK;
	}

	/* Set breakpoint at User Application entry point */
	hr = breakpoint_add(target, reset_addr, 2, BKPT_HARD);
	if (hr != ERROR_OK)
		return hr;

	const struct armv7m_common *cm = target_to_armv7m(target);

	/* M0S8 patform reboots immediatelly after issuing SYSRESETREQ
	 * this disables SWD/JTAG pins momentarily and may break communication
	 * Ignoring return value of mem_ap_write_atomic_u32 seems to be ok here */

	/* Reset the CM0 by asserting SYSRESETREQ */
	LOG_INFO("%s: bkpt @0x%08X, issuing SYSRESETREQ", target->cmd_name, reset_addr);
	mem_ap_write_atomic_u32(cm->debug_ap, NVIC_AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ);

	dap_invalidate_cache(cm->debug_ap->dap);
	register_cache_invalidate(target->reg_cache);

	/* Target is now running, call appropriate callbacks */
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	enum log_levels lvl = change_debug_level(LOG_LVL_USER);
	int64_t t0 = timeval_ms();
	while((timeval_ms() - t0) < halt_timeout) {
		dap_dp_init(cm->debug_ap->dap);
		target->examined = false;
		hr = cortex_m_examine(target);
		if(hr == ERROR_OK)
			break;
		alive_sleep(5);
	}
	change_debug_level(lvl);

	hr = target_wait_state(target, TARGET_HALTED, halt_timeout);
	if (hr != ERROR_OK)
		return hr;

	/* Remove the break point */
	breakpoint_remove(target, reset_addr);

	return ERROR_OK;
}


/**************************************************************************************************
 * @brief Erases entire flash of the device via system ROM API
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_mass_erase(struct flash_bank *bank)
{
	unsigned int i;
	int retval;
	uint8_t protection;
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	/* First of all check the protection state */
	retval = psoc4_get_silicon_id(bank, false, NULL, NULL, &protection);
	if (retval != ERROR_OK)
		return retval;

	/* Prepare flash for further operations */
	if (psoc4_info->imo_required) {
		retval = set_imo_48mhz(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Mass erase path */
	if (protection != PSOC4_CHIP_PROT_PROTECTED) {
		/* Call "Erase All" system ROM API */
		uint32_t param = 0;
		retval = psoc4_sysreq(bank, PSOC4_CMD_ERASE_ALL,
				0, &param, sizeof(param), NULL);

		if (retval != ERROR_OK)
			return retval;
	}
	/* Unlock path */
	else {
		/*
		*	If we are protected perform the following:
		*	Call System ROM Api to move to the OPEN state (this also erases entire flash)
		*	perform soft reset in order to apply new protection and reexamine target
		*/
		retval = psoc4_sysreq(bank, PSOC4_CMD_WRITE_PROTECTION, 1, NULL, 0, NULL);
		if (retval != ERROR_OK)
			return retval;

		/* Ignore status of this call, since the chip will reset */
		uint32_t status;
		psoc4_sysreq(bank, PSOC4_CMD_SOFT_RESET, 0, NULL, 0, &status);
		busy_sleep(50);

		psoc4_info->probed = false;

		/* Reexamine target after reset has happened */
		retval = target_examine_one(target);
		if (retval != ERROR_OK)
			return retval;

		retval = target_poll(bank->target);
		if (retval != ERROR_OK)
			return retval;
	}

	/* set all sectors as erased */
	for (i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;

	/* Disable 0x0000:0000 - 0x0000:0007 remap to ROM */
	retval = target_write_u32(target, psoc4_info->cpuss_sysreq_addr,
			PSOC4_SROM_HMASTER_BIT | PSOC4_DIS_RESET_VECT_REL_BIT);

	return retval;
}


/*************************************************************************************************
 * @brief Writes flash and chip level protections within single flash macro.
 * psoc4_flash_prepare function should be called in advance.
 * @param bank current flash bank
 * @param chip_level_prot chip level protection to be set
 * @param buffer pointer to the buffer with flash protection data
 * @param count number of bytes in buffer
 * @param offset starting offset in flash protection where buffer will be written
 * @param macro index of flash macro to be protected
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int write_flash_macro_protection(struct flash_bank *bank, int chip_level_prot, const uint8_t *buffer, int count, int offset, int macro)
{
	struct target *target = bank->target;
	const int param_sz = 8;

	uint32_t *sysrq_buffer = malloc(param_sz + count);
	if (sysrq_buffer == NULL) {
		LOG_ERROR("no memory for row buffer");
		return ERROR_FAIL;
	}

	memset(sysrq_buffer, 0, param_sz + count);
	memcpy(sysrq_buffer + 2, buffer, count);

	/* Call "Load Latch" system ROM API */
	target_buffer_set_u32(target, (uint8_t *)(sysrq_buffer + 1), count - 1);
	int retval = psoc4_sysreq(bank, PSOC4_CMD_LOAD_LATCH,
			offset	/* Byte number in latch from what to write */
			| (macro << 8),	/* flash macro index */
			sysrq_buffer, param_sz + count,
			NULL);
	if (retval != ERROR_OK)
		goto cleanup;

	/* Call "Write Protection" system ROM API */
	retval = psoc4_sysreq(bank, PSOC4_CMD_WRITE_PROTECTION,
			chip_level_prot | (macro << 8), NULL, 0, NULL);

cleanup:
	free(sysrq_buffer);

	return retval;
}

/**************************************************************************************************
 * @brief Protects certain flash row from being overwritten
 * @param bank current flash bank
 * @param set If non-zero, enable protection; if 0, disable it
 * @param first The first sector to (un)protect
 * @param last The last sector to (un)project
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	if (!psoc4_info->probed)
		return ERROR_FAIL;

	int retval = psoc4_flash_prepare(bank);
	if (retval != ERROR_OK)
		return retval;

	unsigned int num_bits = bank->num_sectors;
	if (num_bits > PSOC4_ROWS_PER_MACRO)
		num_bits = PSOC4_ROWS_PER_MACRO;

	int prot_sz = num_bits / 8;

	uint8_t *data_buffer = malloc(prot_sz);
	if (data_buffer == NULL) {
		LOG_ERROR("no memory for row buffer");
		return ERROR_FAIL;
	}

	unsigned int i, m, sect;
	for (i = first; i <= last && i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = set;

	for (m = 0, sect = 0; m < psoc4_info->num_macros; m++) {
		memset(data_buffer, 0, prot_sz);
		for (i = 0; i < num_bits && sect < bank->num_sectors; i++, sect++) {
			if (bank->sectors[sect].is_protected == 1)
				data_buffer[i/8] |= 1 << (i%8);
		}

		retval = write_flash_macro_protection(bank, PSOC4_CHIP_PROT_OPEN, data_buffer, prot_sz, 0, m);
		if (retval != ERROR_OK)
			break;
	}

	free(data_buffer);

	psoc4_protect_check(bank);

	/* Disable 0x0000:0000 - 0x0000:0007 remap to ROM */
	if (retval == ERROR_OK)
		retval = target_write_u32(target, psoc4_info->cpuss_sysreq_addr,
				PSOC4_SROM_HMASTER_BIT | PSOC4_DIS_RESET_VECT_REL_BIT);

	return retval;
}

#define RAM_STACK_WA_SIZE 512
#define CMD_BUFFER_EMPTY 0
#define CMD_BUFFER_FULL  1
#define CMD_BUFFER_DONE  2

/**************************************************************************************************
 * @brief Performs Program/Erase operation
 * @param addr address of the flash row
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in flash bank
 * @param count number of bytes in buffer
 * @param operation type of the operation (erase or program)
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_write_inner(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count, int operation)
{

	const uint32_t sflash_mask = is_sflash_bank(bank) ? 0x80000000 : 0x00000000;

	static const uint8_t p4_legacy_algo[] = {
		#include "../../../contrib/loaders/flash/psoc4/psoc4_legacy_write.inc"
	};

	static const uint8_t p4_current_algo[] = {
		#include "../../../contrib/loaders/flash/psoc4/psoc4_current_write.inc"
	};

	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	const uint8_t *algo_p = psoc4_info->legacy_family ? p4_legacy_algo : p4_current_algo;
	const size_t algo_size =
		psoc4_info->legacy_family ? sizeof(p4_legacy_algo) : sizeof (p4_current_algo);

	int hr, hr1 = 0, hr2 = 0, hr3 = 0;

	/* Data size should be aligned and padded by the infrastructure */
	assert(count % psoc4_info->row_size == 0);
	assert(offset % psoc4_info->row_size == 0);

	/* Set 48 MHz IMO clock on some parts */
	if (psoc4_info->imo_required) {
		hr = set_imo_48mhz(bank);
		if (hr != ERROR_OK)
			return hr;
	}

	/* Allocate buffer for the algorithm */
	struct working_area *wa_algorithm;
	hr = target_alloc_working_area(target, algo_size, &wa_algorithm);
	if (hr != ERROR_OK)
		return hr;

	/* Write the algorithm code */
	hr = target_write_buffer(target, wa_algorithm->address, algo_size, algo_p);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Allocate buffer for the stack */
	struct working_area *wa_stack;
	hr = target_alloc_working_area(target, RAM_STACK_WA_SIZE, &wa_stack);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Get remaining size of the Working RAM (minus 8 bytes for the header) */
	uint32_t buffer_size = target_get_working_area_avail(target) - 8u;

	/* Round down to the multiple of page size */
	buffer_size &= ~(psoc4_info->row_size - 1);
	if (buffer_size < psoc4_info->row_size) {
		LOG_ERROR("Failed to allocate ping-pong Buffer, add some working area");
		hr = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto err_free_wa_stack;
	}

	/* Allocate the ping-pong buffer */
	struct working_area *wa_buffer;
	hr = target_alloc_working_area(target, buffer_size + 8, &wa_buffer);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	LOG_DEBUG("Allocated %u bytes for ping-pong buffer (%d rows)",
		buffer_size, buffer_size / psoc4_info->row_size);

	/* Initialize the buffer before starting the algo */
	uint32_t pp_header[2] = {0, CMD_BUFFER_EMPTY};
	hr = target_write_buffer(target, wa_buffer->address, sizeof(pp_header),
			(uint8_t *)pp_header);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Pass all parameters to the entry point (see psoc4_write.c) */
	struct reg_param reg_params[5];
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, wa_buffer->address);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, (offset / psoc4_info->row_size) | sflash_mask);

	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, psoc4_info->row_size);

	/* TODO: Don't know why hardcoded value works, even if it is not correct...
	 * Lets just pass it as a parameter to not hardcode it in the algo */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	buf_set_u32(reg_params[3].value, 0, 32, PSOC4_ROWS_PER_MACRO);

	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[4].value, 0, 32, wa_stack->address + wa_stack->size);

	/* Start the algorithm in the beckground */
	struct armv7m_algorithm armv7m_algo;
	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;
	hr = target_start_algorithm(target, 0, NULL, 5, reg_params, wa_algorithm->address,
			0, &armv7m_algo);
	if (hr != ERROR_OK)
		goto err_free_regs_wa_buffer;

	const uint32_t rows_in_buffer = buffer_size / psoc4_info->row_size;

	progress_init(count, operation);
	while (count) {
		uint32_t rows_thisrun = count / psoc4_info->row_size;
		if (rows_thisrun > rows_in_buffer)
			rows_thisrun = rows_in_buffer;
		const uint32_t bytes_thisrun = rows_thisrun * psoc4_info->row_size;

		hr = target_write_buffer(target, wa_buffer->address + 8, bytes_thisrun, buffer);
		if (hr != ERROR_OK)
			goto err_stop_algorithm;

		target_buffer_set_u32(target, (uint8_t *)&pp_header[0], rows_thisrun);
		target_buffer_set_u32(target, (uint8_t *)&pp_header[1], CMD_BUFFER_FULL);

		hr = target_write_buffer(target, wa_buffer->address, sizeof(pp_header),
				(uint8_t *)pp_header);
		if (hr != ERROR_OK)
			goto err_stop_algorithm;

		while (pp_header[1] == CMD_BUFFER_FULL) {
			hr = target_read_u32(target, wa_buffer->address + 4, &pp_header[1]);
			if (hr != ERROR_OK)
				goto err_stop_algorithm;
		}

		if ((pp_header[1] & PSOC4_SROM_STATUS_MASK) == PSOC4_SROM_STATUS_FAILED) {
			LOG_ERROR("Flash algorithm reported failure - \"%s\"", psoc4_decode_sysreq_error(pp_header[1]));
			hr = ERROR_FLASH_OPERATION_FAILED;
			goto err_stop_algorithm;
		}

		count -= bytes_thisrun;
		buffer += bytes_thisrun;
		progress_left(count);
	}

err_stop_algorithm:
	hr1 = target_write_u32(target, wa_buffer->address + 4, CMD_BUFFER_DONE);
	hr2 = target_wait_algorithm(target, 0, NULL, 5, reg_params, 0, 5000, &armv7m_algo);

err_free_regs_wa_buffer:
	for (size_t i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);
	target_free_working_area(target, wa_buffer);

err_free_wa_stack:
	target_free_working_area(target, wa_stack);

err_free_wa_algo:
	target_free_working_area(target, wa_algorithm);

	/* Disable 0x0000:0000 - 0x0000:0007 remap to ROM */
	hr3 = target_write_u32(target, psoc4_info->cpuss_sysreq_addr, PSOC4_DIS_RESET_VECT_REL_BIT);

	int real_hr = hr ? hr : hr1 ? hr1 : hr2 ? hr2 : hr3;
	progress_done(real_hr);
	return real_hr;
}


static int psoc4_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return psoc4_write_inner(bank, buffer, offset, count, PROGRAMMING);
}


/*************************************************************************************************
 * @brief Perform writes to the psoc4_flash_prot bank
 * @param bank current flash bank
 * @param buffer pointer to the buffer with flash protection data
 * @param offset starting offset in flash protection where buffer will be written
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_flash_prot_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	int retval = psoc4_flash_prepare(bank);
	if (retval != ERROR_OK)
		return retval;

	uint32_t bytes_per_macro = psoc4_info->die_num_rows / psoc4_info->num_macros / 8;
	uint32_t cur_macro = offset / bytes_per_macro;
	offset = offset % bytes_per_macro;

	while (count)
	{
		uint32_t bytes_this_run = bytes_per_macro - offset < count ? bytes_per_macro - offset : count;
		retval = write_flash_macro_protection(bank, PSOC4_CHIP_PROT_OPEN, buffer, bytes_this_run, offset, cur_macro++);
		if (retval != ERROR_OK)
			break;

		offset = 0;
		buffer += bytes_this_run;
		count -= bytes_this_run;
	}

	/* Disable 0x0000:0000 - 0x0000:0007 remap to ROM */
	if (retval == ERROR_OK)
		retval = target_write_u32(bank->target, psoc4_info->cpuss_sysreq_addr,
				PSOC4_SROM_HMASTER_BIT | PSOC4_DIS_RESET_VECT_REL_BIT);

	return retval;
}


/*************************************************************************************************
 * @brief Erases entire psoc4_flash_prot bank
 * @param bank current flash bank
 * @param first the first sector to erase
 * @param last the last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_flash_prot_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	uint32_t prot_sz = psoc4_info->num_rows / psoc4_info->num_macros / 8;

	if (!psoc4_info->probed)
		return ERROR_FAIL;

	uint8_t empty[prot_sz];
	memset(empty, 0, sizeof(empty));

	int retval = psoc4_flash_prepare(bank);
	if (retval != ERROR_OK)
		return retval;

	for(unsigned int macro = 0; macro < psoc4_info->num_macros; ++macro) {
		retval = write_flash_macro_protection(bank, PSOC4_CHIP_PROT_OPEN, empty, prot_sz, 0, macro);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Disable 0x0000:0000 - 0x0000:0007 remap to ROM */
	if (retval == ERROR_OK)
		retval = target_write_u32(target, psoc4_info->cpuss_sysreq_addr,
				PSOC4_SROM_HMASTER_BIT | PSOC4_DIS_RESET_VECT_REL_BIT);

	return retval;
}


static int psoc4_sflash_erase(struct flash_bank *bank, int first, int last)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	/* SFlash can not be erased so simply fill it with zeros */
		uint8_t empty[bank->size];
		memset(empty, 0, sizeof(empty));
		return psoc4_write_inner(bank, empty,
				first * psoc4_info->row_size,
				(last - first + 1) * psoc4_info->row_size,
				ERASING);
}


/**************************************************************************************************
 * @brief Performs mass erase operation. Page erase is not supported in PSoC4
 * @param bank current flash bank
 * @param first The first sector to erase
 * @param last The last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	if (is_sflash_bank(bank))
		return psoc4_sflash_erase(bank, first, last);

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return psoc4_mass_erase(bank);

	LOG_WARNING("Only mass erase available, erase skipped! (psoc4 mass_erase <bank_id>)");
	return ERROR_OK;
}

/**************************************************************************************************
 * Rounds the 32-bit integer up to the next highest power of two, e.g.
 * 1000 -> 1024, 1024 -> 1024, 2040 -> 2048 etc.
 * Borrowed from the 'Bit Twiddling Hacks', see
 * https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
 *************************************************************************************************/
static inline uint32_t next_pow2(uint32_t val)
{
	assert(val != 0);

	val--;
	val |= (val >> 1);
	val |= (val >> 2);
	val |= (val >> 4);
	val |= (val >> 8);
	val |= (val >> 16);

	return val + 1;
}

/**************************************************************************************************
 * @brief Reads flash wounding information from the SFlash
 * @param bank current flash bank
 * @param wounding pointer to variable, will be populated with wounding value.
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static uint32_t psoc4_get_wounded_flash_size(struct flash_bank *bank, uint32_t die_flash_size, uint32_t *wounded_flash)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t wounding_addr = psoc4_info->sflash_descr->wounding_offs + psoc4_info->sflash_base;
	uint32_t wounding;

	*wounded_flash = die_flash_size;
	int retval = target_read_u32(target, wounding_addr, &wounding);
	if (retval != ERROR_OK) {
		return retval;
	}

	wounding = (wounding >> 20) & 0x07; /* 22:20 stores FLASH wounding bits */

	/* Wounding should be calculated from the next power of two*/
	if (wounding > 0)
	{
		die_flash_size = next_pow2(die_flash_size);
		*wounded_flash = die_flash_size >> wounding;
	}


	return ERROR_OK;
}


/**************************************************************************************************
 * @brief Detects psoc4 family and reads geometry from the SPCIF_GEOMETRY register
 * @param bank current flash bank
 * @param family_id pointer to variable, will be populated with family id.
 * @param revision_id pointer to variable, will be populated with revision id.
 * @param flash_size_in_kb_out pointer to variable, will be populated with entire flash size in KB.
 * @param row_size_out pointer to variable, will be populated with row size.
 * @param num_macros_out pointer to variable, will be populated with amount of macro.
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_read_geometry(struct flash_bank *bank, uint16_t *family_id, uint16_t *revision_id,
									uint32_t *flash_size_in_kb_out, uint32_t *row_size_out, uint32_t *num_macros_out)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t flash_size_in_kb = 0;
	uint32_t row_size = 0;
	uint32_t num_macros = 0;

	assert(family_id != NULL);

	int retval = psoc4_get_family(target, family_id, revision_id);
	if (retval != ERROR_OK)
		goto exit;

	const struct psoc4_chip_family *family = psoc4_family_by_id(*family_id);
	if (family->id == 0) {
		LOG_ERROR("PSoC 4 family with id 0x%02X is not supported.", *family_id);
		retval = ERROR_FAIL;
		goto exit;
	}

	if (is_sflash_base_v2(*family_id)) {
		psoc4_info->sflash_base = PSOC4_SFLASH_MACRO0_v2;
	}
	else{
		psoc4_info->sflash_base = PSOC4_SFLASH_MACRO0;
	}

	assert(family->spcif_ver >= spcif_v2 && family->spcif_ver <= spcif_v3);
	if (family->spcif_ver == spcif_v2) {
		psoc4_info->spcif = &spcif_v2_t;
	}
	else if (family->spcif_ver == spcif_v3) {
		psoc4_info->spcif = &spcif_v3_t;
	}

	if (family->flags & PSOC4_FAMILY_FLAG_LEGACY) {
		psoc4_info->legacy_family = true;
		psoc4_info->cpuss_sysreq_addr = PSOC4_CPUSS_SYSREQ_LEGACY;
		psoc4_info->cpuss_sysarg_addr = PSOC4_CPUSS_SYSARG_LEGACY;
		psoc4_info->spcif->geometry_addr = PSOC4_SPCIF_GEOMETRY_LEGACY;
	} else {
		psoc4_info->legacy_family = false;
		psoc4_info->cpuss_sysreq_addr = PSOC4_CPUSS_SYSREQ_NEW;
		psoc4_info->cpuss_sysarg_addr = PSOC4_CPUSS_SYSARG_NEW;
		psoc4_info->spcif->geometry_addr = PSOC4_SPCIF_GEOMETRY_NEW;
	}

	psoc4_info->imo_required = true;
	if (family->flags & PSOC4_FLAG_IMO_NOT_REQUIRED)
		psoc4_info->imo_required = false;

	uint32_t spcif_geometry;
	retval = target_read_u32(target, psoc4_info->spcif->geometry_addr, &spcif_geometry);
	if (retval != ERROR_OK)
		goto exit;

	/* Retrieve flash geometry from the dedicated register */
	flash_size_in_kb = (spcif_geometry & psoc4_info->spcif->geometry_flash_mask) >> psoc4_info->spcif->geometry_flash_pos;
	row_size = (spcif_geometry & psoc4_info->spcif->geometry_flash_rows_mask) >> psoc4_info->spcif->geometry_flash_rows_pos;
	num_macros = (spcif_geometry & psoc4_info->spcif->geometry_flash_macro_mask) >> psoc4_info->spcif->geometry_flash_macro_pos;

	if (psoc4_info->legacy_family) {
		flash_size_in_kb = flash_size_in_kb * 256 / 1024;
		row_size *= 128;
	} else {
		flash_size_in_kb = (flash_size_in_kb + 1) * 256 / 1024;
		row_size = 64 * (row_size + 1);
		num_macros++;
	}

exit:
	if (flash_size_in_kb_out)
		*flash_size_in_kb_out = flash_size_in_kb;
	if (row_size_out)
		*row_size_out = row_size;
	if (num_macros_out)
		*num_macros_out = num_macros;

	return retval;
}


/**************************************************************************************************
 * @brief Determines bank size based on UDD if device exists there or read from silicon
 * and prints detailed info about the connected silicon
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
void psoc4_decode_silicon_info(struct flash_bank *bank)
{
	static bool g_info_displayed = false;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t flash_size_in_kb, row_size, num_macros;
	uint16_t si_family_id, si_revision_id;
	uint16_t si_siid;
	uint8_t protection;

	if (g_info_displayed)
		return;

	enum log_levels log_level = change_debug_level(LOG_LVL_USER);

	/* Retrieve family and revision IDs */
	int hr = psoc4_read_geometry(bank, &si_family_id, &si_revision_id, &flash_size_in_kb, &row_size, &num_macros);
	if (hr != ERROR_OK) {
		/* if it fails, chip must be protected, lets check by reading a word from the flash */
		uint32_t val;
		int retval = target_read_u32(target, bank->base, &val);
		if (retval == ERROR_OK)
			goto error_exit;

		/* If flash read fails chip is probably protected, lets check by running a SROM API */
		retval = psoc4_get_silicon_id(bank, true, NULL, NULL, &protection);
		if (retval == ERROR_OK && protection == PSOC4_CHIP_PROT_PROTECTED)
		{
			LOG_USER("Warn:\tChip protection state is %s. Chip's resources access is locked down.", psoc4_decode_chip_protection(protection));
			LOG_USER("\tThe state can be set back to OPEN but only after completely erasing the flash.");
			LOG_USER("\tTo do so, perform erase operation with PSOC4_USE_MEM_AP variable set.");

			g_info_displayed = true;
		}
		goto error_exit;
	}

	LOG_DEBUG("SPCIF geometry: %" PRIu32 " kb flash, row %" PRIu32 " bytes.",
		flash_size_in_kb, row_size);

	/* Read silicon id from SFLASH */
	uint32_t tmp = 0;
	psoc4_info->sflash_descr = psoc4_sflash_by_geometry(row_size, num_macros);
	hr = target_read_u32(target, psoc4_info->sflash_descr->siid_offs + psoc4_info->sflash_base, &tmp);
	if (hr != ERROR_OK)
		goto error_exit;

	si_siid = tmp & 0xFFFF;  /* 15:0 */

	/* Read Protection and translate into format returned by SROM API*/
	hr = target_read_u32(target, psoc4_info->sflash_descr->chip_protection_offs + psoc4_info->sflash_base, &tmp);
	if (hr != ERROR_OK)
		goto error_exit;

	protection = (tmp >> 24) & 0x0f; /* 7:0 */
	if (protection == PSOC4_CHIP_PROT_VIRGIN) protection = PSOC4_CHIP_PROT_OPEN;
	else if (protection == PSOC4_CHIP_PROT_OPEN) protection = PSOC4_CHIP_PROT_VIRGIN;

	const char *tcl =
			"source [find target/cympn.cfg]; "
			"set siid \"%04X\"; "
			"if { [dict exists $MPN $siid ] } {"
			"  set pn [lindex $MPN($siid) 0]; "
			"  set main_size [lindex $MPN($siid) 2]; "
			"  return [list $pn $main_size]; "
			"}";

	char * si_mpn = NULL;
	uint32_t udd_mflash_size = 0;
	char *command = alloc_printf(tcl, si_siid);

	extern struct command_context *global_cmd_ctx;
	Jim_Interp *interp = global_cmd_ctx->interp;
	Jim_Eval(interp, command);
	Jim_Obj *list = Jim_GetResult(interp);
	if(Jim_IsList(list) && Jim_ListLength(interp, list) == 2) {
		si_mpn = strdup(Jim_GetString(Jim_ListGetIndex(interp, list, 0), NULL));
		long l;
		Jim_GetLong(interp, Jim_ListGetIndex(interp, list, 1), &l);
		udd_mflash_size = (uint32_t)(l * 1024u);
	}

	free(command);

	char si_rev_major = "XABCDEFGHIJKLMNOP"[si_revision_id >> 4u];
	char si_rev_minor = "X0123456789abcdef"[si_revision_id & 0x0F];
	LOG_USER("*****************************************");
	LOG_USER("** Silicon: 0x%04X, Family: 0x%02X, Rev.: 0x%02X (%c%c)",
			si_siid, si_family_id, si_revision_id, si_rev_major, si_rev_minor);

	if (si_mpn) {
		LOG_USER("** Detected Device: %s", si_mpn);
	}
	/* if device is not available in UDD, read wounding register to get the real flash size */
	else {
		psoc4_get_wounded_flash_size(bank, flash_size_in_kb * 1024u, &udd_mflash_size);
	}
	LOG_USER("** Detected Family: %s", psoc4_family_by_id(si_family_id)->name);
	LOG_USER("** Detected Main Flash size, kb: %d", udd_mflash_size / 1024u);
	LOG_USER("** Chip Protection: %s", psoc4_decode_chip_protection(protection));
	LOG_USER("*****************************************");

	g_info_displayed = true;
	if (!psoc4_info->user_bank_size)
		psoc4_info->user_bank_size = udd_mflash_size;

	if (si_mpn)
		free(si_mpn);

error_exit:
	change_debug_level(log_level);
}


/**************************************************************************************************
 * @brief Probes the device and populates related data structures with target flash geometry data.
 * This is done by reading SPCIF_GEOMETRY register describing available amount of memory.
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_probe(struct flash_bank *bank)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	int retval;
	uint16_t family_id;
	uint16_t revision_id;
	uint32_t flash_size_in_kb;
	uint32_t row_size;
	uint32_t num_macros;

	psoc4_info->probed = false;

	retval = psoc4_read_geometry(bank, &family_id, &revision_id, &flash_size_in_kb, &row_size, &num_macros);
	if (retval != ERROR_OK) {
		/* fake coreid indicates that the mem_ap is currently used to unprotect the device.
		 * Do not return an error in this case */
		if (bank->target->coreid == 0xff) {
			psoc4_info->probed = true;
			return ERROR_OK;
		}
		return retval;
	}

	psoc4_info->sflash_descr = psoc4_sflash_by_geometry(row_size, num_macros);

	/* calculate number of pages */
	uint32_t num_rows = flash_size_in_kb * 1024 / row_size;
	psoc4_info->die_num_rows = num_rows;

	/* check number of flash macros */
	if (num_macros != (num_rows + PSOC4_ROWS_PER_MACRO - 1) / PSOC4_ROWS_PER_MACRO)
		LOG_WARNING("Number of macros does not correspond with flash size!");

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (psoc4_info->user_bank_size && !is_sflash_bank(bank)) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = psoc4_info->user_bank_size / 1024;
		num_rows = flash_size_in_kb * 1024 / row_size;
	}
	else {
		uint32_t mflash_size;
		psoc4_get_wounded_flash_size(bank, flash_size_in_kb * 1024u, &mflash_size);
		num_rows = mflash_size / row_size;
	}

	if (bank->sectors)
		free(bank->sectors);

	if (is_sflash_bank(bank)) {
		num_rows = 4;
		num_macros = 1;

		if (row_size == 64)
			num_rows = 0;

		if (row_size == 256)
			bank->base = psoc4_info->sflash_base + 0x400;
		else
			bank->base = psoc4_info->sflash_base + 0x200;
	}

	psoc4_info->family_id = family_id;
	psoc4_info->num_macros = num_macros;
	psoc4_info->row_size = row_size;
	psoc4_info->num_rows = num_rows;
	bank->size = num_rows * row_size;
	bank->num_sectors = num_rows;
	bank->sectors = alloc_block_array(0, row_size, num_rows);
	if (bank->sectors == NULL)
		return ERROR_FAIL;

	bank->write_start_alignment = row_size;
	bank->write_end_alignment = row_size;

	LOG_DEBUG("flash bank set %" PRIu32 " rows", num_rows);
	psoc4_info->probed = true;

	return ERROR_OK;
}

static int psoc4_auto_probe(struct flash_bank *bank)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	if (psoc4_info->probed)
		return ERROR_OK;
	return psoc4_probe(bank);
}


/**************************************************************************************************
 * @brief Wraps original psoc4_probe to configure flash protection bank.
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_flash_prot_probe(struct flash_bank *bank)
{
	int retval = psoc4_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	if (bank->sectors)
		free(bank->sectors);

	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	bank->size = psoc4_info->num_rows / 8;
	bank->num_sectors = 1;
	bank->sectors = alloc_block_array(0, bank->size, bank->num_sectors);
	if (bank->sectors == NULL)
		return ERROR_FAIL;

	bank->write_start_alignment = FLASH_WRITE_ALIGN_SECTOR;
	bank->write_end_alignment = FLASH_WRITE_ALIGN_SECTOR;
	bank->is_memory_mapped = false;

	psoc4_info->probed = true;

	return ERROR_OK;
}

static int psoc4_flash_prot_auto_probe(struct flash_bank *bank)
{
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;
	if (psoc4_info->probed)
		return ERROR_OK;
	return psoc4_flash_prot_probe(bank);
}


/**************************************************************************************************
 * @brief Display human-readable information about the flash bank into the given buffer.
 * @param bank current flash bank
 * @param buf - where to put the text for the human to read
 * @param buf_size - the size of the human buffer.
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int get_psoc4_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	struct psoc4_flash_bank *psoc4_info = bank->driver_priv;

	if (!psoc4_info->probed)
		return ERROR_FAIL;

	const struct psoc4_chip_family *family = psoc4_family_by_id(psoc4_info->family_id);
	uint32_t size_in_kb = bank->size / 1024;

	if (target->state != TARGET_HALTED) {
		snprintf(buf, buf_size, "%s, flash %" PRIu32 " kb"
			" (halt target to see details)", family->name, size_in_kb);
		return ERROR_OK;
	}

	int retval;
	int printed = 0;
	uint32_t silicon_id;
	uint16_t family_id;
	uint8_t protection;

	retval = psoc4_get_silicon_id(bank, false, &silicon_id, &family_id, &protection);
	if (retval != ERROR_OK)
		return retval;

	if (family_id != psoc4_info->family_id)
		printed = snprintf(buf, buf_size, "Family id mismatch 0x%02" PRIx16
				"/0x%02" PRIx16 ", silicon id 0x%08" PRIx32,
				psoc4_info->family_id, family_id, silicon_id);
	else {
		printed = snprintf(buf, buf_size, "%s silicon id 0x%08" PRIx32 "",
				family->name, silicon_id);
	}

	buf += printed;
	buf_size -= printed;

	const char *prot_txt = psoc4_decode_chip_protection(protection);
	snprintf(buf, buf_size, ", flash %" PRIu32 " kb %s", size_in_kb, prot_txt);
	return ERROR_OK;
}

COMMAND_HANDLER(psoc4_handle_reset_halt)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	return psoc4_reset_halt(target);
}

COMMAND_HANDLER(psoc4_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (is_flash_prot_bank(bank))
		retval = psoc4_flash_prot_erase(bank, 0, bank->num_sectors - 1);
	else if (is_sflash_bank(bank))
		retval = psoc4_sflash_erase(bank, 0, bank->num_sectors - 1);
	else
		retval = psoc4_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "psoc mass erase complete");
	else
		command_print(CMD, "psoc mass erase failed");

	return retval;
}

COMMAND_HANDLER(psoc4_handle_silicon_info_command)
{
	struct flash_bank *bank;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bank = get_flash_bank_by_num_noprobe(0);
	if (!bank)
		return ERROR_FAIL;

	psoc4_decode_silicon_info(bank);

	return ERROR_OK;
}

static const struct command_registration psoc4_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = psoc4_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "reset_halt",
		.handler = psoc4_handle_reset_halt,
		.mode = COMMAND_EXEC,
		.usage = "psoc4 reset_halt",
		.help = "Tries to simulate broken Vector Catch",
	},
	{
		.name = "silicon_info",
		.handler = psoc4_handle_silicon_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Prints detailed info about the connected silicon",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc4_command_handlers[] = {
	{
		.name = "psoc4",
		.mode = COMMAND_ANY,
		.help = "PSoC 4 flash command group",
		.usage = "",
		.chain = psoc4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver psoc4_flash = {
	.name = "psoc4",
	.commands = psoc4_command_handlers,
	.flash_bank_command = psoc4_flash_bank_command,
	.erase = psoc4_erase,
	.protect = psoc4_protect,
	.write = psoc4_write,
	.read = default_flash_read,
	.probe = psoc4_probe,
	.auto_probe = psoc4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = psoc4_protect_check,
	.info = get_psoc4_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

const struct flash_driver psoc4_flash_prot = {
	.name = "psoc4_flash_prot",
	.commands = psoc4_command_handlers,
	.flash_bank_command = psoc4_flash_bank_command,
	.erase = psoc4_flash_prot_erase,
	.protect = NULL,
	.write = psoc4_flash_prot_write,
	.read = psoc4_flash_prot_read,
	.probe = psoc4_flash_prot_probe,
	.auto_probe = psoc4_flash_prot_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.info = get_psoc4_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
