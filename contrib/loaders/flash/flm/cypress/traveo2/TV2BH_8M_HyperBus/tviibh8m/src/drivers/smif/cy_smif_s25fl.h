/***************************************************************************//**
* \file cy_smif_s25fl.c
* \version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
*
* Provides an API declaration of the Cypress S25FL HSSPI series driver.
*
********************************************************************************
* \copyright
* Copyright 2016-2018, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_SMIF_S25FL_H)
#define CY_SMIF_S25FL_H

#if defined(__cplusplus)
extern "C" {
#endif

#define S25FL_USE_LC_ENHANCED

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "cy_device_headers.h"
#include "cy_smif.h"

typedef enum
{
    CY_SMIF_CMD_READ_ID    = 0x90, // Read Electronic Manufacturer Signature
    CY_SMIF_CMD_RDID       = 0x9F, // Read ID (JEDEC Manufacturer ID and JEDEC CFI)
    CY_SMIF_CMD_RES        = 0xAB, // Read Electronic Signature 
    CY_SMIF_CMD_RDSR1      = 0x05, // Read Status Register-1 
    CY_SMIF_CMD_RDSR2      = 0x07, // Read Status Register-2 
    CY_SMIF_CMD_RDCR       = 0x35, // Read Configuration Register-1 
    CY_SMIF_CMD_WRR        = 0x01, // Write Register (Status-1, Configuration-1) 
    CY_SMIF_CMD_WRDI       = 0x04, // Write Disable 
    CY_SMIF_CMD_WREN       = 0x06, // Write Enable 
    CY_SMIF_CMD_CLSR       = 0x30, // Clear Status Register-1 - Erase/Prog. Fail Reset 
    CY_SMIF_CMD_ABRD       = 0x14, // AutoBoot Register Read 
    CY_SMIF_CMD_ABWR       = 0x15, // AutoBoot Register Write 
    CY_SMIF_CMD_BRRD       = 0x16, // Bank Register Read 
    CY_SMIF_CMD_BRWR       = 0x17, // Bank Register Write 
    CY_SMIF_CMD_BRAC       = 0xB9, // Bank Register Access (Legacy Command formerly used for Deep Power Down)
    CY_SMIF_CMD_DLPRD      = 0x41, // Data Learning Pattern Read 
    CY_SMIF_CMD_PNVDLR     = 0x43, // Program NV Data Learning Register 
    CY_SMIF_CMD_WVDLR      = 0x4A, // Write Volatile Data Learning Register 
    CY_SMIF_CMD_READ       = 0x03, // Read (3- or 4-byte address) 
    CY_SMIF_CMD_4READ      = 0x13, // Read (4-byte address) 
    CY_SMIF_CMD_FAST_READ  = 0x0B, // Fast Read (3- or 4-byte address) 
    CY_SMIF_CMD_4FAST_READ = 0x0C, // Fast Read (4-byte address) 
    CY_SMIF_CMD_DDRFR      = 0x0D, // DDR Fast Read (3- or 4-byte address)
    CY_SMIF_CMD_4DDRFR     = 0x0E, // DDR Fast Read (4-byte address) 
    CY_SMIF_CMD_DOR        = 0x3B, // Read Dual Out (3- or 4-byte address) 
    CY_SMIF_CMD_4DOR       = 0x3C, // Read Dual Out (4-byte address) 
    CY_SMIF_CMD_QOR        = 0x6B, // Read Quad Out (3- or 4-byte address)
    CY_SMIF_CMD_4QOR       = 0x6C, // Read Quad Out (4-byte address) 
    CY_SMIF_CMD_DIOR       = 0xBB, // Dual I/O Read (3- or 4-byte address) 
    CY_SMIF_CMD_4DIOR      = 0xBC, // Dual I/O Read (4-byte address) 
    CY_SMIF_CMD_DDRDIOR    = 0xBD, // DDR Dual I/O Read (3- or 4-byte address)
    CY_SMIF_CMD_4DDRDIOR   = 0xBE, // DDR Dual I/O Read (4-byte address) 
    CY_SMIF_CMD_QIOR       = 0xEB, // Quad I/O Read (3- or 4-byte address)
    CY_SMIF_CMD_4QIOR      = 0xEC, // Quad I/O Read (4-byte address) 
    CY_SMIF_CMD_DDRQIOR    = 0xED, // DDR Quad I/O Read (3- or 4-byte address) 
    CY_SMIF_CMD_4DDRQIOR   = 0xEE, // DDR Quad I/O Read (4-byte address) 
    CY_SMIF_CMD_PP         = 0x02, // Page Program (3- or 4-byte address) 
    CY_SMIF_CMD_4PP        = 0x12, // Page Program (4-byte address) 
    CY_SMIF_CMD_QPP        = 0x32, // Quad Page Program (3- or 4-byte address) 
    CY_SMIF_CMD_QPPA       = 0x38, // Quad Page Program - Alternate instruction (3- or 4-byte address)
    CY_SMIF_CMD_4QPP       = 0x34, // Quad Page Program (4-byte address) 
    CY_SMIF_CMD_PGSP       = 0x85, // Program Suspend 
    CY_SMIF_CMD_PGRS       = 0x8A, // Program Resume 
    CY_SMIF_CMD_P4E        = 0x20, // Parameter 4-kB, sector Erase (3- or 4-byte address)
    CY_SMIF_CMD_4P4E       = 0x21, // Parameter 4-kB, sector Erase (4-byte address)
    CY_SMIF_CMD_BE         = 0x60, // Bulk Erase 
    CY_SMIF_CMD_BEA        = 0xC7, // Bulk Erase (alternate command)
    CY_SMIF_CMD_SE         = 0xD8, // Erase 64 kB or 256 kB (3- or 4-byte address)
    CY_SMIF_CMD_4SE        = 0xDC, // Erase 64 kB or 256 kB (4-byte address) 
    CY_SMIF_CMD_ERSP       = 0x75, // Erase Suspend 
    CY_SMIF_CMD_ERRS       = 0x7A, // Erase Resume 
    CY_SMIF_CMD_OTPP       = 0x42, // OTP Program 
    CY_SMIF_CMD_OTPR       = 0x4B, // OTP Read 
    CY_SMIF_CMD_DYBRD      = 0xE0, // DYB Read 
    CY_SMIF_CMD_DYBWR      = 0xE1, // DYB Write 
    CY_SMIF_CMD_PPBRD      = 0xE2, // PPB Read 
    CY_SMIF_CMD_PPBP       = 0xE3, // PPB Program 
    CY_SMIF_CMD_PPBE       = 0xE4, // PPB Erase 
    CY_SMIF_CMD_ASPRD      = 0x2B, // ASP Read 
    CY_SMIF_CMD_ASPP       = 0x2F, // ASP Program 
    CY_SMIF_CMD_PLBRD      = 0xA7, // PPB Lock Bit Read 
    CY_SMIF_CMD_PLBWR      = 0xA6, // PPB Lock Bit Write 
    CY_SMIF_CMD_PASSRD     = 0xE7, // Password Read 
    CY_SMIF_CMD_PASSP      = 0xE8, // Password Program 
    CY_SMIF_CMD_PASSU      = 0xE9, // Password Unlock 
    CY_SMIF_CMD_RESET      = 0xF0, // Software Reset 
    CY_SMIF_CMD_MBR        = 0xFF, // Mode Bit Reset 
    CY_SMIF_CMD_MPM        = 0xA3, // Reserved for Multi-I/O-High Perf Mode (MPM) 
} en_cy_smif_s25fl128_cmd_t;

typedef struct
{
    uint8_t u1WIP   : 1;
    uint8_t u1WEL   : 1;
    uint8_t u3BP    : 3;
    uint8_t u1E_ERR : 1;
    uint8_t u1P_ERR : 1;
    uint8_t u1SRWD  : 1;
} stc_cy_smif_s25fl128_SR1_t;

typedef struct
{
    uint8_t u1PS  : 1;
    uint8_t u1ES  : 1;
    uint8_t u6RFU : 6;
} stc_cy_smif_s25fl128_SR2_t;


typedef struct
{
    uint8_t u1FREEZE : 1;
    uint8_t u1QUAD   : 1;
    uint8_t u1TBPARM : 1;
    uint8_t u1BPNV   : 1;
    uint8_t u1RFU    : 1;
    uint8_t u1TBPROT : 1;
    uint8_t u2LC     : 2;
} stc_cy_smif_s25fl128_CR1_t;

typedef struct
{
    uint8_t u1BA24   : 1;
    uint8_t u6RFU    : 6;
    uint8_t u1EXTADD : 1;
} stc_cy_smif_s25fl128_BAR_t;

typedef struct
{
    uint32_t u1ABE   : 1;
    uint32_t u8ABSD  : 8;
    uint32_t u23ABSA : 23;
} stc_cy_smif_s25fl128_ABR_t;

typedef union
{
    uint8_t                    u8;
    stc_cy_smif_s25fl128_SR1_t field;
} un_cy_smif_s25fl128_SR1_t;

typedef union
{
    uint8_t                    u8;
    stc_cy_smif_s25fl128_SR2_t field;
} un_cy_smif_s25fl128_SR2_t;

typedef union
{
    uint8_t                    u8;
    stc_cy_smif_s25fl128_BAR_t field;
} un_cy_smif_s25fl128_BAR_t;


typedef union
{
    uint8_t                    u8;
    stc_cy_smif_s25fl128_CR1_t field;
} un_cy_smif_s25fl128_CR1_t;

typedef union
{
    uint32_t                     u32;
    stc_cy_smif_s25fl128_ABR_t field;
} un_cy_smif_s25fl128_ABR_t;

typedef enum
{
    /* During the Quad I/O Read Command Sequence, 
       if the Mode bits are any value other than Axh, 
       then the next time CS# is raised high the device will be 
       released from Quad I/O High Performance Read mode.
       During the DDR Quad I/O Read Command Sequence, 
       If the upper nibble (IO[7:4]) and lower nibble (IO[3:0]) 
       of the Mode bits are complementary (i.e. 5h and Ah) the device 
       transitions to Continuous Read DDR Quad I/O Mode
    */
    CY_SMIF_XIP_RW_CONTINUOUS     = 0xA5, 
    CY_SMIF_XIP_RW_NON_CONTINUOUS = 0x00,
} cy_en_smif_rw_mode_t;

typedef enum
{
    CY_SMIF_READ       = CY_SMIF_CMD_READ,       // Read (3- or 4-byte address) 
    CY_SMIF_4READ      = CY_SMIF_CMD_4READ,      // Read (4-byte address) 
    CY_SMIF_FAST_READ  = CY_SMIF_CMD_FAST_READ,  // Fast Read (3- or 4-byte address) 
    CY_SMIF_4FAST_READ = CY_SMIF_CMD_4FAST_READ, // Fast Read (4-byte address) 
    CY_SMIF_DOR        = CY_SMIF_CMD_DOR,        // Read Dual Out (3- or 4-byte address) 
    CY_SMIF_4DOR       = CY_SMIF_CMD_4DOR,       // Read Dual Out (4-byte address) 
    CY_SMIF_QOR        = CY_SMIF_CMD_QOR,        // Read Quad Out (3- or 4-byte address)
    CY_SMIF_4QOR       = CY_SMIF_CMD_4QOR,       // Read Quad Out (4-byte address) 
    CY_SMIF_DIOR       = CY_SMIF_CMD_DIOR,       // Dual I/O Read (3- or 4-byte address) 
    CY_SMIF_4DIOR      = CY_SMIF_CMD_4DIOR,      // Dual I/O Read (4-byte address) 
    CY_SMIF_QIOR       = CY_SMIF_CMD_QIOR,       // Quad I/O Read (3- or 4-byte address)
    CY_SMIF_4QIOR      = CY_SMIF_CMD_4QIOR,      // Quad I/O Read (4-byte address) 
    CY_SMIF_DDRFR      = CY_SMIF_CMD_DDRFR,      // DDR Fast Read (3- or 4-byte address)
    CY_SMIF_4DDRFR     = CY_SMIF_CMD_4DDRFR,     // DDR Fast Read (4-byte address) 
    CY_SMIF_DDRDIOR    = CY_SMIF_CMD_DDRDIOR,    // DDR Dual I/O Read (3- or 4-byte address) 
    CY_SMIF_4DDRDIOR   = CY_SMIF_CMD_4DDRDIOR,   // DDR Dual I/O Read (4-byte address) 
    CY_SMIF_DDRQIOR    = CY_SMIF_CMD_DDRQIOR,    // DDR Quad I/O Read (3- or 4-byte address) 
    CY_SMIF_4DDRQIOR   = CY_SMIF_CMD_4DDRQIOR,   // DDR Quad I/O Read (4-byte address) 
} cy_en_smif_rd_cmd_t;

typedef enum
{
    CY_SMIF_PP   = CY_SMIF_CMD_PP,   // Page Program (3- or 4-byte address) 
    CY_SMIF_4PP  = CY_SMIF_CMD_4PP,  // Page Program (4-byte address) 
    CY_SMIF_QPP  = CY_SMIF_CMD_QPP,  // Quad Page Program (3- or 4-byte address) 
    CY_SMIF_QPPA = CY_SMIF_CMD_QPPA, // Quad Page Program - Alternate instruction (3- or 4-byte address)
    CY_SMIF_4QPP = CY_SMIF_CMD_4QPP, // Quad Page Program (4-byte address) 
} cy_en_smif_wt_cmd_t;

typedef struct
{
    uint8_t Reserved0 : 1;
    uint8_t IsDualQuad: 1;
    uint8_t width     : 2;
    uint8_t dataSel   : 2;
    uint8_t Reserved1 : 2;
} cy_stc_smif_device_param_field_t;

typedef enum
{
    CY_SMIF_DEV_OSPI            = 0x0C,
    CY_SMIF_DEV_D_QUAD_BIT_0To3 = 0x0A,
    CY_SMIF_DEV_D_QUAD_BIT_4To7 = 0x2A,
    CY_SMIF_DEV_QUAD_BIT_0To3   = 0x08,
    CY_SMIF_DEV_QUAD_BIT_4To7   = 0x28,
    CY_SMIF_DEV_DUAL_BIT_0To1   = 0x04,
    CY_SMIF_DEV_DUAL_BIT_2To3   = 0x14,
    CY_SMIF_DEV_DUAL_BIT_4To5   = 0x24,
    CY_SMIF_DEV_DUAL_BIT_6To7   = 0x34,
    CY_SMIF_DEV_SINGLE_BIT_0To1 = 0x00,
    CY_SMIF_DEV_SINGLE_BIT_2To3 = 0x10,
    CY_SMIF_DEV_SINGLE_BIT_4To5 = 0x20,
    CY_SMIF_DEV_SINGLE_BIT_6To7 = 0x30,
} cy_en_smif_device_type_t;

typedef union
{
    cy_en_smif_device_type_t         en;
    cy_stc_smif_device_param_field_t bits;
} cy_un_smif_device_type_t;

typedef enum
{
    CY_SMIF_SPI_LATENCY0 = 3u,
    CY_SMIF_SPI_LATENCY1 = 0u,
    CY_SMIF_SPI_LATENCY2 = 1u,
    CY_SMIF_SPI_LATENCY3 = 2u,
} cy_en_smif_spi_latency_code_t;

typedef enum
{
    CY_SMIF_BLOCKING     = 0u,
    CY_SMIF_NON_BLOCKING = 1u,
} cy_en_smif_block_mode_t;

typedef struct
{
    cy_en_smif_rd_cmd_t        xipReadCmd;
    cy_en_smif_rw_mode_t       xipReadMode;
    cy_en_smif_wt_cmd_t        xipWriteCmd;
    bool                           mergeTimeoutEnable;
    cy_en_smif_dev_merge_timeout_t mergeTimeout;
    bool                           totalTimeoutEnable;
    uint16_t                       totalTimeout;
    uint32_t                       addr;
    cy_en_device_size_t            size;
    cy_en_smif_spi_latency_code_t  lc_spi;
} cy_stc_device_s25fl_config_t;

typedef struct
{
    uint8_t ids[81];
} cy_stc_JedecId_t;

__STATIC_INLINE cy_en_smif_xip_addr_byte_t CY_SMIF_GetAddrByteFromCmd(uint8_t cmd)
{
    // Assume that EXTADD of Bank Address Register is "0"
    if(  (cmd == CY_SMIF_4READ)      ||
              (cmd == CY_SMIF_4FAST_READ) ||
              (cmd == CY_SMIF_4DDRFR)     ||
              (cmd == CY_SMIF_4DOR)       ||
              (cmd == CY_SMIF_4QOR)       ||
              (cmd == CY_SMIF_4DIOR)      ||
              (cmd == CY_SMIF_4DDRDIOR)   ||
              (cmd == CY_SMIF_4QIOR)      ||
              (cmd == CY_SMIF_4DDRQIOR)   ||
              (cmd == CY_SMIF_4PP)        ||
              (cmd == CY_SMIF_4QPP)       ||
              (cmd == CY_SMIF_4PP)        ||
              (cmd == CY_SMIF_4QPP) )
    {
        return CY_SMIF_XIP_ADDRESS_4_BYTE;
    }
    else if(  (cmd == CY_SMIF_READ)      ||
              (cmd == CY_SMIF_FAST_READ) ||
              (cmd == CY_SMIF_DDRFR)     ||
              (cmd == CY_SMIF_DOR)       ||
              (cmd == CY_SMIF_QOR)       ||
              (cmd == CY_SMIF_DIOR)      ||
              (cmd == CY_SMIF_DDRDIOR)   ||
              (cmd == CY_SMIF_QIOR)      ||
              (cmd == CY_SMIF_PP)        ||
              (cmd == CY_SMIF_QPPA)      ||
              (cmd == CY_SMIF_QPP)       ||
              (cmd == CY_SMIF_PP)        ||
              (cmd == CY_SMIF_QPP) )
    {
        return CY_SMIF_XIP_ADDRESS_3_BYTE;
    }
    else
    {
        CY_ASSERT_L3(false);
    }
}

typedef struct
{
    cy_en_smif_txfr_width_t width;
    cy_en_smif_data_rate_t  dataRate;
    cy_en_smif_field_presence_t presence;
} cy_stc_cmd_sq_t;

typedef struct
{
    cy_en_smif_txfr_width_t width;
    cy_en_smif_data_rate_t  dataRate;
} cy_stc_addr_sq_t;

typedef struct
{
    cy_en_smif_txfr_width_t width;
    cy_en_smif_data_rate_t  dataRate;
    cy_en_smif_field_presence_t presence;
} cy_stc_mode_sq_t;

typedef struct
{
    cy_en_smif_txfr_width_t width;
    cy_en_smif_data_rate_t  dataRate;
} cy_stc_data_sq_t;

typedef struct
{
    cy_en_smif_rd_cmd_t cmd;
    uint8_t                 cmdTypeNum;
    uint8_t                 addrTypeNum;
    uint8_t                 modeTypeNum;
    uint8_t                 dataTypeNum;
} cy_stc_read_cmd_info_t;

typedef struct
{
    cy_en_smif_wt_cmd_t cmd;
    uint8_t                 cmdTypeNum;
    uint8_t                 addrTypeNum;
    uint8_t                 modeTypeNum;
    uint8_t                 dataTypeNum;
} cy_stc_write_cmd_info_t;

cy_en_smif_status_t Cy_SMIF_InitDeviceForS25FL(volatile stc_SMIF_DEVICE_t *device, cy_en_smif_device_type_t devType, const cy_stc_device_s25fl_config_t *config);
static uint8_t CY_SMIF_DummyCycleEachReadCmd(cy_en_smif_rd_cmd_t cmd, cy_en_smif_spi_latency_code_t lc);
static cy_en_smif_xip_addr_byte_t CY_SMIF_GetAddrByteSizeFromMemSize(uint32_t size);
static uint8_t CY_SMIF_GetReadCmdInfoNum(cy_en_smif_rd_cmd_t cmd);
static uint8_t CY_SMIF_GetWriteCmdInfoNum(cy_en_smif_wt_cmd_t cmd);
static void CY_SMIF_SetXIPReadSequence(volatile stc_SMIF_DEVICE_t *device, cy_en_smif_rd_cmd_t cmd, cy_en_smif_rw_mode_t mode, uint8_t dummySize, bool dualQuad);
static void CY_SMIF_SetXIPWriteSequence(volatile stc_SMIF_DEVICE_t *device, cy_en_smif_wt_cmd_t cmd, bool dualQuad);



cy_en_smif_status_t Cy_SMIF_S25FL_ReadId(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t* readManufactureID,
                                        uint8_t* readDeviceID,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ReadJedecId(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_JedecId_t* readData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ReadElectronicSignature(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t* readData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ReadStatusRegister1(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t* readData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ReadStatusRegister2(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t* readData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ReadConfigurationRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t* readData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ReadBankRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t* readData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_WriteBankRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t writeData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_AccessBankRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t BarBit0,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_WriteStatusRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t writeData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_WriteStatusAndConfigRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t writeStatusData,
                                        uint8_t writeConfigData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_WriteEnable(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_WriteDisable(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ClearStatusRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ReadAutoBootRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t* readData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_WriteAutoBootRegister(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t writeData,
                                        cy_stc_smif_context_t const *context);


cy_en_smif_status_t Cy_SMIF_S25FL_ProgramNVDLR(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t writeData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_WriteVDLR(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t writeData,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_Read(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_en_smif_rd_cmd_t readCmd,
                                        uint32_t readAddress,
                                        uint32_t size,
                                        uint8_t buf[],
                                        cy_en_smif_spi_latency_code_t lcCode,
                                        cy_en_smif_rw_mode_t mode,
                                        cy_en_smif_block_mode_t blockingMode,
                                        cy_stc_smif_context_t *context);

cy_en_smif_status_t Cy_SMIF_S25FL_Program(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_en_smif_wt_cmd_t writeCmd,
                                        uint32_t programedAddress,
                                        uint32_t size,
                                        const uint8_t buf[],
                                        cy_en_smif_block_mode_t blockingMode,
                                        cy_stc_smif_context_t *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ProgramSuspend(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ProgramResume(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_4KB_SectorErase(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t erasedAddress,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_SectorErase(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t erasedAddress,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_BulkErase(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);


cy_en_smif_status_t Cy_SMIF_S25FL_EraseSuspend(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_EraseResume(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_OPT_Program(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t programedAddress,
                                        uint32_t size,
                                        uint8_t buf[],
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_OPT_Read(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t readAddress,
                                        uint32_t size,
                                        uint8_t buf[],
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ASP_Read(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint16_t* readValue,
                                        cy_stc_smif_context_t const *context);


cy_en_smif_status_t Cy_SMIF_S25FL_ASP_Program(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint16_t* programedValue,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_DYB_Read(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t readAddress,
                                        uint8_t* readValue,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_DYB_Write(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t writeAddress,
                                        uint8_t  writeValue,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_PPB_Read(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t readAddress,
                                        uint8_t* readValue,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_PPB_Program(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint32_t programedAddress,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_PPB_Erase(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_PPB_LockBitRead(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint8_t* readValue,
                                        cy_stc_smif_context_t const *context);


cy_en_smif_status_t Cy_SMIF_S25FL_PPB_LockBitWrite(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_PasswordRead(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint64_t* readPW,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_PasswordProgram(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        uint64_t  programPW,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_SoftwareReset(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

cy_en_smif_status_t Cy_SMIF_S25FL_ModeBitReset(volatile stc_SMIF_t *base,
                                        cy_en_smif_slave_select_t slave,
                                        cy_stc_smif_context_t const *context);

#if defined(__cplusplus)
}
#endif

#endif /* (CY_SMIF_S25FL_H) */

