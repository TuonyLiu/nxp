
#ifndef _DISK_FLASH_H
#define _DISK_FLASH_H


#include "fsl_flexspi.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_common.h"


#define EXAMPLE_FLEXSPI FLEXSPI
#define FLASH_SIZE 0x2000 /* 8M */
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI_AMBA_BASE
#define FLASH_PAGE_SIZE 256
#define EXAMPLE_SECTOR 0
#define SECTOR_SIZE 0x1000 /* 4K */
#define EXAMPLE_FLEXSPI_CLOCK kCLOCK_FlexSpi

#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL 0
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST 1
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD 2
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE 3
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR 4
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE 5
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD 6
#define NOR_CMD_LUT_SEQ_IDX_READID 7
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG 8
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG 9

#define CUSTOM_LUT_LENGTH 60
#define FLASH_BUSY_STATUS_POL 1
#define FLASH_BUSY_STATUS_OFFSET 0
#define FLASH_ERROR_STATUS_MASK 0x00


#define FLASH_START 0x60000000   // flash起始地址

#define FLASH_WRITE 0x60000000   // flash写入图片起始地址

#define FLASH_A_SIZE   1000     //  A区最大扇区
#define FLASH_B_SIZE   1000     //  B区最大扇区

#define FLASH_A_IMAGE_SIZE  10  //  A区最大图片数量
#define FLASH_B_IMAGE_SIZE  10  //  B区最大图片数量

#define FLASH_A_IMAGE_START  3  //  A区存储图片位置和大小的起始位置

#define FLASH_B_IMAGE_START  (FLASH_A_IMAGE_START + FLASH_A_IMAGE_SIZE*2)  //  B区存储图片位置和大小的起始位置


int disk_flash_init(void);

/*!
 * @brief  擦除一个扇区
 *
 * @param sector   扇区编号 从定义的起始地址开始算
 *
 * @retval 0x00    success.
 */
int disk_flash_erase(uint32_t sector);

/*!
 * @brief  写一个扇区
 *
 * @param sector   扇区编号 从定义的起始地址开始算
 *
 * @retval 0x00    success.
 */
int disk_flash_write(uint32_t sector,void *buff);

/*!
 * @brief  写一个扇区
 *
 * @param sector   扇区编号 从定义的起始地址开始算
 *
 * @retval 0x00    success.
 */
int disk_flash_read(uint32_t sector,void *buff);

#endif