/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_mmc.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST

/*! @brief Data block count accessed in card */
#define DATA_BLOCK_COUNT (5U)
/*! @brief Start data block number accessed in card */
#define DATA_BLOCK_START (2U)
/*! @brief The first group to erase */
#define ERASE_GROUP_START (0U)
/*! @brief The last group to erase */
#define ERASE_GROUP_END (0U)
/*! @brief Data buffer size. */
#define DATA_BUFFER_SIZE (FSL_SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_PowerOffSDCARD(void);
void BOARD_PowerOnSDCARD(void);
/*!
* @brief printf the card information log.
*
* @param card Card descriptor.
*/
static void CardInformationLog(mmc_card_t *card);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Card descriptor. */
mmc_card_t g_mmc;

/* @brief decription about the read/write buffer
* The size of the read/write buffer should be a multiple of 512, since SDHC/SDXC card uses 512-byte fixed
* block length and this driver example is enabled with a SDHC/SDXC card.If you are using a SDSC card, you
* can define the block length by yourself if the card supports partial access.
* The address of the read/write buffer should align to the specific DMA data buffer address align value if
* DMA transfer is used, otherwise the buffer address is not important.
* At the same time buffer address/size should be aligned to the cache line size if cache is supported.
*/
/*! @brief Data written to the card */
SDK_ALIGN(uint8_t g_dataWrite[SDK_SIZEALIGN(DATA_BUFFER_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)],
          MAX(SDMMC_DATA_BUFFER_ALIGN_CACHE, SDMMCHOST_DMA_BUFFER_ADDR_ALIGN));
/*! @brief Data read from the card */
SDK_ALIGN(uint8_t g_dataRead[SDK_SIZEALIGN(DATA_BUFFER_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)],
          MAX(SDMMC_DATA_BUFFER_ALIGN_CACHE, SDMMCHOST_DMA_BUFFER_ADDR_ALIGN));

/*! @brief SDMMC host detect card configuration */
static const sdmmchost_detect_card_t s_sdCardDetect = {
#ifndef BOARD_SD_DETECT_TYPE
  .cdType = kSDMMCHOST_DetectCardByGpioCD,
#else
  .cdType = BOARD_SD_DETECT_TYPE,
#endif
  .cdTimeOut_ms = (~0U),
};

/*! @brief SDMMC card power control configuration */
#if defined DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
static const sdmmchost_pwr_card_t s_sdCardPwrCtrl = {
  .powerOn = BOARD_PowerOnSDCARD, .powerOnDelay_ms = 500U, .powerOff = BOARD_PowerOffSDCARD, .powerOffDelay_ms = 0U,
};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void BOARD_PowerOffSDCARD(void)
{
  /*
      Do nothing here.

      SD card will not be detected correctly if the card VDD is power off,
     the reason is caused by card VDD supply to the card detect circuit, this issue is exist on EVK board rev A1 and A2.

      If power off function is not implemented after soft reset and prior to SD Host initialization without remove/insert card,
     a UHS-I card may not reach its highest speed mode during the second card initialization.
     Application can avoid this issue by toggling the SD_VDD (GPIO) before the SD host initialization.
  */
}

void BOARD_PowerOnSDCARD(void)
{
  BOARD_USDHC_SDCARD_POWER_CONTROL(true);
}

static void BOARD_USDHCClockConfiguration(void)
{
  /*configure system pll PFD2 fractional divider to 18*/
  CLOCK_InitSysPfd(kCLOCK_Pfd0, 0x12U);
  /* Configure USDHC clock source and divider */
  CLOCK_SetDiv(kCLOCK_Usdhc1Div, 0U);
  CLOCK_SetMux(kCLOCK_Usdhc1Mux, 1U);
}


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
    GPIO_PinInit(GPIO2, 31, &led_config);

    mmc_card_t *card = &g_mmc;
    char ch = '0';
    bool isReadOnly;
    bool failedFlag = false;
    int i;

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_USDHCClockConfiguration();
    BOARD_InitDebugConsole();

    card->host.base = BOARD_MMC_HOST_BASEADDR;
    card->host.sourceClock_Hz = BOARD_MMC_HOST_CLK_FREQ;
    card->hostVoltageWindowVCC = BOARD_MMC_VCC_SUPPLY;
    card->busTiming = kMMC_HighSpeedTiming;
    card->busWidth  = kMMC_DataBusWidth4bit;
    
    BOARD_PowerOnSDCARD();
    
    PRINTF("\r\nMMCCARD polling example.\r\n");

    /* MMC card VCC supply, only allow 3.3 or 1.8v, depend on your board config.
    * If a power reset circuit is avaliable on you board for mmc, and 1.8v is supported,
    * #define BOARD_USDHC_MMCCARD_POWER_CONTROL_INIT()
    * #define BOARD_USDHC_MMCCARD_POWER_CONTROL(state)
    * in board.h must be implemented.
    * User can remove preset the voltage window and sdmmc will switch VCC automatically. */
    card->hostVoltageWindowVCC = BOARD_MMC_VCC_SUPPLY;
    /* Init card. */
    if (MMC_Init(card))
    {
        PRINTF("\n MMC card init failed \n");
        return -1;
    }
    /* card information log */
    CardInformationLog(card);

    PRINTF("\r\nRead/Write the card continuously until encounter error.... \r\n");
    /* Check if card is readonly. */
    isReadOnly = MMC_CheckReadOnly(card);
    if (isReadOnly)
    {
        while (true)
        {
            if (failedFlag || (ch == 'q'))
            {
                break;
            }

            PRINTF("\r\nRead one data block......\r\n");
            if (kStatus_Success != MMC_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, 1U))
            {
                PRINTF("Read one data block failed.\r\n");
                failedFlag = true;
                continue;
            }

            PRINTF("Read multiple data blocks......\r\n");
            if (kStatus_Success != MMC_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, DATA_BLOCK_COUNT))
            {
                PRINTF("Read multiple data blocks failed.\r\n");
                failedFlag = true;
                continue;
            }

            PRINTF(
                "\r\nInput 'q' to quit read process.\
                \r\nInput other char to read data blocks again.\r\n");
            ch = GETCHAR();
            PUTCHAR(ch);
        }
    }
    else
    {
        memset(g_dataWrite, 1U, sizeof(g_dataWrite));

        while (true)
        {
            if (failedFlag || (ch == 'q'))
            {
                break;
            }

            PRINTF("\r\nWrite/read one data block......\r\n");
            if (kStatus_Success != MMC_WriteBlocks(card, g_dataWrite, DATA_BLOCK_START, 1U))
            {
                PRINTF("Write one data block failed.\r\n");
                failedFlag = true;
                continue;
            }

            memset(g_dataRead, 0U, sizeof(g_dataRead));
            if (kStatus_Success != MMC_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, 1U))
            {
                PRINTF("Read one data block failed.\r\n");
                failedFlag = true;
                continue;
            }

            PRINTF("Compare the read/write content......\r\n");
            if (memcmp(g_dataRead, g_dataWrite, FSL_SDMMC_DEFAULT_BLOCK_SIZE))
            {
                PRINTF("The read/write content isn't consistent.\r\n");
                failedFlag = true;
                continue;
            }
            PRINTF("The read/write content is consistent.\r\n");

            PRINTF("Write/read multiple data blocks......\r\n");
            if (kStatus_Success != MMC_WriteBlocks(card, g_dataWrite, DATA_BLOCK_START, DATA_BLOCK_COUNT))
            {
                PRINTF("Write multiple data blocks failed.\r\n");
                failedFlag = true;
                continue;
            }

            memset(g_dataRead, 0U, sizeof(g_dataRead));
            if (kStatus_Success != MMC_ReadBlocks(card, g_dataRead, DATA_BLOCK_START, DATA_BLOCK_COUNT))
            {
                PRINTF("Read multiple data blocks failed.\r\n");
                failedFlag = true;
                continue;
            }

            PRINTF("Compare the read/write content......\r\n");
            if (memcmp(g_dataRead, g_dataWrite, FSL_SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT))
            {
                PRINTF("The read/write content isn't consistent.\r\n");
                failedFlag = true;
                continue;
            }
            PRINTF("The read/write content is consistent.\r\n");

            PRINTF("Erase data groups......\r\n");
            if (kStatus_Success != MMC_EraseGroups(card, ERASE_GROUP_START, ERASE_GROUP_END))
            {
                PRINTF("\n Erases blocks failed \n");
                failedFlag = true;
                continue;
            }

            PRINTF(
                "\r\nInput 'q' to quit read/write/erase process.\
                \r\nInput other char to read/write/erase data blocks again.\r\n");
            ch = GETCHAR();
            PUTCHAR(ch);
        }
    }
    PRINTF("\r\nThe example will not read/write data blocks again.\r\n");

    MMC_Deinit(card);

    while (true)
    {
    }
}


static void CardInformationLog(mmc_card_t *card)
{
    assert(card);

    PRINTF("\r\nCard user partition size %d * %d bytes\r\n", card->blockSize, card->userPartitionBlocks);
    PRINTF("\r\nWorking condition:\r\n");

    if (card->hostVoltageWindowVCC == kMMC_VoltageWindows270to360)
    {
        PRINTF("\r\n  Voltage: VCC - 2.7V ~ 3.3V");
    }
    else if (card->hostVoltageWindowVCC == kMMC_VoltageWindow170to195)
    {
        PRINTF("\r\n  Voltage: VCC - 1.7V ~ 1.95V");
    }
    if (card->hostVoltageWindowVCCQ == kMMC_VoltageWindows270to360)
    {
        PRINTF("  VCCQ - 2.7V ~ 3.3V\r\n");
    }
    else if (card->hostVoltageWindowVCCQ == kMMC_VoltageWindow170to195)
    {
        PRINTF("  VCCQ - 1.7V ~ 1.95V\r\n");
    }
    else if (card->hostVoltageWindowVCCQ == kMMC_VoltageWindow120)
    {
        PRINTF("  VCCQ - 1.2V\r\n");
    }

    if (card->busTiming == kMMC_HighSpeedTimingNone)
    {
        PRINTF("\r\n  Timing mode: Default");
    }
    else if (card->busTiming == kMMC_HighSpeedTiming)
    {
        PRINTF("\r\n  Timing mode: High Speed\r\n");
    }
    else if (card->busTiming == kMMC_HighSpeed200Timing)
    {
        PRINTF("\r\n  Timing mode: HS200\r\n");
    }
    else if (card->busTiming == kMMC_HighSpeed400Timing)
    {
        PRINTF("\r\n  Timing mode: HS400\r\n");
    }

    if (card->busWidth == kMMC_DataBusWidth4bitDDR)
    {
        PRINTF("\r\n  Bus width: 4-bit DDR\r\n");
    }
    else if (card->busWidth == kMMC_DataBusWidth8bitDDR)
    {
        PRINTF("\r\n  Bus width: 8-bit DDR\r\n");
    }
    else if (card->busWidth == kMMC_DataBusWidth8bit)
    {
        PRINTF("\r\n  Bus width: 8-bit\r\n");
    }
    else if (card->busWidth == kMMC_DataBusWidth4bit)
    {
        PRINTF("\r\n  Bus width: 4-bit\r\n");
    }
    else if (card->busWidth == kMMC_DataBusWidth1bit)
    {
        PRINTF("\r\n  Bus width: 1-bit\r\n");
    }

    if ((card->busTiming == kMMC_HighSpeedTiming) &&
        ((card->busWidth == kMMC_DataBusWidth4bitDDR) || (card->busWidth == kMMC_DataBusWidth8bitDDR)))
    {
        PRINTF("\r\n  Freq : %d HZ\r\n", card->busClock_Hz / 2U);
    }
    else
    {
        PRINTF("\r\n  Freq : %d HZ\r\n", card->busClock_Hz);
    }
}

