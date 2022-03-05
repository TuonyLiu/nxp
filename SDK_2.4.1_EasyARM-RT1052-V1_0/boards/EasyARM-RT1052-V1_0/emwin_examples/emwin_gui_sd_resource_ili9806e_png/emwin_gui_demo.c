/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_sd.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "board.h"
#include "emwin_support.h"

#include "GUI.h"
#include "BUTTON.h"
#include "CHECKBOX.h"
#include "SLIDER.h"
#include "DROPDOWN.h"
#include "RADIO.h"
#include "MULTIPAGE.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "clock_config.h"
#include "fsl_ili9806e.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifndef GUI_NORMAL_FONT
#define GUI_NORMAL_FONT (&GUI_Font16_ASCII)
#endif

#ifndef GUI_LARGE_FONT
#define GUI_LARGE_FONT (&GUI_Font16B_ASCII)
#endif

#ifndef GUI_SCALE_FACTOR
#define GUI_SCALE_FACTOR 1
#endif

#ifndef GUI_SCALE_FACTOR_X
#define GUI_SCALE_FACTOR_X GUI_SCALE_FACTOR
#endif

#ifndef GUI_SCALE_FACTOR_Y
#define GUI_SCALE_FACTOR_Y GUI_SCALE_FACTOR
#endif

#define GUI_SCALE(a) ((int)((a) * (GUI_SCALE_FACTOR)))
#define GUI_SCALE_X(x) ((int)((x) * (GUI_SCALE_FACTOR_X)))
#define GUI_SCALE_Y(y) ((int)((y) * (GUI_SCALE_FACTOR_Y)))
#define GUI_SCALE_COORDS(x, y) GUI_SCALE_X(x), GUI_SCALE_Y(y)
#define GUI_SCALE_RECT(x0, y0, xs, ys) GUI_SCALE_X(x0), GUI_SCALE_Y(y0), GUI_SCALE_X(xs), GUI_SCALE_Y(ys)

#define GUI_ID_DRAWAREA (GUI_ID_USER + 0)
#define GUI_ID_PAGEWIN1 (GUI_ID_USER + 1)
#define GUI_ID_PAGEWIN2 (GUI_ID_USER + 2)
/*******************************************************************************
 * SDCard Define
 ******************************************************************************/
#define DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_EnableLcdInterrupt(void);

void BOARD_PowerOffSDCARD(void);
void BOARD_PowerOnSDCARD(void);
/*!
* @brief wait card insert function.
*/
static status_t sdcardWaitCardInsert(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static FATFS g_fileSystem; /* File system object */
static FIL g_fileObject;   /* File object */

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

/* Initialize the LCD_DISP. */
void BOARD_InitLcd(void)
{
    volatile uint32_t i = 0x100U;

    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Reset the LCD. */
    GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &config);

    GPIO_WritePinOutput(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);

    while (i--)
    {
    }

    GPIO_WritePinOutput(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);

    /* Backlight. */
    config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);

    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);
}

void BOARD_InitLcdifPixelClock(void)
{
    /*
     * The desired output frame rate is 60Hz. So the pixel clock frequency is:
     * (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2) * 60 = 9.2M.
     * Here set the LCDIF pixel clock to 9.3M.
     */

    /*
     * Initialize the Video PLL.
     * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 93MHz.
     */
    clock_video_pll_config_t config = {
        .loopDivider = 31, .postDivider = 8, .numerator = 0, .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 3);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 0);
}


extern volatile uint32_t g_eventTimeMilliseconds;

void ms_delay (uint32_t ms)
{
    uint32_t dest_systick_count = g_eventTimeMilliseconds + ms;
    
    while ( g_eventTimeMilliseconds < dest_systick_count);
}

static void cbHKWin(WM_MESSAGE *pMsg)
{

    switch (pMsg->MsgId)
    {
    case WM_PAINT:

        break;

    default:
            WM_DefaultProc(pMsg);
    }
}



/*********************************************************************
*
*       _GetData
*
* Function description
*   This routine is called by GUI_JPEG_DrawEx(). The routine is responsible
*   for setting the data pointer to a valid data location with at least
*   one valid byte.
*
* Parameters:
*   p           - Pointer to application defined data.
*   NumBytesReq - Number of bytes requested.
*   ppData      - Pointer to data pointer. This pointer should be set to
*                 a valid location.
*   StartOfFile - If this flag is 1, the data pointer should be set to the
*                 beginning of the data stream.
*
* Return value:
*   Number of data bytes available.
*/
#define EM_WIN_GET_DATA_BUF_SIZE      1024*1024

SDK_ALIGN(uint8_t __emWin_get_data_buf[SDK_SIZEALIGN(EM_WIN_GET_DATA_BUF_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)],
          MAX(SDMMC_DATA_BUFFER_ALIGN_CACHE, SDMMCHOST_DMA_BUFFER_ADDR_ALIGN));
    
static int _GetData(void * p, const U8 ** ppData, unsigned NumBytesReq, U32 Off) 
{
    FRESULT error;
    UINT bytesRead;
    //
    // Check buffer size
    //
    
    if (NumBytesReq > EM_WIN_GET_DATA_BUF_SIZE) {
        NumBytesReq = EM_WIN_GET_DATA_BUF_SIZE;
    }
    
    //
    // Set file pointer to the required position
    //
    /* Move the file pointer */
    if (f_lseek(&g_fileObject, Off))
    {
        PRINTF("Set file pointer position failed. \r\n");
        while(1);
        //return 0;
    }
    //
    // Read data into buffer
    //
    error = f_read(&g_fileObject, __emWin_get_data_buf, NumBytesReq, &bytesRead);
    if (error)
    {
        PRINTF("Read file failed. \r\n");
        while(1);
        //return 0;
    }
    //
    // Set data pointer to the beginning of the buffer
    //
    *ppData = __emWin_get_data_buf;
    //
    // Return number of available bytes
    //

    return bytesRead;
}

extern GUI_CONST_STORAGE GUI_BITMAP bmup;

/*!
 * @brief Main function
 */
int main(void)
{
    FRESULT error;
    FSIZE_t image_size = 0;
    DIR directory; /* Directory object */
    FILINFO fileInformation;
    UINT bytesWritten;
    UINT bytesRead;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
    volatile bool failedFlag = false;
    char ch = '0';
    BYTE work[FF_MAX_SS];

//    SCB->VTOR = 0x00000000;  //Enable ITCM Vectors Table
    
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_InitI2C1Pins();
    BOARD_InitSemcPins();
    BOARD_SDCardInitPins();
    
    BOARD_BootClockRUN();
    BOARD_InitLcdifPixelClock();
    BOARD_USDHCClockConfiguration();
    BOARD_InitDebugConsole();
    BOARD_InitLcd();

    PRINTF("GUI demo start.\r\n");
    
    SysTick_Config(SystemCoreClock/1000);
    __DSB();
    __ISB();
    Ili9806e_Init();

    //emWin Init
    GUI_Init();
    GUI_SetColor(GUI_WHITE);
    GUI_FillRect(0,0,480,800);
    //
    // Displays the current version at the current cursor position
    //    
    GUI_DispString(GUI_GetVersionString());

    PRINTF("\r\nPlease insert a card into board for resource load.\r\n");
    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        return -1;
    }

    if (f_mount(&g_fileSystem, driverNumberBuffer, 0U))
    {
        PRINTF("Mount volume failed.\r\n");
        return -1;
    }
#if (FF_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        return -1;
    }
#endif
    
    error = f_open(&g_fileObject, _T("/image.bmp"), (FA_READ) );
    if (error)
    {
        if (error == FR_EXIST)
        {
            PRINTF("File exists.\r\n");
        }
        else
        {
            PRINTF("Open file failed.\r\n");
            return -1;
        }
    }
    image_size = f_size(&g_fileObject);
    
#if  1  //do you load bmp file in sdram frist
    if (f_lseek(&g_fileObject, 0))
    {
        PRINTF("Set file pointer position failed. \r\n");
        while(1);
        //return 0;
    }
    //
    // Read data into buffer
    //
    error = f_read(&g_fileObject, __emWin_get_data_buf, image_size, &bytesRead);
    if (error || (bytesRead != image_size))
    {
        PRINTF("Read file failed. \r\n");
        while(1);
        //return 0;
    }
    GUI_BMP_Draw(__emWin_get_data_buf,0,0);
    
    error = f_open(&g_fileObject, _T("/up.png"), (FA_READ) );
    f_lseek(&g_fileObject, 0);
    image_size = f_size(&g_fileObject);
    error = f_read(&g_fileObject, __emWin_get_data_buf, image_size, &bytesRead);
    volatile int32_t png_xsize = 0;
    volatile int32_t png_ysize = 0;
    
    png_xsize = GUI_PNG_GetXSize((void *)__emWin_get_data_buf,image_size);
    png_ysize = GUI_PNG_GetYSize((void *)__emWin_get_data_buf,image_size);
    GUI_PNG_Draw((void *)__emWin_get_data_buf,image_size,100,400);
    
#else
    GUI_BMP_DrawEx(_GetData,((void *) &g_fileObject),0,0);
#endif
    WM_Exec();
    while (1)
    {
        GUI_Delay(10);
//         /* Poll touch controller for update */
//         if (BOARD_Touch_Poll())
//         {
// #ifdef GUI_BUFFERS
//             GUI_MULTIBUF_Begin();
// #endif
//             GUI_Exec();
// #ifdef GUI_BUFFERS
//             GUI_MULTIBUF_End();
// #endif
//         }
    }
}


static status_t sdcardWaitCardInsert(void)
{
    /* Save host information. */
    g_sd.host.base = SD_HOST_BASEADDR;
    g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
    /* card detect type */
    g_sd.usrParam.cd = &s_sdCardDetect;
#if defined DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
    g_sd.usrParam.pwr = &s_sdCardPwrCtrl;
#endif
    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        PRINTF("\r\nSD host init fail\r\n");
        return kStatus_Fail;
    }
    /* power off card */
    SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);
    /* wait card insert */
    if (SD_WaitCardDetectStatus(SD_HOST_BASEADDR, &s_sdCardDetect, true) == kStatus_Success)
    {
        PRINTF("\r\nCard inserted.\r\n");
        /* power on the card */
        SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);
    }
    else
    {
        PRINTF("\r\nCard detect fail.\r\n");
        return kStatus_Fail;
    }

    return kStatus_Success;
}
