/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_elcdif.h"
#include "fsl_cache.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "jpeglib.h"
#include "board.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "clock_config.h"

#include "avi.h"
#include "mjpeg.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_ELCDIF LCDIF

#define APP_IMG_HEIGHT 272
#define APP_IMG_WIDTH 480
#define APP_HSW 41
#define APP_HFP 4
#define APP_HBP 8
#define APP_VSW 10
#define APP_VFP 4
#define APP_VBP 2
#define APP_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)

#define APP_LCDIF_DATA_BUS kELCDIF_DataBus16Bit

/* Display. */
#define LCD_DISP_GPIO GPIO1
#define LCD_DISP_GPIO_PIN 8
/* Back light. */
#define LCD_BL_GPIO GPIO1
#define LCD_BL_GPIO_PIN 10

/* Frame buffer data alignment, for better performance, the LCDIF frame buffer should be 64B align. */
#define FRAME_BUFFER_ALIGN 64
/* SD power control function */
#define DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
/*
 * For better performance, three frame buffers are used in this demo.
 */
#define APP_LCD_FB_NUM 2 /* LCD frame buffer number. */
#define APP_LCD_FB_BPP 2 /* LCD frame buffer byte per pixel, RGB888 format, 24-bit. */

/* Cache line size. */
#ifndef FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#define FSL_FEATURE_L2CACHE_LINESIZE_BYTE 0
#endif
#ifndef FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#define FSL_FEATURE_L1DCACHE_LINESIZE_BYTE 0
#endif

#if (FSL_FEATURE_L2CACHE_LINESIZE_BYTE > FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)
#define APP_CACHE_LINE_SIZE FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#else
#define APP_CACHE_LINE_SIZE FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_EnableLcdInterrupt(void);
void BOARD_PowerOffSDCARD(void);
void BOARD_PowerOnSDCARD(void);
static status_t sdcardWaitCardInsert(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static volatile bool g_lcdFramePending = false;
static void *volatile s_fbList = NULL; /* List to the frame buffers. */
static void *volatile inactiveBuf = NULL;
static void *volatile activeBuf = NULL;

/* This struct contains the JPEG decompression parameters */
static struct jpeg_decompress_struct cinfo;
/* This struct represents a JPEG error handler */
static struct jpeg_error_mgr jerr;

SDK_ALIGN(static uint8_t g_frameBuffer[APP_LCD_FB_NUM][SDK_SIZEALIGN(APP_IMG_HEIGHT * APP_IMG_WIDTH * APP_LCD_FB_BPP,
                                                                     APP_CACHE_LINE_SIZE)],
          APP_CACHE_LINE_SIZE);

uint8_t *gp_vam_addr0 = g_frameBuffer[0];
uint8_t *gp_vam_addr1 = g_frameBuffer[1];

AT_NONCACHEABLE_SECTION(static FATFS g_fileSystem); /* File system object */
AT_NONCACHEABLE_SECTION(static FIL jpgFil);

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
extern void APP_LCDIF_IRQHandler(void);

void LCDIF_IRQHandler(void)
{
    APP_LCDIF_IRQHandler();
}

/* Enable interrupt. */
void BOARD_EnableLcdInterrupt(void)
{
    EnableIRQ(LCDIF_IRQn);
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

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);

    while (i--)
    {
    }

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);

    /* Backlight. */
    config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);
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

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 4);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
}

void BOARD_PowerOffSDCARD(void)
{
    /* Doing nothing here, because the card detect circuit power is supply from the card VDD,
    if power off the card, card will not be detect correctly, this issue is exist on EVK board rev A1 and A2
    Note:
    If power off function is not implemented, a UHS-I card may not reach its highest speed mode during the second
    times card intialization without remove/insert card , but application can avoid this issue by initial the card only
    one time.
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



/* Get the empty frame buffer from the s_fbList. */
static void *APP_GetFrameBuffer(void)
{
    void *fb;

    fb = s_fbList;

    if (NULL != fb)
    {
        s_fbList = *(void **)fb;
    }

    return fb;
}

/* Put the unused frame buffer to the s_fbList. */
static void APP_PutFrameBuffer(void *fb)
{
    *(void **)fb = s_fbList;
    s_fbList = fb;
}
#if 0
/*!
 * @brief  Jpeg Decode
 * @param  file:    pointer to the jpg file
 * @param  buff:     pointer to the image line
 */
void jpeg_decode(FIL *file, uint8_t *buffer)
{
    uint8_t *jpg_buffer;
    uint8_t *jpg_buffer_aligned;
    uint8_t *read_pos;
    UINT jpg_size;
    UINT bytesRead;
    UINT bytesRemain;

    // Decode JPEG Image
    JSAMPROW row_pointer[1] = {0}; /* Output row buffer */
    uint32_t row_stride = 0;       /* physical row width in image buffer */

    // Step 1: allocate and initialize JPEG decompression object
    cinfo.err = jpeg_std_error(&jerr);

    // Step 2: Initialize the JPEG decompression object
    jpeg_create_decompress(&cinfo);

    jpg_size = f_size(file);

    jpg_buffer = (unsigned char *)malloc(jpg_size + 2 * APP_CACHE_LINE_SIZE);
    jpg_buffer_aligned = (void *)(SDK_SIZEALIGN((uint32_t)jpg_buffer, APP_CACHE_LINE_SIZE));

    bytesRemain = jpg_size;
    read_pos = jpg_buffer_aligned;

    DCACHE_CleanInvalidateByRange((uint32_t)jpg_buffer_aligned, jpg_size);

    while (bytesRemain > 0)
    {
        f_read(file, read_pos, bytesRemain, &bytesRead);
        bytesRemain -= bytesRead;
        read_pos += bytesRead;
    }

    jpeg_mem_src(&cinfo, jpg_buffer_aligned, jpg_size);

    // Step 3: read image parameters with jpeg_read_header()
    jpeg_read_header(&cinfo, true);

    // Step 4: set parameters for decompression
    cinfo.dct_method = JDCT_FLOAT;

    // Step 5: start decompressor
    jpeg_start_decompress(&cinfo);

    row_stride = APP_IMG_WIDTH * APP_LCD_FB_BPP;
    while (cinfo.output_scanline < cinfo.output_height)
    {
        row_pointer[0] = &buffer[cinfo.output_scanline * row_stride];
        jpeg_read_scanlines(&cinfo, row_pointer, 1);
    }

    // Step 6: Finish decompression
    jpeg_finish_decompress(&cinfo);

    // Step 7: Release JPEG decompression object
    jpeg_destroy_decompress(&cinfo);

    free(jpg_buffer);
}
#endif

static int MOUNT_SDCard(void)
{
    FRESULT error;
    const TCHAR driverName[3U] = {SDDISK + '0', ':', '/'};

    // clear FATFS manually
    memset((void *)&g_fileSystem, 0, sizeof(g_fileSystem));

    /* Wait for the card insert. */
    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        PRINTF("Card not inserted.\r\n");
        return -1;
    }

    // Mount the driver
    if (f_mount(&g_fileSystem, driverName, 0))
    {
        PRINTF("Mount volume failed.\r\n");
        return -2;
    }

#if (FF_FS_RPATH >= 2U)
    if (f_chdrive((char const *)&driverName[0U]))
    {
        PRINTF("Change drive failed.\r\n");
        return -3;
    }
#endif

    // Open file to check
    error = f_open(&jpgFil, _T("/0.avi"), FA_OPEN_EXISTING);
    if (error != FR_OK)
    {
        PRINTF("No demo avi file!\r\n");
        return -4;
    }

    f_close(&jpgFil);

    return 0;
}

void APP_LCDIF_IRQHandler(void)
{
    uint32_t intStatus;

    intStatus = ELCDIF_GetInterruptStatus(APP_ELCDIF);

    ELCDIF_ClearInterruptStatus(APP_ELCDIF, intStatus);

    if (intStatus & kELCDIF_CurFrameDone)
    {
        if (g_lcdFramePending)
        {
            /*
             * The inactive buffer turns to be active frame buffer, the
             * old active frame buffer is not used, so put it into the
             * frame buffer list.
             */
            APP_PutFrameBuffer(activeBuf);

            activeBuf = inactiveBuf;
            g_lcdFramePending = false;
        }
    }
}

void APP_ELCDIF_Init(void)
{
    uint8_t i;

    const elcdif_rgb_mode_config_t config = {
        .panelWidth = APP_IMG_WIDTH,
        .panelHeight = APP_IMG_HEIGHT,
        .hsw = APP_HSW,
        .hfp = APP_HFP,
        .hbp = APP_HBP,
        .vsw = APP_VSW,
        .vfp = APP_VFP,
        .vbp = APP_VBP,
        .polarityFlags = APP_POL_FLAGS,
        .bufferAddr = (uint32_t)g_frameBuffer[0],
        .pixelFormat = kELCDIF_PixelFormatRGB565,
        .dataBus = APP_LCDIF_DATA_BUS,
    };

    for (i = 1; i < APP_LCD_FB_NUM; i++)
    {
        APP_PutFrameBuffer(g_frameBuffer[i]);
    }

    activeBuf = g_frameBuffer[0];
    ELCDIF_RgbModeInit(APP_ELCDIF, &config);
}

extern int video_play_mjpeg(uint8_t *pname);
/*!
 * @brief Main function
 */
int main(void)
{
    int i = 0;
    FRESULT error;
    char jpgFileName[10];
    void *freeFb;

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitLcdifPixelClock();
    BOARD_USDHCClockConfiguration();
    BOARD_InitDebugConsole();
    BOARD_InitLcd();

    PRINTF("SD JPEG demo start:\r\n");

    APP_ELCDIF_Init();

    BOARD_EnableLcdInterrupt();

    // Init the SD card
    if (0 != MOUNT_SDCard())
    {
        PRINTF("SD card mount error. Demo stopped!");
        return -1;
    }

    // clear the framebuffer first
    memset(g_frameBuffer, 0, sizeof(g_frameBuffer));

    ELCDIF_EnableInterrupts(APP_ELCDIF, kELCDIF_CurFrameDoneInterruptEnable);
    ELCDIF_RgbModeStart(APP_ELCDIF);

    while (1)
    {
        video_play_mjpeg("/0.avi");
        #if 0
        // format the filename
        sprintf(jpgFileName, "/%d.jpg", i++);
        // open it
        error = f_open(&jpgFil, jpgFileName, FA_READ);
        if (error != FR_OK)
        {
            i = 0;
            continue;
        }

        /* Get free frame buffer and convert the jpeg output to it. */
        do
        {
            ELCDIF_DisableInterrupts(APP_ELCDIF, kELCDIF_CurFrameDoneInterruptEnable);
            freeFb = APP_GetFrameBuffer();
            ELCDIF_EnableInterrupts(APP_ELCDIF, kELCDIF_CurFrameDoneInterruptEnable);
        } while (NULL == freeFb);

        PRINTF("Decoding %s...", jpgFileName);
        jpeg_decode(&jpgFil, freeFb);
        PRINTF("done!\r\n", jpgFileName);
        f_close(&jpgFil);

        DCACHE_CleanInvalidateByRange((uint32_t)freeFb, APP_IMG_HEIGHT * APP_IMG_WIDTH * APP_LCD_FB_BPP);

        /* Make sure previous set frame buffer is actived and shown. */
        while (g_lcdFramePending)
        {
        }

        ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)freeFb);
        inactiveBuf = freeFb;
        g_lcdFramePending = true;
        #endif
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
