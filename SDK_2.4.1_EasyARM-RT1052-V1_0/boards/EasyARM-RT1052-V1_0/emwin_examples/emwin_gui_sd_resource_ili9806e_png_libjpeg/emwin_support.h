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

#ifndef _EMWIN_SUPPORT_H_
#define _EMWIN_SUPPORT_H_

/* Macros for the LCD controller. */
#define APP_ELCDIF LCDIF

#define APP_IMG_HEIGHT 800
#define APP_IMG_WIDTH 480
#define APP_HSW 20
#define APP_HFP 4
#define APP_HBP 1
#define APP_VSW 33
#define APP_VFP 2
#define APP_VBP 10
#define APP_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)

/* Display. */
#define LCD_DISP_GPIO GPIO1
#define LCD_DISP_GPIO_PIN 8
/* Back light. */
#define LCD_BL_GPIO GPIO2
#define LCD_BL_GPIO_PIN 30

/* Macros for the touch touch controller. */
#define BOARD_TOUCH_I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)

#define BOARD_TOUCH_I2C_CLOCK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))
#define BOARD_TOUCH_I2C_BAUDRATE 100000U

#define LPI2C_DATA_LENGTH 32U

#define LPI2C_MASTER_IRQ LPI2C1_IRQn
#define LPI2C_MASTER_IRQHandler LPI2C1_IRQHandler
#define BOARD_TOUCH_INT_PORT PORTA
#define BOARD_TOUCH_INT_PIN 2

#define BOARD_LCD_READABLE 1

#define LCD_WIDTH 480
#define LCD_HEIGHT 800
#define LCD_BITS_PER_PIXEL 16
#define LCD_BYTES_PER_PIXEL (LCD_BITS_PER_PIXEL / 8)

/* Color depth dependent definitions */
#if LCD_BITS_PER_PIXEL == 8
#define DISPLAY_DRIVER GUIDRV_LIN_8
#define COLOR_CONVERSION GUICC_0
#define ELCDIF_PIXEL_FORMAT kELCDIF_PixelFormatRAW8
#define APP_LCDIF_DATA_BUS kELCDIF_DataBus8Bit
#elif LCD_BITS_PER_PIXEL == 16
#define DISPLAY_DRIVER GUIDRV_LIN_16
#define COLOR_CONVERSION GUICC_M565
#define ELCDIF_PIXEL_FORMAT kELCDIF_PixelFormatRGB565
#define APP_LCDIF_DATA_BUS kELCDIF_DataBus16Bit
#else
#define DISPLAY_DRIVER GUIDRV_LIN_32
#define COLOR_CONVERSION GUICC_M8888I
#define ELCDIF_PIXEL_FORMAT kELCDIF_PixelFormatXRGB8888
#define APP_LCDIF_DATA_BUS kELCDIF_DataBus16Bit
#endif

/* Define scale factors */
#define GUI_SCALE_FACTOR 0.8
#define GUI_SCALE_FACTOR_X 1.5
#define GUI_SCALE_FACTOR_Y 1.1

/* Use larger fonts */
#define GUI_NORMAL_FONT (&GUI_Font24_ASCII)
#define GUI_LARGE_FONT (&GUI_Font32B_ASCII)

#define GUI_BUFFERS 2
#define GUI_NUMBYTES 1*1024*1024U /*! Amount of memory assigned to the emWin library */

#define FRAME_BUFFER_ALIGN 64

#define VRAM_SIZE (LCD_HEIGHT * LCD_WIDTH * LCD_BYTES_PER_PIXEL)

extern int BOARD_Touch_Poll(void);

#endif