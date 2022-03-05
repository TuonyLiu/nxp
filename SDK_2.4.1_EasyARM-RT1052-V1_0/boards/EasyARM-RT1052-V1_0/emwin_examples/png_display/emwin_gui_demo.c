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
 * Prototypes
 ******************************************************************************/
void BOARD_EnableLcdInterrupt(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

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

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 4);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
}



static DROPDOWN_Handle hDropdown0;
static RADIO_Handle hRadio0;
static CHECKBOX_Handle hCheck0;
static WM_HWIN hDrawArea;

static SLIDER_Handle hSlider0;
static SLIDER_Handle hSlider1;
static SPINBOX_Handle hSpinbox0;
static PROGBAR_Handle hProgbar0;

static const GUI_COLOR color_list[] = {GUI_BLACK,   GUI_YELLOW, GUI_ORANGE, GUI_RED,
                                       GUI_MAGENTA, GUI_BLUE,   GUI_CYAN,   GUI_GREEN};
static const GUI_POINT triangle_points[] = {
    {GUI_SCALE(0), GUI_SCALE(0)}, {GUI_SCALE(-50), GUI_SCALE(100)}, {GUI_SCALE(50), GUI_SCALE(100)}};

static void cbDrawArea(WM_MESSAGE *pMsg)
{
    switch (pMsg->MsgId)
    {
        case WM_PAINT:
            GUI_SetColor(color_list[DROPDOWN_GetSel(hDropdown0)]);
            switch (RADIO_GetValue(hRadio0))
            {
                case 0:
                    if (CHECKBOX_GetState(hCheck0))
                    {
                        GUI_FillRect(GUI_SCALE_X(70) - GUI_SCALE(50), GUI_SCALE_Y(70) - GUI_SCALE(50),
                                     GUI_SCALE_X(70) + GUI_SCALE(50), GUI_SCALE_Y(70) + GUI_SCALE(50));
                    }
                    else
                    {
                        GUI_DrawRect(GUI_SCALE_X(70) - GUI_SCALE(50), GUI_SCALE_Y(70) - GUI_SCALE(50),
                                     GUI_SCALE_X(70) + GUI_SCALE(50), GUI_SCALE_Y(70) + GUI_SCALE(50));
                    }
                    break;
                case 1:
                    if (CHECKBOX_GetState(hCheck0))
                    {
                        GUI_FillPolygon(triangle_points, 3, GUI_SCALE_COORDS(70, 20));
                    }
                    else
                    {
                        GUI_DrawPolygon(triangle_points, 3, GUI_SCALE_COORDS(70, 20));
                    }
                    break;
                case 2:
                    if (CHECKBOX_GetState(hCheck0))
                    {
                        GUI_FillEllipse(GUI_SCALE_COORDS(70, 70), GUI_SCALE(50), GUI_SCALE(50));
                    }
                    else
                    {
                        GUI_DrawEllipse(GUI_SCALE_COORDS(70, 70), GUI_SCALE(50), GUI_SCALE(50));
                    }
                    break;
            }
            break;
        default:
            WM_DefaultProc(pMsg);
            break;
    }
}

static void cbPageWin1(WM_MESSAGE *pMsg)
{
    int NCode;
    int Id;

    switch (pMsg->MsgId)
    {
        case WM_NOTIFY_PARENT:
            Id = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;

            switch (Id)
            {
                case GUI_ID_RADIO0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            WM_InvalidateWindow(hDrawArea);
                            break;
                    }
                    break;

                case GUI_ID_DROPDOWN0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_SEL_CHANGED:
                            WM_InvalidateWindow(hDrawArea);
                            break;
                    }
                    break;

                case GUI_ID_CHECK0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            WM_InvalidateWindow(hDrawArea);
                            break;
                    }
                    break;
            }
            break;

        default:
            WM_DefaultProc(pMsg);
            break;
    }
}

static void cbPageWin2(WM_MESSAGE *pMsg)
{
    int NCode;
    int Id;

    switch (pMsg->MsgId)
    {
        case WM_NOTIFY_PARENT:
            Id = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;

            switch (Id)
            {
                case GUI_ID_SLIDER0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            PROGBAR_SetValue(hProgbar0, SLIDER_GetValue(hSlider0));
                            break;
                    }
                    break;

                case GUI_ID_SLIDER1:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            SPINBOX_SetValue(hSpinbox0, SLIDER_GetValue(hSlider1));
                            break;
                    }
                    break;

                case GUI_ID_SPINBOX0:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_VALUE_CHANGED:
                            SLIDER_SetValue(hSlider1, SPINBOX_GetValue(hSpinbox0));
                            break;
                    }
                    break;
            }
            break;

        default:
            WM_DefaultProc(pMsg);
            break;
    }
}

extern const unsigned char _acimage[131640UL + 1];
extern const unsigned char _acani00[30792UL + 1];
/*!
 * @brief Main function
 */
int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_InitI2C1Pins();
    BOARD_InitSemcPins();
    BOARD_BootClockRUN();
    BOARD_InitLcdifPixelClock();
    BOARD_InitDebugConsole();
    BOARD_InitLcd();

    PRINTF("GUI demo start.\r\n");
    
    GUI_Init();
    
    //
    // Displays the current version at the current cursor position
    //    
    GUI_DispString(GUI_GetVersionString());
    
    GUI_BMP_Draw((void *)_acimage,0,0);
    
    volatile int32_t png_xsize = 0;
    volatile int32_t png_ysize = 0;
    
    png_xsize = GUI_PNG_GetXSize((void *)_acani00,30793);
    png_ysize = GUI_PNG_GetYSize((void *)_acani00,30793);
    GUI_PNG_Draw((void *)_acani00,30793,0,0);
    
    WM_Exec();

    while (1)
    {
        /* Poll touch controller for update */
        if (BOARD_Touch_Poll())
        {
#ifdef GUI_BUFFERS
            GUI_MULTIBUF_Begin();
#endif
            GUI_Exec();
#ifdef GUI_BUFFERS
            GUI_MULTIBUF_End();
#endif
        }
    }
}
