/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

#include "fsl_tsc.h"
#include "fsl_adc.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_common.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC_BASE ADC2
#define DEMO_TSC_BASE TSC

#define EXAMPLE_TSC_IRQHANDLER TSC_DIG_IRQHandler
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void ADC_Configuration(void);
void NVIC_Configuration(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool g_tscTouch = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
* @brief TSC IRQ handler.
*/
void EXAMPLE_TSC_IRQHANDLER(void)
{
    if ((TSC_GetInterruptStatusFlags(DEMO_TSC_BASE) & kTSC_DetectSignalFlag) == kTSC_DetectSignalFlag)
    {
        TSC_ClearInterruptStatusFlags(DEMO_TSC_BASE, kTSC_DetectSignalFlag);
        TSC_StartMeasure(DEMO_TSC_BASE);
    }
    else
    {
        if ((TSC_GetInterruptStatusFlags(DEMO_TSC_BASE) & kTSC_ValidSignalFlag) == kTSC_ValidSignalFlag)
        {
            TSC_ClearInterruptStatusFlags(DEMO_TSC_BASE, kTSC_ValidSignalFlag);
            g_tscTouch = true;
        }
        TSC_ClearInterruptStatusFlags(DEMO_TSC_BASE, kTSC_MeasureSignalFlag);
    }
}

/*!
* @brief Set configuration of ADC working with TSC.
*/
void ADC_Configuration(void)
{
    adc_config_t adcConfigStrcut;
    adc_channel_config_t adcChannelConfigStruct;

    /* Initialize the ADC module. */
    ADC_GetDefaultConfig(&adcConfigStrcut);
    ADC_Init(DEMO_ADC_BASE, &adcConfigStrcut);
#if !(defined(FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE) && FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE)
    ADC_EnableHardwareTrigger(DEMO_ADC_BASE, true);
#endif

    /* Before TSC starts work, software driver configure ADC_HCx.
     * For four-wire resistive screen, x-coordinate measure triggers to ADC_HC3
     * and y-coordinate measure triggers to ADC_HC1. So we need configure ADC_HC1
     * and ADC_HC3 to make sure TSC could work normally.
     */
    adcChannelConfigStruct.channelNumber = 1U; /* Channel1 is ynlr port. */
    adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    ADC_SetChannelConfig(DEMO_ADC_BASE, 3U, &adcChannelConfigStruct);
    adcChannelConfigStruct.channelNumber = 3U; /* Channel3 is xnur port. */
    ADC_SetChannelConfig(DEMO_ADC_BASE, 1U, &adcChannelConfigStruct);

    /* Do auto hardware calibration. */
    if (kStatus_Success == ADC_DoAutoCalibration(DEMO_ADC_BASE))
    {
        PRINTF("ADC_DoAntoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC_DoAntoCalibration() Failed.\r\n");
    }
}

/*!
* @brief Set NVIC configuration.
*/
void NVIC_Configuration(void)
{
    NVIC_EnableIRQ(TSC_DIG_IRQn);
}

/*
 *
 * The example indicate how to make TSC, ADC  and touchscreen work together in interrupt mode.
 * Once touchscreen is touched, serial terminal will print the information about x/y-coordinate.
 */
int main(void)
{
    tsc_config_t k_tscConfig;

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    
    PRINTF("TSC fourWireModeInterrupt Example Start!\r\n");
    
    ADC_Configuration();
    NVIC_Configuration();

    TSC_GetDefaultConfig(&k_tscConfig);
    TSC_Init(DEMO_TSC_BASE, &k_tscConfig);
    /* Please enable interrupts and corresponding interrupt signals to insure proper interrupt operation.  */
    TSC_EnableInterruptSignals(DEMO_TSC_BASE,
                               kTSC_ValidSignalEnable | kTSC_MeasureSignalEnable | kTSC_DetectSignalEnable);
    TSC_EnableInterrupts(DEMO_TSC_BASE, kTSC_MeasureInterruptEnable | kTSC_DetectInterruptEnable);
    PRINTF("Please touch screen.\r\n");

    while (1)
    {
        TSC_SoftwareReset(DEMO_TSC_BASE);
        TSC_StartSenseDetection(DEMO_TSC_BASE);
        while ((TSC_GetStatusFlags(DEMO_TSC_BASE) & kTSC_StateMachineFlag) != kTSC_IdleState)
        {
        }
        if (g_tscTouch)
        {
            g_tscTouch = false;
            PRINTF("x = %d, y = %d\r\n", TSC_GetMeasureValue(DEMO_TSC_BASE, kTSC_XCoordinateValueSelection),
                   TSC_GetMeasureValue(DEMO_TSC_BASE, kTSC_YCoordinateValueSelection));
        }
    }
}
