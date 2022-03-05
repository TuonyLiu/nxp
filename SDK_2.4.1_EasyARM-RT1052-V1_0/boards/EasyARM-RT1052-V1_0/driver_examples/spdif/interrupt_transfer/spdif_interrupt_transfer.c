/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
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

#include "board.h"
#include "fsl_spdif.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SPDIF SPDIF
#define BUFFER_SIZE 768
#define BUFFER_NUM 4
#define PLAY_COUNT 5000
#define SAMPLE_RATE 48000
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
spdif_handle_t txHandle = {0};
spdif_handle_t rxHandle = {0};
static volatile bool isTxFinished = false;
static volatile bool isRxFinished = false;
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t audioBuff[BUFFER_SIZE * BUFFER_NUM], 4);
static volatile uint32_t beginCount = 0;
static volatile uint32_t sendCount = 0;
static volatile uint32_t receiveCount = 0;
static volatile uint8_t fullBlock = 0;
static volatile uint8_t emptyBlock = 0;

static spdif_transfer_t txXfer = {0};
static spdif_transfer_t rxXfer = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/
static void txCallback(SPDIF_Type *base, spdif_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_SPDIF_TxIdle)
    {
        sendCount++;
        emptyBlock++;

        if (sendCount == beginCount)
        {
            isTxFinished = true;
            SPDIF_TransferAbortSend(base, handle);
        }
    }
}

static void rxCallback(SPDIF_Type *base, spdif_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_SPDIF_RxCnew)
    {
        /* Copy C channel contents to tx */
        base->STCSCH = base->SRCSH;
        base->STCSCL = base->SRCSL;
    }

    if (status == kStatus_SPDIF_RxIdle)
    {
        receiveCount++;
        fullBlock++;

        if (receiveCount == beginCount)
        {
            isRxFinished = true;
            SPDIF_TransferAbortReceive(base, handle);
        }
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    spdif_config_t config;
    uint32_t sourceClock = 0;
    uint8_t txIndex = 0, rxIndex = 0;

    clock_audio_pll_config_t audioPllConfig = {0};

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Init audio pll */
    audioPllConfig.loopDivider = 30;
    audioPllConfig.postDivider = 2;
    audioPllConfig.denominator = 100;
    audioPllConfig.numerator = 72;
    CLOCK_InitAudioPll(&audioPllConfig);

    /* Set spdif clock root source */
    CLOCK_SetMux(kCLOCK_SpdifMux, 0);
    /* Set spdif pre-div */
    CLOCK_SetDiv(kCLOCK_Spdif0PreDiv, 0);
    CLOCK_SetDiv(kCLOCK_Spdif0Div, 0);

    PRINTF("SPDIF example started!\n\r");

    SPDIF_GetDefaultConfig(&config);
    SPDIF_Init(EXAMPLE_SPDIF, &config);

    sourceClock = CLOCK_GetPllFreq(kCLOCK_PllAudio);
    SPDIF_TxSetSampleRate(EXAMPLE_SPDIF, SAMPLE_RATE, sourceClock);

    SPDIF_TransferTxCreateHandle(EXAMPLE_SPDIF, &txHandle, txCallback, NULL);
    SPDIF_TransferRxCreateHandle(EXAMPLE_SPDIF, &rxHandle, rxCallback, NULL);

    txXfer.dataSize = BUFFER_SIZE;
    rxXfer.dataSize = BUFFER_SIZE;

    emptyBlock = BUFFER_NUM;
    fullBlock = 0;
    beginCount = PLAY_COUNT;

    /* Wait until finished */
    while ((isTxFinished != true) || (isRxFinished != true))
    {
        if ((isTxFinished != true) && (fullBlock > 0))
        {
            txXfer.data = audioBuff + txIndex * BUFFER_SIZE;
            SPDIF_TransferSendNonBlocking(EXAMPLE_SPDIF, &txHandle, &txXfer);
            fullBlock--;
            txIndex = (txIndex + 1) % BUFFER_NUM;
        }

        if ((isRxFinished != true) && (emptyBlock > 0))
        {
            rxXfer.data = audioBuff + rxIndex * BUFFER_SIZE;
            SPDIF_TransferReceiveNonBlocking(EXAMPLE_SPDIF, &rxHandle, &rxXfer);
            emptyBlock--;
            rxIndex = (rxIndex + 1) % BUFFER_NUM;
        }
    }

    PRINTF("\n\r SPDIF example finished!\n\r ");
    while (1)
    {
    }
}
