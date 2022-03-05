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
#include "fsl_spdif_edma.h"
#include "fsl_debug_console.h"
#include "fsl_dmamux.h"
#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SPDIF SPDIF
#define EXAMPLE_DMA DMA0
#define EXAMPLE_DMAMUX DMAMUX
#define SPDIF_IRQ SPDIF_IRQn
#define SPDIF_ErrorHandle SPDIF_IRQHandler
#define SPDIF_RX_LEFT_CHANNEL (0)
#define SPDIF_RX_RIGHT_CHANNEL (1)
#define SPDIF_TX_LEFT_CHANNEL (2)
#define SPDIF_TX_RIGHT_CHANNEL (3)
#define SPDIF_RX_SOURCE (85)
#define SPDIF_TX_SOURCE (86)
#define BUFFER_SIZE 1024
#define BUFFER_NUM 4
#define PLAY_COUNT 5000
#define SAMPLE_RATE 48000
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
AT_NONCACHEABLE_SECTION_INIT(spdif_edma_handle_t txHandle) = {0};
AT_NONCACHEABLE_SECTION_INIT(spdif_edma_handle_t rxHandle) = {0};
edma_handle_t dmaTxLeftHandle = {0};
edma_handle_t dmaTxRightHandle = {0};
edma_handle_t dmaRxLeftHandle = {0};
edma_handle_t dmaRxRightHandle = {0};
static volatile bool isTxFinished = false;
static volatile bool isRxFinished = false;
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t audioLeftBuff[BUFFER_SIZE * BUFFER_NUM], 4);
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t audioRightBuff[BUFFER_SIZE * BUFFER_NUM], 4);
uint8_t udata[4] = {0};
uint8_t qdata[4] = {0};
static volatile uint32_t beginCount = 0;
static volatile uint32_t sendCount = 0;
static volatile uint32_t receiveCount = 0;
static volatile uint8_t fullBlock = 0;
static volatile uint8_t emptyBlock = 0;

static spdif_edma_transfer_t txXfer = {0};
static spdif_edma_transfer_t rxXfer = {0};
/*******************************************************************************
 * Code
 ******************************************************************************/
static void txCallback(SPDIF_Type *base, spdif_edma_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_SPDIF_TxIdle)
    {
        sendCount++;
        emptyBlock++;

        if (sendCount == beginCount)
        {
            isTxFinished = true;
            SPDIF_TransferAbortSendEDMA(base, handle);
        }
    }
}

static void rxCallback(SPDIF_Type *base, spdif_edma_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_SPDIF_RxIdle)
    {
        receiveCount++;
        fullBlock++;

        if (receiveCount == beginCount)
        {
            isRxFinished = true;
            SPDIF_TransferAbortReceiveEDMA(base, handle);
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
    edma_config_t dmaConfig = {0};

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

    PRINTF("SPDIF EDMA example started!\n\r");

    /* Create EDMA handle */
    /*
     * dmaConfig.enableRoundRobinArbitration = false;
     * dmaConfig.enableHaltOnError = true;
     * dmaConfig.enableContinuousLinkMode = false;
     * dmaConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(EXAMPLE_DMA, &dmaConfig);
    EDMA_CreateHandle(&dmaTxLeftHandle, EXAMPLE_DMA, SPDIF_TX_LEFT_CHANNEL);
    EDMA_CreateHandle(&dmaRxLeftHandle, EXAMPLE_DMA, SPDIF_RX_LEFT_CHANNEL);
    EDMA_CreateHandle(&dmaTxRightHandle, EXAMPLE_DMA, SPDIF_TX_RIGHT_CHANNEL);
    EDMA_CreateHandle(&dmaRxRightHandle, EXAMPLE_DMA, SPDIF_RX_RIGHT_CHANNEL);

    DMAMUX_Init(EXAMPLE_DMAMUX);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, SPDIF_TX_LEFT_CHANNEL, SPDIF_TX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, SPDIF_TX_LEFT_CHANNEL);
    DMAMUX_SetSource(EXAMPLE_DMAMUX, SPDIF_RX_LEFT_CHANNEL, SPDIF_RX_SOURCE);
    DMAMUX_EnableChannel(EXAMPLE_DMAMUX, SPDIF_RX_LEFT_CHANNEL);

    SPDIF_GetDefaultConfig(&config);
    SPDIF_Init(EXAMPLE_SPDIF, &config);

    sourceClock = CLOCK_GetPllFreq(kCLOCK_PllAudio);
    SPDIF_TxSetSampleRate(EXAMPLE_SPDIF, 48000, sourceClock);

    /* Enable SPDIF interrupt to handle error and other subcode */
    EnableIRQ(SPDIF_IRQ);
    SPDIF_EnableInterrupts(EXAMPLE_SPDIF, kSPDIF_UChannelReceiveRegisterFull | kSPDIF_QChannelReceiveRegisterFull |
                                              kSPDIF_RxControlChannelChange | kSPDIF_TxFIFOError);

    SPDIF_TransferTxCreateHandleEDMA(EXAMPLE_SPDIF, &txHandle, txCallback, NULL, &dmaTxLeftHandle, &dmaTxRightHandle);
    SPDIF_TransferRxCreateHandleEDMA(EXAMPLE_SPDIF, &rxHandle, rxCallback, NULL, &dmaRxLeftHandle, &dmaRxRightHandle);

    txXfer.dataSize = BUFFER_SIZE;
    rxXfer.dataSize = BUFFER_SIZE;

    emptyBlock = BUFFER_NUM;
    fullBlock = 0;
    beginCount = PLAY_COUNT;

    /* Wait for DPLL locked */
    while ((SPDIF_GetStatusFlag(EXAMPLE_SPDIF) & kSPDIF_RxDPLLLocked) == 0)
    {
    }

    /* Wait until finished */
    while ((isTxFinished != true) && (isRxFinished != true))
    {
        if ((isTxFinished != true) && (fullBlock > 0))
        {
            txXfer.leftData = audioLeftBuff + txIndex * BUFFER_SIZE;
            txXfer.rightData = audioRightBuff + txIndex * BUFFER_SIZE;
            SPDIF_TransferSendEDMA(EXAMPLE_SPDIF, &txHandle, &txXfer);
            fullBlock--;
            txIndex = (txIndex + 1) % BUFFER_NUM;
        }

        if ((isRxFinished != true) && (emptyBlock > 0))
        {
            rxXfer.leftData = audioLeftBuff + rxIndex * BUFFER_SIZE;
            rxXfer.rightData = audioRightBuff + rxIndex * BUFFER_SIZE;
            SPDIF_TransferReceiveEDMA(EXAMPLE_SPDIF, &rxHandle, &rxXfer);
            emptyBlock--;
            rxIndex = (rxIndex + 1) % BUFFER_NUM;
        }
    }
    PRINTF("\n\r SPDIF EDMA example finished!\n\r ");
    while (1)
    {
    }
}

void SPDIF_ErrorHandle(void)
{
    uint32_t data = 0;

    /* Handle cnew event */
    if (SPDIF_GetStatusFlag(EXAMPLE_SPDIF) & kSPDIF_RxControlChannelChange)
    {
        SPDIF_ClearStatusFlags(EXAMPLE_SPDIF, kSPDIF_RxControlChannelChange);
        EXAMPLE_SPDIF->STCSCL = EXAMPLE_SPDIF->SRCSL;
        EXAMPLE_SPDIF->STCSCH = EXAMPLE_SPDIF->SRCSH;
    }

    /* Handle Q channel full */
    if (SPDIF_GetStatusFlag(EXAMPLE_SPDIF) & kSPDIF_QChannelReceiveRegisterFull)
    {
        data = SPDIF_ReadQChannel(EXAMPLE_SPDIF);
        qdata[0] = (data & 0xFF);
        qdata[1] = (data >> 8U) & 0xFF;
        qdata[2] = (data >> 16U) & 0xFF;
    }

    if (SPDIF_GetStatusFlag(EXAMPLE_SPDIF) & kSPDIF_UChannelReceiveRegisterFull)
    {
        data = SPDIF_ReadUChannel(EXAMPLE_SPDIF);
        udata[0] = (data & 0xFF);
        udata[1] = (data >> 8U) & 0xFF;
        udata[2] = (data >> 16U) & 0xFF;
    }

    if (SPDIF_GetStatusFlag(EXAMPLE_SPDIF) & kSPDIF_TxFIFOError)
    {
        /* Reset FIFO */
        SPDIF_ClearStatusFlags(EXAMPLE_SPDIF, kSPDIF_TxFIFOError);
        data = EXAMPLE_SPDIF->SCR & (~SPDIF_SCR_TXFIFO_CTRL_MASK);
        data |= SPDIF_SCR_TXFIFO_CTRL(0x2);
        EXAMPLE_SPDIF->SCR = data;
        data = EXAMPLE_SPDIF->SCR & (~SPDIF_SCR_TXFIFO_CTRL_MASK);
        data |= SPDIF_SCR_TXFIFO_CTRL(0x1);
        EXAMPLE_SPDIF->SCR = data;
        /* Abort current EDMA transfer */
        SPDIF_TransferAbortSendEDMA(EXAMPLE_SPDIF, &txHandle);
        emptyBlock = BUFFER_NUM;
    }

    if (SPDIF_GetStatusFlag(EXAMPLE_SPDIF) & kSPDIF_RxFIFOError)
    {
        SPDIF_ClearStatusFlags(EXAMPLE_SPDIF, kSPDIF_RxFIFOError);
        EXAMPLE_SPDIF->SCR |= SPDIF_SCR_RXFIFO_RST_MASK;
        EXAMPLE_SPDIF->SCR &= ~SPDIF_SCR_RXFIFO_RST_MASK;
        SPDIF_TransferAbortReceiveEDMA(EXAMPLE_SPDIF, &rxHandle);
        emptyBlock = BUFFER_NUM;
    }
}
