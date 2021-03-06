/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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

#include "bldc_control.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
* Code
******************************************************************************/

/*!
 * @brief BLDC motor commutation control.
 *
 * This function performs commutation, changes commutation sector
 *
 * @param   psCtrlBLDC  The pointer of main BLDC control structure
 *
 * @return None
 */
void MCS_BLDCCommutation(mcs_bldc_ctrl_t *psCtrlBLDC)
{
    /* select next commutation sector based on direction of spin */
    if (psCtrlBLDC->ui16MotorDir == 0)
    {
        /* forward - increase sector */
        if (++psCtrlBLDC->i16SectorCmt > 5)
            psCtrlBLDC->i16SectorCmt = 0;
    }
    else
    {
        /* backward - decrease sector */
        if (--psCtrlBLDC->i16SectorCmt < 0)
            psCtrlBLDC->i16SectorCmt = 5;
    }
}

/*!
 * @brief Rotor alignment to the initial known position.
 *
 * This function calculates duty cycle to obtain required alignment current
 * during alignment
 *
 * @param   psCtrlBLDC  The pointer of main BLDC control structure
 *
 * @return None
 */
void MCS_BLDCAlignment(mcs_bldc_ctrl_t *psCtrlBLDC)
{
    /* calculate align current error */
    psCtrlBLDC->f16IDcBusPiErr = MLIB_SubSat_F16(psCtrlBLDC->f16IDcBusAlign, psCtrlBLDC->f16IDcBusNoFilt);

    /* calculate align current PI controller */
    psCtrlBLDC->f16IDcBusPiOutput = GFLIB_CtrlPIpAW_F16(psCtrlBLDC->f16IDcBusPiErr, &psCtrlBLDC->bIDcBusPiStopIntFlag,
                                                        &psCtrlBLDC->sIDcBusPiParams);

    /* set required duty cycle */
    psCtrlBLDC->f16DutyCycle = psCtrlBLDC->f16IDcBusPiOutput;
}

/*!
 * @brief Rotor alignment to the initial known position.
 *
 * This function performs BLDC control loop - current and speed PI controllers
 *
 * @param   psCtrlBLDC  The pointer of main BLDC control structure
 *
 * @return None
 */
void MCS_BLDCControl(mcs_bldc_ctrl_t *psCtrlBLDC)
{
    frac16_t f16SpeedMeasuredAbs;

    f16SpeedMeasuredAbs =
        MLIB_Conv_F16l(MLIB_DivSat_F32(psCtrlBLDC->i16SpeedScaleConst, psCtrlBLDC->ui32PeriodSixCmtSum));

    /* required speed ramp calculation */
    psCtrlBLDC->f16SpeedRamp =
        MLIB_Conv_F16l(GFLIB_Ramp_F32(MLIB_Conv_F32s(psCtrlBLDC->f16SpeedCmd), &psCtrlBLDC->sSpeedRampParams));

    /* process spin direction and calculate speed error */
    if (psCtrlBLDC->ui16MotorDir == 0)
    {
        /* forward */
        psCtrlBLDC->f16SpeedMeasured = f16SpeedMeasuredAbs;
        psCtrlBLDC->f16SpeedPiErr = MLIB_SubSat_F16(psCtrlBLDC->f16SpeedRamp, psCtrlBLDC->f16SpeedMeasured);
    }
    else
    {
        /* backward */
        psCtrlBLDC->f16SpeedMeasured = -f16SpeedMeasuredAbs;
        psCtrlBLDC->f16SpeedPiErr = MLIB_SubSat_F16(psCtrlBLDC->f16SpeedMeasured, psCtrlBLDC->f16SpeedRamp);
    }

    /* calculate Speed PI controller */
    psCtrlBLDC->f16SpeedPiOutput =
        GFLIB_CtrlPIpAW_F16(psCtrlBLDC->f16SpeedPiErr, &psCtrlBLDC->bSpeedPiStopIntFlag, &psCtrlBLDC->sSpeedPiParams);

    /* calculate DC-bus current limit error */
    psCtrlBLDC->f16IDcBusPiErr = MLIB_SubSat_F16(psCtrlBLDC->f16IDcBusLim, psCtrlBLDC->f16IDcBus);

    /* calculate current (torque) PI controller */
    psCtrlBLDC->f16IDcBusPiOutput = GFLIB_CtrlPIpAW_F16(psCtrlBLDC->f16IDcBusPiErr, &psCtrlBLDC->bIDcBusPiStopIntFlag,
                                                        &psCtrlBLDC->sIDcBusPiParams);

    psCtrlBLDC->f16DutyCycle = psCtrlBLDC->f16SpeedPiOutput;

    /* decide whether current limiting */
    if (psCtrlBLDC->f16IDcBusPiOutput >= psCtrlBLDC->f16SpeedPiOutput)
    {
        /* no need for current limitation, duty cycle is set by speed controller */
        GFLIB_CtrlPIpAWInit_F16(psCtrlBLDC->f16DutyCycle, &psCtrlBLDC->sIDcBusPiParams);
        psCtrlBLDC->bSpeedPiStopIntFlag = FALSE;
    }
    else
    {
        /* current limitation is active, duty cycle is set by current controller */
        GFLIB_CtrlPIpAWInit_F16(psCtrlBLDC->f16DutyCycle, &psCtrlBLDC->sSpeedPiParams);
        psCtrlBLDC->f16DutyCycle = psCtrlBLDC->f16IDcBusPiOutput;
        psCtrlBLDC->bSpeedPiStopIntFlag = TRUE;
    }
}

