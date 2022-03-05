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
#include "fsl_sgtl5000.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitations
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*
 * SGTL5000 register cache
 * We can't read the SGTL5000 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const uint16_t SGTL5000_reg[SGTL5000_CACHEREGNUM] = {
    0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000a, 0x01c0, 0x0000, 0x00ff, 0x00ff, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x007b, 0x0100, 0x0032, 0x0000, 0x00c3, 0x00c3, 0x01c0, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000, 0x0002, 0x0037, 0x004d, 0x0080, 0x0008, 0x0031, 0x0026, 0x00e9,
};

static uint16_t reg_cache[SGTL5000_CACHEREGNUM];
/*******************************************************************************
 * Code
 ******************************************************************************/
void ms_delay(uint32_t ms)
{
    volatile uint32_t i = 0;

    while (ms--) {
        for (i = 0; i < 8000000; ++i)
        {
            __asm("NOP"); /* delay */
        }
    }

}

status_t SGTL5000_Init(codec_handle_t *handle, void *SGTL5000_config)
{
    SGTL5000_config_t *config = (SGTL5000_config_t *)SGTL5000_config;
    uint16_t reg, ana_pwr, lreg_ctrl, ref_ctrl, lo_ctrl, short_ctrl, sss;

    memcpy(reg_cache, SGTL5000_reg, sizeof(SGTL5000_reg));

    /* Set SGTL5000 I2C address */
    handle->slaveAddress = SGTL5000_I2C_ADDR;

    /* Read & Check chip id,SGTL5000 id is 0xA0?? */
    uint16_t chip_id = 0;
    SGTL5000_ReadReg(handle, 0x0000, &chip_id);

    if ( (chip_id >> 8) & 0xFF != 0xA0 ) {
        return kStatus_Fail;
    }
    
#if 0
	/* reset value */
	ana_pwr    = SGTL5000_DAC_STEREO |
                 SGTL5000_LINREG_SIMPLE_POWERUP |
                 SGTL5000_STARTUP_POWERUP |
                 SGTL5000_ADC_STEREO | SGTL5000_REFTOP_POWERUP;
	lreg_ctrl  = 0;
	ref_ctrl   = 0;
	lo_ctrl    = 0;
	short_ctrl = 0;
	sss        = SGTL5000_DAC_SEL_I2S_IN << SGTL5000_DAC_SEL_SHIFT;

    /* turn of startup power */
    ana_pwr &= ~SGTL5000_STARTUP_POWERUP;
    ana_pwr &= ~SGTL5000_LINREG_SIMPLE_POWERUP;

    /* VDDC use VDDIO rail */
    lreg_ctrl |= SGTL5000_VDDC_ASSN_OVRD;
    lreg_ctrl |= SGTL5000_VDDC_MAN_ASSN_VDDIO <<
                 SGTL5000_VDDC_MAN_ASSN_SHIFT;

	/* If PLL is powered up (such as on power cycle) leave it on. */
    SGTL5000_ReadReg(handle, SGTL5000_CHIP_ANA_POWER, &reg);
	ana_pwr |= reg & (SGTL5000_PLL_POWERUP | SGTL5000_VCOAMP_POWERUP);

	/* set line out ref voltage to vddio/2 */
    uint32_t vag = 3300 / 2;
	if (vag <= SGTL5000_LINE_OUT_GND_BASE)
		vag = 0;
	else if (vag >= SGTL5000_LINE_OUT_GND_BASE + SGTL5000_LINE_OUT_GND_STP *
		 SGTL5000_LINE_OUT_GND_MAX)
		vag = SGTL5000_LINE_OUT_GND_MAX;
	else
		vag = (vag - SGTL5000_LINE_OUT_GND_BASE) /
		    SGTL5000_LINE_OUT_GND_STP;
	lo_ctrl |= vag << SGTL5000_LINE_OUT_GND_SHIFT;

	/* enable small pop */
	ref_ctrl |= SGTL5000_SMALL_POP;

	/* Controls the output bias current for the lineout */
	lo_ctrl |=
	    (SGTL5000_LINE_OUT_CURRENT_360u << SGTL5000_LINE_OUT_CURRENT_SHIFT);

	/* set short detect */
	/* keep default */

	/* set routing */
	/* keep default, bypass DAP */

	SGTL5000_WriteReg(handle, SGTL5000_CHIP_LINREG_CTRL, lreg_ctrl);
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_ANA_POWER, ana_pwr);
	ms_delay(10);


	SGTL5000_WriteReg(handle, SGTL5000_CHIP_REF_CTRL, ref_ctrl);
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_LINE_OUT_CTRL, lo_ctrl);
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_SHORT_CTRL, short_ctrl);
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_SSS_CTRL, sss);
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_DIG_POWER, 0);

	reg = SGTL5000_DAC_VOL_RAMP_EN |
	    SGTL5000_DAC_MUTE_RIGHT | SGTL5000_DAC_MUTE_LEFT;
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_ADCDAC_CTRL, reg);

    SGTL5000_WriteReg(handle, SGTL5000_CHIP_PAD_STRENGTH, 0x015f);

	SGTL5000_ReadReg(handle,  SGTL5000_CHIP_ANA_ADC_CTRL, &reg);
	reg &= ~SGTL5000_ADC_VOL_M6DB;
	reg &= ~(SGTL5000_ADC_VOL_LEFT_MASK | SGTL5000_ADC_VOL_RIGHT_MASK);
	reg |= (0xf << SGTL5000_ADC_VOL_LEFT_SHIFT)
	    | (0xf << SGTL5000_ADC_VOL_RIGHT_SHIFT);
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_ANA_ADC_CTRL, reg);

	reg = SGTL5000_LINE_OUT_MUTE | SGTL5000_HP_MUTE |
	    SGTL5000_HP_ZCD_EN | SGTL5000_ADC_ZCD_EN;
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_ANA_CTRL, reg);

	SGTL5000_WriteReg(handle, SGTL5000_CHIP_MIC_CTRL, 0);
	SGTL5000_WriteReg(handle, SGTL5000_CHIP_CLK_TOP_CTRL, 0);
	/* disable DAP */
	SGTL5000_WriteReg(handle, SGTL5000_DAP_CTRL, 0);

#endif



    //--------------- Power Supply Configuration----------------
    // NOTE: This next 2 Write calls is needed ONLY if VDDD is
    // internally driven by the chip
    // Configure VDDD level to 1.2V (bits 3:0) Write CHIP_LINREG_CTRL 0x0008
    // Power up internal linear regulator (Set bit 9) Write CHIP_ANA_POWER 0x7260

    // NOTE: This next Write call is needed ONLY if VDDD is
    // externally driven
    // Turn off startup power supplies to save power (Clear bit 12 and 13) Write CHIP_ANA_POWER    0x4260
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_ANA_POWER, 0x4260);
    // NOTE: The next modify call is only needed if both VDDA and
    // VDDIO are greater than 3.1V
    // Configure the charge pump to use the VDDIO rail (set bit 5 and bit 6) Write CHIP_LINREG_CTRL   0x006C
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_LINREG_CTRL, 0x006C);

    //------ Reference Voltage and Bias Current Configuration----------
    // NOTE: The value written in the next 2 Write calls is dependent
    // on the VDDA voltage value.
    // Set ground, ADC, DAC reference voltage (bits 8:4). The value should
    // be set to VDDA/2. This example assumes VDDA = 1.8V. VDDA/2 = 0.9V.
    // The bias current should be set to 50% of the nominal value (bits 3:1) Write CHIP_REF_CTRL    0x004E
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_REF_CTRL, 0x004E);
    // Set LINEOUT reference voltage to VDDIO/2 (1.65 V) (bits 5:0)
    //and bias current (bits 11:8) to the recommended value of 0.36 mA
    //for 10 kOhm load with 1.0 nF capacitance
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_LINE_OUT_CTRL, 0x0322);

    //------------Other Analog Block Configurations--------------
    // Configure slow ramp up rate to minimize pop (bit 0)
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_REF_CTRL, 0x004F);
    // Enable short detect mode for headphone left/right
    // and center channel and set short detect current trip level
    // to 75 mA
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_SHORT_CTRL, 0x1106);
    // Enable Zero-cross detect if needed for HP_OUT (bit 5) and ADC (bit 1)
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_ANA_CTRL, 0x0133);

    //------------Power up Inputs/Outputs/Digital Blocks---------
    // Power up LINEOUT, HP, ADC, DAC
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_ANA_POWER, 0x6AFF);
    // Power up desired digital blocks
    // I2S_IN (bit 0), I2S_OUT (bit 1), DAP (bit 4), DAC (bit 5),
    // ADC (bit 6) are powered on
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_DIG_POWER, 0x0073);

    //----------------Set LINEOUT Volume Level-------------------
    // Set the LINEOUT volume level based on voltage reference (VAG)
    // values using this formula
    // Value = (int)(40*log(VAG_VAL/LO_VAGCNTRL) + 15)
    // Assuming VAG_VAL and LO_VAGCNTRL is set to 0.9 V and 1.65 V respectively, the
    // left LO vol (bits 12:8) and right LO volume (bits 4:0) value should be set 
    // to 5
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_LINE_OUT_VOL, 0x0505);

    //----------------Config System MCLK and Sample Clock-------------------
    // Configure SYS_FS clock to 48kHz
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_CLK_CTRL, SGTL5000_SYS_FS_MASK,  \
                               (SGTL5000_SYS_FS_48k << SGTL5000_SYS_FS_SHIFT));
    // Configure MCLK_FREQ to 256*Fs                               
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_CLK_CTRL, SGTL5000_MCLK_FREQ_MASK, \
                               (SGTL5000_MCLK_FREQ_256FS << SGTL5000_MCLK_FREQ_SHIFT));

    // Configure the I2S clocks in master mode
    // NOTE: I2S LRCLK is same as the system sample clock bit 7
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_I2S_CTRL, SGTL5000_I2S_MASTER, SGTL5000_I2S_MASTER);

    //Config left justified format
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_I2S_CTRL, SGTL5000_I2S_MODE_MASK, \
                               SGTL5000_I2S_MODE_I2S_LJ << SGTL5000_I2S_MODE_SHIFT);
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_I2S_CTRL, SGTL5000_I2S_LRALIGN, SGTL5000_I2S_LRALIGN);

    //----------------Input/Output Routing-------------------
    // Example 1: I2S_IN -> DAP -> DAC -> LINEOUT, HP_OUT
    // Route I2S_IN to DAP
    //Modify CHIP_SSS_CTRL->DAP_SELECT 0x0001 // bits 7:6
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_SSS_CTRL, SGTL5000_DAP_SEL_MASK, \
                               SGTL5000_DAP_SEL_I2S_IN << SGTL5000_DAP_SEL_SHIFT);
    // Route DAP to DAC
    //Modify CHIP_SSS_CTRL->DAC_SELECT 0x0003 // bits 5:4
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_SSS_CTRL, SGTL5000_DAC_SEL_MASK, \
                               SGTL5000_DAC_SEL_DAP << SGTL5000_DAC_SEL_SHIFT);
    // Select DAC as the input to HP_OUT
    //Modify CHIP_ANA_CTRL->SELECT_HP 0x0000 // bit 6
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_ANA_CTRL, SGTL5000_HP_SEL_MASK, \
                               SGTL5000_HP_SEL_DAC << SGTL5000_HP_SEL_SHIFT);
    #if 0
    // Example 2: MIC_IN -> ADC -> I2S_OUT
    // Set ADC input to MIC_IN
    Modify CHIP_ANA_CTRL->SELECT_ADC 0x0000 // bit 2
    // Route ADC to I2S_OUT
    Modify CHIP_SSS_CTRL->I2S_SELECT 0x0000 // bits 1:0
    // Example 3: LINEIN -> HP_OUT
    // Select LINEIN as the input to HP_OUT
    Modify CHIP_ANA_CTRL->SELECT_HP 0x0001 // bit 6
    #endif
    //----------------DIGITAL AUDIO PROCESSOR CONFIGURATION-------------------
    // Enable DAP block
    // NOTE: DAP will be in a pass-through mode if none of DAP
    // sub-blocks are enabled.
    //Modify DAP_CONTROL->DAP_EN 0x0001 // bit 0
    SGTL5000_ModifyReg(handle, SGTL5000_DAP_CONTROL, SGTL5000_DAP_EN, SGTL5000_DAP_EN);
    //----------------Dual Input Mixer-------------------
    // Enable Dual Input Mixer
    //Modify DAP_CONTROL->MIX_EN 0x0001 // bit 4
    SGTL5000_ModifyReg(handle, SGTL5000_DAP_CONTROL, SGTL5000_DAP_MIX_EN, SGTL5000_DAP_MIX_EN);
    // NOTE: This example assumes mix level of main and mix
    // channels as 100% and 50% respectively
    // Configure main channel volume to 100% (No change from input
    // level)
    //Write DAP_MAIN_CHAN 0x4000
    SGTL5000_WriteReg(handle, SGTL5000_DAP_MAIN_CHAN, 0x4000);
    // Configure mix channel volume to 50% (attenuate the mix
    // input level by half)
    //Write DAP_MIX_CHAN 0x4000
    SGTL5000_WriteReg(handle, SGTL5000_DAP_MIX_CHAN, 0x4000);

    //---------------Automatic Volume Control (AVC)-------------------
    // Configure threshold to -18dB
    //Write DAP_AVC_THRESHOLD 0x0A40
    SGTL5000_WriteReg(handle, SGTL5000_DAP_AVC_THRESHOLD, 0x0A40);
    // Configure attack rate to 16dB/s
    //Write DAP_AVC_ATTACK 0x0014
    SGTL5000_WriteReg(handle, SGTL5000_DAP_AVC_ATTACK, 0x0014);
    // Configure decay rate to 2dB/s
    //Write DAP_AVC_DECAY 0x0028
    SGTL5000_WriteReg(handle, SGTL5000_DAP_AVC_DECAY, 0x0028);

    //---------------- Input Volume Control---------------------
    // Configure ADC left and right analog volume to desired default.
    // Example shows volume of 0dB
    //Write CHIP_ANA_ADC_CTRL 0x0000
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_ANA_ADC_CTRL, 0x0000);
    // Configure MIC gain if needed. Example shows gain of 20dB
    //Modify CHIP_MIC_CTRL->GAIN 0x0001    // bits 1:0
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_MIC_CTRL, SGTL5000_MIC_GAIN_MASK, 1 << SGTL5000_MIC_GAIN_SHIFT);

    // LINEOUT and DAC volume control
    //Modify CHIP_ANA_CTRL->MUTE_LO 0x0000 // bit 8
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_ANA_CTRL, SGTL5000_LINE_OUT_MUTE, 0);
    // Configure DAC left and right digital volume. Example shows
    // volume of 0dB
    //Write CHIP_DAC_VOL 0x3C3C
    SGTL5000_WriteReg(handle, SGTL5000_CHIP_DAC_VOL, 0x3C3C);
    //Modify CHIP_ADCDAC_CTRL->DAC_MUTE_LEFT 0x0000 // bit 2
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_ADCDAC_CTRL, SGTL5000_VOL_BUSY_DAC_RIGHT, 0);
    //Modify CHIP_ADCDAC_CTRL->DAC_MUTE_RIGHT 0x0000 // bit 3
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_ADCDAC_CTRL, SGTL5000_VOL_BUSY_DAC_LEFT ,0);

    //adc mute
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_ANA_CTRL, SGTL5000_ADC_MUTE , SGTL5000_ADC_MUTE);

    return kStatus_Success;
}

status_t SGTL5000_Deinit(codec_handle_t *handle)
{
    SGTL5000_SetModule(handle, kSGTL5000_ModuleADC, false);
    SGTL5000_SetModule(handle, kSGTL5000_ModuleDAC, false);
    SGTL5000_SetModule(handle, kSGTL5000_ModuleVREF, false);
    SGTL5000_SetModule(handle, kSGTL5000_ModuleLineIn, false);
    SGTL5000_SetModule(handle, kSGTL5000_ModuleLineOut, false);
    SGTL5000_SetModule(handle, kSGTL5000_ModuleSpeaker, false);

    return kStatus_Success;
}

void SGTL5000_SetMasterSlave(codec_handle_t *handle, bool master)
{
    if (master == 1)
    {
        SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_MS_MASK, SGTL5000_IFACE1_MS(SGTL5000_IFACE1_MASTER));
    }
    else
    {
        SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_MS_MASK, SGTL5000_IFACE1_MS(SGTL5000_IFACE1_SLAVE));
    }
}

status_t SGTL5000_SetModule(codec_handle_t *handle, SGTL5000_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kSGTL5000_ModuleADC:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER1, SGTL5000_POWER1_ADCL_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER1_ADCL_SHIFT));
            SGTL5000_ModifyReg(handle, SGTL5000_POWER1, SGTL5000_POWER1_ADCR_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER1_ADCR_SHIFT));
            break;
        case kSGTL5000_ModuleDAC:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER2, SGTL5000_POWER2_DACL_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER2_DACL_SHIFT));
            SGTL5000_ModifyReg(handle, SGTL5000_POWER2, SGTL5000_POWER2_DACR_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER2_DACR_SHIFT));
            break;
        case kSGTL5000_ModuleVREF:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER1, SGTL5000_POWER1_VREF_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER1_VREF_SHIFT));
            break;
        case kSGTL5000_ModuleLineIn:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER1, SGTL5000_POWER1_AINL_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER1_AINL_SHIFT));
            SGTL5000_ModifyReg(handle, SGTL5000_POWER1, SGTL5000_POWER1_AINR_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER1_AINR_SHIFT));
            break;
        case kSGTL5000_ModuleLineOut:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER2, SGTL5000_POWER2_LOUT1_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER2_LOUT1_SHIFT));
            SGTL5000_ModifyReg(handle, SGTL5000_POWER2, SGTL5000_POWER2_ROUT1_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER2_ROUT1_SHIFT));
            break;
        case kSGTL5000_ModuleMICB:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER1, SGTL5000_POWER1_MICB_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER1_MICB_SHIFT));
            break;
        case kSGTL5000_ModuleSpeaker:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER2, SGTL5000_POWER2_SPKL_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER2_SPKL_SHIFT));
            SGTL5000_ModifyReg(handle, SGTL5000_POWER2, SGTL5000_POWER2_SPKR_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER2_SPKR_SHIFT));
            SGTL5000_WriteReg(handle, SGTL5000_CLASSD1, 0xF7);
            break;
        case kSGTL5000_ModuleMIC:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER3, SGTL5000_POWER3_LMIC_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER3_LMIC_SHIFT));
            SGTL5000_ModifyReg(handle, SGTL5000_POWER3, SGTL5000_POWER3_RMIC_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER3_RMIC_SHIFT));
            break;
        case kSGTL5000_ModuleOMIX:
            SGTL5000_ModifyReg(handle, SGTL5000_POWER3, SGTL5000_POWER3_LOMIX_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER3_LOMIX_SHIFT));
            SGTL5000_ModifyReg(handle, SGTL5000_POWER3, SGTL5000_POWER3_ROMIX_MASK,
                             ((uint16_t)isEnabled << SGTL5000_POWER3_ROMIX_SHIFT));
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t SGTL5000_SetDataRoute(codec_handle_t *handle, SGTL5000_route_t route)
{
    status_t ret = kStatus_Success;
    switch (route)
    {
        case kSGTL5000_RouteBypass:
            /* Bypass means from line-in to HP*/
            /*
             * Left LINPUT3 to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            SGTL5000_WriteReg(handle, SGTL5000_LOUTMIX, 0x80);

            /*
             * Right RINPUT3 to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            SGTL5000_WriteReg(handle, SGTL5000_ROUTMIX, 0x80);
            break;
        case kSGTL5000_RoutePlayback:
            /* Data route I2S_IN-> DAC-> HP */
            /*
             * Left DAC to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            SGTL5000_WriteReg(handle, SGTL5000_LOUTMIX, 0x100);

            /*
             * Right DAC to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            SGTL5000_WriteReg(handle, SGTL5000_ROUTMIX, 0x100);
            SGTL5000_WriteReg(handle, SGTL5000_POWER3, 0x0C);
            /* Set power for DAC */
            SGTL5000_SetModule(handle, kSGTL5000_ModuleDAC, true);
            SGTL5000_SetModule(handle, kSGTL5000_ModuleOMIX, true);
            SGTL5000_SetModule(handle, kSGTL5000_ModuleLineOut, true);
            break;
        case kSGTL5000_RoutePlaybackandRecord:
            /*
             * Left DAC to left output mixer, LINPUT3 left output mixer volume = 0dB
             */
            SGTL5000_WriteReg(handle, SGTL5000_LOUTMIX, 0x100);

            /*
             * Right DAC to right output mixer, RINPUT3 right output mixer volume = 0dB
             */
            SGTL5000_WriteReg(handle, SGTL5000_ROUTMIX, 0x100);
            SGTL5000_WriteReg(handle, SGTL5000_POWER3, 0x3C);
            SGTL5000_SetModule(handle, kSGTL5000_ModuleDAC, true);
            SGTL5000_SetModule(handle, kSGTL5000_ModuleADC, true);
            SGTL5000_SetModule(handle, kSGTL5000_ModuleLineIn, true);
            SGTL5000_SetModule(handle, kSGTL5000_ModuleOMIX, true);
            SGTL5000_SetModule(handle, kSGTL5000_ModuleLineOut, true);
            break;
        case kSGTL5000_RouteRecord:
            /* LINE_IN->ADC->I2S_OUT */
            /*
             * Left and right input boost, LIN3BOOST and RIN3BOOST = 0dB
             */
            SGTL5000_WriteReg(handle, SGTL5000_POWER3, 0x30);
            /* Power up ADC and AIN */
            SGTL5000_SetModule(handle, kSGTL5000_ModuleLineIn, true);
            SGTL5000_SetModule(handle, kSGTL5000_ModuleADC, true);
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t SGTL5000_SetLeftInput(codec_handle_t *handle, SGTL5000_input_t input)
{
    status_t ret = kStatus_Success;
    uint16_t val = 0;

    switch (input)
    {
        case kSGTL5000_InputSingleEndedMic:
            /* Only LMN1 enabled, LMICBOOST to 13db, LMIC2B enabled */
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINL_MASK | SGTL5000_POWER1_ADCL_MASK | SGTL5000_POWER1_MICB_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            ret = SGTL5000_WriteReg(handle, SGTL5000_LINPATH, 0x138);
            ret = SGTL5000_WriteReg(handle, SGTL5000_LINVOL, 0x117);
            break;
        case kSGTL5000_InputDifferentialMicInput2:
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINL_MASK | SGTL5000_POWER1_ADCL_MASK | SGTL5000_POWER1_MICB_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            ret = SGTL5000_WriteReg(handle, SGTL5000_LINPATH, 0x178);
            ret = SGTL5000_WriteReg(handle, SGTL5000_LINVOL, 0x117);
            break;
        case kSGTL5000_InputDifferentialMicInput3:
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINL_MASK | SGTL5000_POWER1_ADCL_MASK | SGTL5000_POWER1_MICB_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            ret = SGTL5000_WriteReg(handle, SGTL5000_LINPATH, 0x1B8);
            ret = SGTL5000_WriteReg(handle, SGTL5000_LINVOL, 0x117);
            break;
        case kSGTL5000_InputLineINPUT2:
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINL_MASK | SGTL5000_POWER1_ADCL_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            SGTL5000_ReadReg(handle, SGTL5000_INBMIX1, &val);
            val |= 0xE;
            ret = SGTL5000_WriteReg(handle, SGTL5000_INBMIX1, val);
            break;
        case kSGTL5000_InputLineINPUT3:
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINL_MASK | SGTL5000_POWER1_ADCL_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            SGTL5000_ReadReg(handle, SGTL5000_INBMIX1, &val);
            val |= 0x70;
            ret = SGTL5000_WriteReg(handle, SGTL5000_INBMIX1, val);
            break;
        default:
            break;
    }

    return ret;
}

status_t SGTL5000_SetRightInput(codec_handle_t *handle, SGTL5000_input_t input)
{
    status_t ret = kStatus_Success;
    uint16_t val = 0;

    switch (input)
    {
        case kSGTL5000_InputSingleEndedMic:
            /* Only LMN1 enabled, LMICBOOST to 13db, LMIC2B enabled */
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINR_MASK | SGTL5000_POWER1_ADCR_MASK | SGTL5000_POWER1_MICB_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            ret = SGTL5000_WriteReg(handle, SGTL5000_RINPATH, 0x138);
            ret = SGTL5000_WriteReg(handle, SGTL5000_RINVOL, 0x117);
            break;
        case kSGTL5000_InputDifferentialMicInput2:
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINR_MASK | SGTL5000_POWER1_ADCR_MASK | SGTL5000_POWER1_MICB_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            ret = SGTL5000_WriteReg(handle, SGTL5000_RINPATH, 0x178);
            ret = SGTL5000_WriteReg(handle, SGTL5000_RINVOL, 0x117);
            break;
        case kSGTL5000_InputDifferentialMicInput3:
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINR_MASK | SGTL5000_POWER1_ADCR_MASK | SGTL5000_POWER1_MICB_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            ret = SGTL5000_WriteReg(handle, SGTL5000_RINPATH, 0x1B8);
            ret = SGTL5000_WriteReg(handle, SGTL5000_RINVOL, 0x117);
            break;
        case kSGTL5000_InputLineINPUT2:
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINR_MASK | SGTL5000_POWER1_ADCR_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            SGTL5000_ReadReg(handle, SGTL5000_INBMIX2, &val);
            val |= 0xE;
            ret = SGTL5000_WriteReg(handle, SGTL5000_INBMIX2, val);
            break;
        case kSGTL5000_InputLineINPUT3:
            SGTL5000_ReadReg(handle, SGTL5000_POWER1, &val);
            val |= (SGTL5000_POWER1_AINR_MASK | SGTL5000_POWER1_ADCR_MASK);
            ret = SGTL5000_WriteReg(handle, SGTL5000_POWER1, val);
            SGTL5000_ReadReg(handle, SGTL5000_INBMIX2, &val);
            val |= 0x70;
            ret = SGTL5000_WriteReg(handle, SGTL5000_INBMIX2, val);
            break;
        default:
            break;
    }

    return ret;
}

status_t SGTL5000_SetProtocol(codec_handle_t *handle, SGTL5000_protocol_t protocol)
{
    status_t ret = kStatus_Success;
    switch (protocol)
    {
        case kSGTL5000_BusI2S:
            SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_FORMAT_MASK,
                             SGTL5000_IFACE1_FORMAT(SGTL5000_IFACE1_FORMAT_I2S));
            break;
        case kSGTL5000_BusLeftJustified:
            SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_FORMAT_MASK,
                             SGTL5000_IFACE1_FORMAT(SGTL5000_IFACE1_FORMAT_LJ));
            break;
        case kSGTL5000_BusRightJustified:
            SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_FORMAT_MASK,
                             SGTL5000_IFACE1_FORMAT(SGTL5000_IFACE1_FORMAT_RJ));
            break;
        case kSGTL5000_BusPCMA:
            SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_FORMAT_MASK,
                             SGTL5000_IFACE1_FORMAT(SGTL5000_IFACE1_FORMAT_DSP));
            SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_LRP_MASK, SGTL5000_IFACE1_LRP(SGTL5000_IFACE1_DSP_MODEA));
            break;
        case kSGTL5000_BusPCMB:
            SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_FORMAT_MASK,
                             SGTL5000_IFACE1_FORMAT(SGTL5000_IFACE1_FORMAT_DSP));
            SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_LRP_MASK, SGTL5000_IFACE1_LRP(SGTL5000_IFACE1_DSP_MODEB));
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_WL_MASK, SGTL5000_IFACE1_WL(SGTL5000_IFACE1_WL_32BITS));
    return ret;
}

status_t SGTL5000_SetVolume(codec_handle_t *handle, SGTL5000_module_t module, uint32_t volume)
{
    status_t ret = kStatus_Success;
    uint8_t vol = (volume & 0xFF);

    ret = SGTL5000_WriteReg(handle, SGTL5000_CHIP_ANA_HP_CTRL,  (uint16_t) (vol << 8 | vol));

    return ret;
}

uint32_t SGTL5000_GetVolume(codec_handle_t *handle, SGTL5000_module_t module)
{
    uint16_t vol = 0;
    switch (module)
    {
        case kSGTL5000_ModuleADC:
            SGTL5000_ReadReg(handle, SGTL5000_LADC, &vol);
            vol &= 0xFF;
            break;
        case kSGTL5000_ModuleDAC:
            SGTL5000_ReadReg(handle, SGTL5000_LDAC, &vol);
            vol &= 0xFF;
            break;
        case kSGTL5000_ModuleHP:
            SGTL5000_ReadReg(handle, SGTL5000_LOUT1, &vol);
            vol &= 0x7F;
            break;
        case kSGTL5000_ModuleLineOut:
            SGTL5000_ReadReg(handle, SGTL5000_LINVOL, &vol);
            vol &= 0x3F;
            break;
        default:
            vol = 0;
            break;
    }
    return vol;
}

status_t SGTL5000_SetMute(codec_handle_t *handle, SGTL5000_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kSGTL5000_ModuleADC:
            /*
             * Digital Mute
             */
            if (isEnabled)
            {
                ret = SGTL5000_WriteReg(handle, SGTL5000_LADC, 0x100);
                ret = SGTL5000_WriteReg(handle, SGTL5000_RADC, 0x100);
            }
            else
            {
                ret = SGTL5000_WriteReg(handle, SGTL5000_LADC, 0x1C3);
                ret = SGTL5000_WriteReg(handle, SGTL5000_RADC, 0x1C3);
            }
            break;
        case kSGTL5000_ModuleDAC:
            /*
             * Digital mute
             */
            if (isEnabled)
            {
                ret = SGTL5000_WriteReg(handle, SGTL5000_LDAC, 0x100);
                ret = SGTL5000_WriteReg(handle, SGTL5000_RDAC, 0x100);
            }
            else
            {
                ret = SGTL5000_WriteReg(handle, SGTL5000_LDAC, 0x1FF);
                ret = SGTL5000_WriteReg(handle, SGTL5000_RDAC, 0x1FF);
            }
            break;
        case kSGTL5000_ModuleHP:
            /*
             * Analog mute
             */
            if (isEnabled)
            {
                ret = SGTL5000_WriteReg(handle, SGTL5000_LOUT1, 0x100);
                ret = SGTL5000_WriteReg(handle, SGTL5000_ROUT1, 0x100);
            }
            else
            {
                ret = SGTL5000_WriteReg(handle, SGTL5000_LOUT1, 0x179);
                ret = SGTL5000_WriteReg(handle, SGTL5000_ROUT1, 0x179);
            }
            break;
        case kSGTL5000_ModuleLineOut:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t SGTL5000_ConfigDataFormat(codec_handle_t *handle, uint32_t mclk, uint32_t sample_rate, uint32_t bits)
{
    status_t retval = kStatus_Success;
    uint32_t div = 0;
    uint16_t val = 0;
    //----------------Config System MCLK and Sample Clock-------------------
    // Configure SYS_FS clock to 48kHz
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_CLK_CTRL, SGTL5000_SYS_FS_MASK,  \
                               (SGTL5000_SYS_FS_48k << SGTL5000_SYS_FS_SHIFT));
    // Configure MCLK_FREQ to 256*Fs                               
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_CLK_CTRL, SGTL5000_MCLK_FREQ_MASK, \
                               (SGTL5000_MCLK_FREQ_256FS << SGTL5000_MCLK_FREQ_SHIFT));
                               
    // Configure the I2S clocks in master mode
    // NOTE: I2S LRCLK is same as the system sample clock bit 7
    SGTL5000_ModifyReg(handle, SGTL5000_CHIP_I2S_CTRL, SGTL5000_I2S_MASTER, SGTL5000_I2S_MASTER);

    switch (bits)
    {
        case 16:
            retval = SGTL5000_ModifyReg(handle, SGTL5000_CHIP_I2S_CTRL, SGTL5000_I2S_DLEN_MASK,
                                      SGTL5000_I2S_DLEN_16 << SGTL5000_I2S_DLEN_SHIFT);
            break;
        case 20:
            retval = SGTL5000_ModifyReg(handle, SGTL5000_CHIP_I2S_CTRL, SGTL5000_I2S_DLEN_MASK,
                                      SGTL5000_I2S_DLEN_20 << SGTL5000_I2S_DLEN_SHIFT);
            break;
        case 24:
            retval = SGTL5000_ModifyReg(handle, SGTL5000_CHIP_I2S_CTRL, SGTL5000_I2S_DLEN_MASK,
                                      SGTL5000_I2S_DLEN_24 << SGTL5000_I2S_DLEN_SHIFT);
            break;
        case 32:
            retval = SGTL5000_ModifyReg(handle, SGTL5000_CHIP_I2S_CTRL, SGTL5000_I2S_DLEN_MASK,
                                      SGTL5000_I2S_DLEN_32 << SGTL5000_I2S_DLEN_SHIFT);
            break;
        default:
            retval = kStatus_InvalidArgument;
            break;
    }
#if 0
    /* Compute sample rate div, dac and adc are the same sample rate */
    div = mclk / sample_rate;
    if (div == 256)
    {
        val = 0;
    }
    else if (div > 256)
    {
        val = (((div / 256U) << 6U) | ((div / 256U) << 3U));
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    retval = SGTL5000_WriteReg(handle, SGTL5000_CLOCK1, val);

    /* Compute bclk div */
    div /= 64U;
    switch (div)
    {
        case 4:
        case 5:
        case 6:
            val = (0x1C0 | div);
            break;
        case 8:
            val = 0x1C7;
            break;
        case 11:
            val = 0x1C8;
            break;
        case 12:
            val = 0x1C9;
            break;
        case 16:
            val = 0x1CA;
            break;
        case 22:
            val = 0x1CB;
            break;
        case 24:
            val = 0x1CC;
            break;
        case 32:
            val = 0x1CF;
            break;
        default:
            val = 0;
            retval = kStatus_InvalidArgument;
            break;
    }

    retval = SGTL5000_WriteReg(handle, SGTL5000_CLOCK2, val);
    /*
     * Slave mode (MS = 0), LRP = 0, 32bit WL, left justified (FORMAT[1:0]=0b01)
     */
    switch (bits)
    {
        case 16:
            retval = SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_WL_MASK,
                                      SGTL5000_IFACE1_WL(SGTL5000_IFACE1_WL_16BITS));
            break;
        case 20:
            retval = SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_WL_MASK,
                                      SGTL5000_IFACE1_WL(SGTL5000_IFACE1_WL_20BITS));
            break;
        case 24:
            retval = SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_WL_MASK,
                                      SGTL5000_IFACE1_WL(SGTL5000_IFACE1_WL_24BITS));
            break;
        case 32:
            retval = SGTL5000_ModifyReg(handle, SGTL5000_IFACE1, SGTL5000_IFACE1_WL_MASK,
                                      SGTL5000_IFACE1_WL(SGTL5000_IFACE1_WL_32BITS));
            break;
        default:
            retval = kStatus_InvalidArgument;
            break;
    }
#endif
    return retval;
}

status_t SGTL5000_SetJackDetect(codec_handle_t *handle, bool isEnabled)
{
    uint8_t retval = 0;
    uint16_t val = 0;

    SGTL5000_ReadReg(handle, SGTL5000_ADDCTL2, &val);

    if (isEnabled)
    {
        val |= 0x40U;
    }
    else
    {
        val &= 0xCF;
    }

    retval = SGTL5000_WriteReg(handle, SGTL5000_ADDCTL2, val);

    return retval;
}

status_t SGTL5000_WriteReg(codec_handle_t *handle, uint16_t reg, uint16_t val)
{
    uint8_t retval = 0;

    retval = CODEC_I2C_WriteReg(handle->slaveAddress, kCODEC_RegAddr16Bit, reg, kCODEC_RegWidth16Bit, val,
                                handle->I2C_SendFunc);

    if (retval == kStatus_Success)
    {
        //reg_cache[reg] = val;
    }

    return retval;
}

status_t SGTL5000_ReadReg(codec_handle_t *handle, uint16_t reg, uint16_t *val)
{
    uint8_t retval = 0;

    retval = CODEC_I2C_ReadReg(handle->slaveAddress, kCODEC_RegAddr16Bit, reg, kCODEC_RegWidth16Bit, val,
                               handle->I2C_ReceiveFunc);
    if (retval == kStatus_Success)
    {
        //reg_cache[reg] = val;
    }

    return retval;
}

status_t SGTL5000_ModifyReg(codec_handle_t *handle, uint16_t reg, uint16_t mask, uint16_t val)
{
    uint8_t retval = 0;
    uint16_t reg_val = 0;
    retval = SGTL5000_ReadReg(handle, reg, &reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }
    reg_val &= (uint16_t)~mask;
    reg_val |= val;
    retval = SGTL5000_WriteReg(handle, reg, reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}
