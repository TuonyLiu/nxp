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

#ifndef _FSL_SGTL5000_H_
#define _FSL_SGTL5000_H_

#include "fsl_codec_common.h"

/*!
 * @addtogroup SGTL5000
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Define the register address of SGTL5000. */
#define	SGTL5000_CHIP_ID							0x0000
#define	SGTL5000_CHIP_DIG_POWER						0x0002
#define	SGTL5000_CHIP_CLK_CTRL						0x0004
#define	SGTL5000_CHIP_I2S_CTRL						0x0006
#define	SGTL5000_CHIP_SSS_CTRL						0x000A
#define	SGTL5000_CHIP_ADCDAC_CTRL					0x000E
#define	SGTL5000_CHIP_DAC_VOL						0x0010
#define	SGTL5000_CHIP_PAD_STRENGTH					0x0014
#define	SGTL5000_CHIP_ANA_ADC_CTRL					0x0020
#define	SGTL5000_CHIP_ANA_HP_CTRL					0x0022
#define	SGTL5000_CHIP_ANA_CTRL						0x0024
#define	SGTL5000_CHIP_LINREG_CTRL					0x0026
#define	SGTL5000_CHIP_REF_CTRL						0x0028
#define	SGTL5000_CHIP_MIC_CTRL						0x002A
#define	SGTL5000_CHIP_LINE_OUT_CTRL					0x002C
#define	SGTL5000_CHIP_LINE_OUT_VOL					0x002E
#define	SGTL5000_CHIP_ANA_POWER						0x0030
#define	SGTL5000_CHIP_PLL_CTRL						0x0032
#define	SGTL5000_CHIP_CLK_TOP_CTRL					0x0034
#define	SGTL5000_SHIP_ANA_STATUS					0x0036
#define	SGTL5000_CHIP_ANA_TEST1						0x0038
#define	SGTL5000_CHIP_ANA_TEST2						0x003A
#define	SGTL5000_CHIP_SHORT_CTRL					0x003C
#define	SGTL5000_DAP_CONTROL						0x0100
#define	SGTL5000_DAP_PEQ							0x0102
#define	SGTL5000_DAP_BASS_ENHANCE					0x0104
#define	SGTL5000_DAP_BASS_ENHANCE_CTRL				0x0106
#define	SGTL5000_DAP_AUDIO_EQ						0x0108
#define	SGTL5000_DAP_SGTL_SURROUND					0x010A
#define	SGTL5000_DAP_FILTER_COEF_ACCESS				0x010C
#define	SGTL5000_DAP_COEF_WR_B0_MSB					0x010E
#define	SGTL5000_DAP_COEF_WR_B0_LSB					0x0110
#define	SGTL5000_DAP_AUDIO_EQ_BASS_BAND0			0x0116
#define	SGTL5000_DAP_AUDIO_EQ_BAND1					0x0118
#define	SGTL5000_DAP_AUDIO_EQ_BAND2					0x011A
#define	SGTL5000_DAP_AUDIO_EQ_BAND3					0x011C
#define	SGTL5000_DAP_AUDIO_EQ_TREBLE_BAND4			0x011E
#define	SGTL5000_DAP_MAIN_CHAN						0x0120
#define	SGTL5000_DAP_MIX_CHAN						0x0122
#define	SGTL5000_DAP_AVC_CTRL						0x0124
#define	SGTL5000_DAP_AVC_THRESHOLD					0x0126
#define	SGTL5000_DAP_AVC_ATTACK						0x0128
#define	SGTL5000_DAP_AVC_DECAY						0x012A

#define	SGTL5000_DAP_COEF_WR_B1_MSB					0x012C
#define	SGTL5000_DAP_COEF_WR_B1_LSB					0x012E
#define	SGTL5000_DAP_COEF_WR_B2_MSB					0x0130
#define	SGTL5000_DAP_COEF_WR_B2_LSB					0x0132
#define	SGTL5000_DAP_COEF_WR_A1_MSB					0x0134
#define	SGTL5000_DAP_COEF_WR_A1_LSB					0x0136
#define	SGTL5000_DAP_COEF_WR_A2_MSB					0x0138
#define	SGTL5000_DAP_COEF_WR_A2_LSB					0x013A


/*
 * Field Definitions.
 */

/*
 * SGTL5000_CHIP_ID
 */
#define SGTL5000_PARTID_MASK			0xff00
#define SGTL5000_PARTID_SHIFT			8
#define SGTL5000_PARTID_WIDTH			8
#define SGTL5000_PARTID_PART_ID		    0xa0
#define SGTL5000_REVID_MASK			    0x00ff
#define SGTL5000_REVID_SHIFT			0
#define SGTL5000_REVID_WIDTH			8

/*
 * SGTL5000_CHIP_DIG_POWER
 */
#define SGTL5000_ADC_EN				    0x0040
#define SGTL5000_DAC_EN				    0x0020
#define SGTL5000_DAP_POWERUP			0x0010
#define SGTL5000_I2S_OUT_POWERUP		0x0002
#define SGTL5000_I2S_IN_POWERUP			0x0001

/*
 * SGTL5000_CHIP_CLK_CTRL
 */
#define SGTL5000_RATE_MODE_MASK			0x0030
#define SGTL5000_RATE_MODE_SHIFT		4
#define SGTL5000_RATE_MODE_WIDTH		2
#define SGTL5000_RATE_MODE_DIV_1		0
#define SGTL5000_RATE_MODE_DIV_2		1
#define SGTL5000_RATE_MODE_DIV_4		2
#define SGTL5000_RATE_MODE_DIV_6		3
#define SGTL5000_SYS_FS_MASK			0x000c
#define SGTL5000_SYS_FS_SHIFT			2
#define SGTL5000_SYS_FS_WIDTH			2
#define SGTL5000_SYS_FS_32k			    0x0
#define SGTL5000_SYS_FS_44_1k			0x1
#define SGTL5000_SYS_FS_48k			    0x2
#define SGTL5000_SYS_FS_96k			    0x3
#define SGTL5000_MCLK_FREQ_MASK			0x0003
#define SGTL5000_MCLK_FREQ_SHIFT		0
#define SGTL5000_MCLK_FREQ_WIDTH		2
#define SGTL5000_MCLK_FREQ_256FS		0x0
#define SGTL5000_MCLK_FREQ_384FS		0x1
#define SGTL5000_MCLK_FREQ_512FS		0x2
#define SGTL5000_MCLK_FREQ_PLL			0x3

/*
 * SGTL5000_CHIP_I2S_CTRL
 */
#define SGTL5000_I2S_SCLKFREQ_MASK		0x0100
#define SGTL5000_I2S_SCLKFREQ_SHIFT		8
#define SGTL5000_I2S_SCLKFREQ_WIDTH		1
#define SGTL5000_I2S_SCLKFREQ_64FS		0x0
#define SGTL5000_I2S_SCLKFREQ_32FS		0x1	/* Not for RJ mode */
#define SGTL5000_I2S_MASTER			    0x0080
#define SGTL5000_I2S_SCLK_INV			0x0040
#define SGTL5000_I2S_DLEN_MASK			0x0030
#define SGTL5000_I2S_DLEN_SHIFT			4
#define SGTL5000_I2S_DLEN_WIDTH			2
#define SGTL5000_I2S_DLEN_32			0x0
#define SGTL5000_I2S_DLEN_24			0x1
#define SGTL5000_I2S_DLEN_20			0x2
#define SGTL5000_I2S_DLEN_16			0x3
#define SGTL5000_I2S_MODE_MASK			0x000c
#define SGTL5000_I2S_MODE_SHIFT			2
#define SGTL5000_I2S_MODE_WIDTH			2
#define SGTL5000_I2S_MODE_I2S_LJ		0x0
#define SGTL5000_I2S_MODE_RJ			0x1
#define SGTL5000_I2S_MODE_PCM			0x2
#define SGTL5000_I2S_LRALIGN			0x0002
#define SGTL5000_I2S_LRPOL			    0x0001	/* set for which mode */

/*
 * SGTL5000_CHIP_SSS_CTRL
 */
#define SGTL5000_DAP_MIX_LRSWAP			0x4000
#define SGTL5000_DAP_LRSWAP			    0x2000
#define SGTL5000_DAC_LRSWAP			    0x1000
#define SGTL5000_I2S_OUT_LRSWAP			0x0400
#define SGTL5000_DAP_MIX_SEL_MASK		0x0300
#define SGTL5000_DAP_MIX_SEL_SHIFT		8
#define SGTL5000_DAP_MIX_SEL_WIDTH		2
#define SGTL5000_DAP_MIX_SEL_ADC		0x0
#define SGTL5000_DAP_MIX_SEL_I2S_IN		0x1
#define SGTL5000_DAP_SEL_MASK			0x00c0
#define SGTL5000_DAP_SEL_SHIFT			6
#define SGTL5000_DAP_SEL_WIDTH			2
#define SGTL5000_DAP_SEL_ADC			0x0
#define SGTL5000_DAP_SEL_I2S_IN			0x1
#define SGTL5000_DAC_SEL_MASK			0x0030
#define SGTL5000_DAC_SEL_SHIFT			4
#define SGTL5000_DAC_SEL_WIDTH			2
#define SGTL5000_DAC_SEL_ADC			0x0
#define SGTL5000_DAC_SEL_I2S_IN			0x1
#define SGTL5000_DAC_SEL_DAP			0x3
#define SGTL5000_I2S_OUT_SEL_MASK		0x0003
#define SGTL5000_I2S_OUT_SEL_SHIFT		0
#define SGTL5000_I2S_OUT_SEL_WIDTH		2
#define SGTL5000_I2S_OUT_SEL_ADC		0x0
#define SGTL5000_I2S_OUT_SEL_I2S_IN		0x1
#define SGTL5000_I2S_OUT_SEL_DAP		0x3

/*
 * SGTL5000_CHIP_ADCDAC_CTRL
 */
#define SGTL5000_VOL_BUSY_DAC_RIGHT		0x2000
#define SGTL5000_VOL_BUSY_DAC_LEFT		0x1000
#define SGTL5000_DAC_VOL_RAMP_EN		0x0200
#define SGTL5000_DAC_VOL_RAMP_EXPO		0x0100
#define SGTL5000_DAC_MUTE_RIGHT			0x0008
#define SGTL5000_DAC_MUTE_LEFT			0x0004
#define SGTL5000_ADC_HPF_FREEZE			0x0002
#define SGTL5000_ADC_HPF_BYPASS			0x0001

/*
 * SGTL5000_CHIP_DAC_VOL
 */
#define SGTL5000_DAC_VOL_RIGHT_MASK		0xff00
#define SGTL5000_DAC_VOL_RIGHT_SHIFT    8
#define SGTL5000_DAC_VOL_RIGHT_WIDTH    8
#define SGTL5000_DAC_VOL_LEFT_MASK		0x00ff
#define SGTL5000_DAC_VOL_LEFT_SHIFT		0
#define SGTL5000_DAC_VOL_LEFT_WIDTH		8

/*
 * SGTL5000_CHIP_PAD_STRENGTH
 */
#define SGTL5000_PAD_I2S_LRCLK_MASK		0x0300
#define SGTL5000_PAD_I2S_LRCLK_SHIFT    8
#define SGTL5000_PAD_I2S_LRCLK_WIDTH    2
#define SGTL5000_PAD_I2S_SCLK_MASK		0x00c0
#define SGTL5000_PAD_I2S_SCLK_SHIFT		6
#define SGTL5000_PAD_I2S_SCLK_WIDTH		2
#define SGTL5000_PAD_I2S_DOUT_MASK		0x0030
#define SGTL5000_PAD_I2S_DOUT_SHIFT		4
#define SGTL5000_PAD_I2S_DOUT_WIDTH		2
#define SGTL5000_PAD_I2C_SDA_MASK		0x000c
#define SGTL5000_PAD_I2C_SDA_SHIFT		2
#define SGTL5000_PAD_I2C_SDA_WIDTH		2
#define SGTL5000_PAD_I2C_SCL_MASK		0x0003
#define SGTL5000_PAD_I2C_SCL_SHIFT		0
#define SGTL5000_PAD_I2C_SCL_WIDTH		2

/*
 * SGTL5000_CHIP_ANA_ADC_CTRL
 */
#define SGTL5000_ADC_VOL_M6DB			0x0100
#define SGTL5000_ADC_VOL_RIGHT_MASK		0x00f0
#define SGTL5000_ADC_VOL_RIGHT_SHIFT    4
#define SGTL5000_ADC_VOL_RIGHT_WIDTH    4
#define SGTL5000_ADC_VOL_LEFT_MASK		0x000f
#define SGTL5000_ADC_VOL_LEFT_SHIFT		0
#define SGTL5000_ADC_VOL_LEFT_WIDTH		4

/*
 * SGTL5000_CHIP_ANA_HP_CTRL
 */
#define SGTL5000_HP_VOL_RIGHT_MASK		0x7f00
#define SGTL5000_HP_VOL_RIGHT_SHIFT		8
#define SGTL5000_HP_VOL_RIGHT_WIDTH		7
#define SGTL5000_HP_VOL_LEFT_MASK		0x007f
#define SGTL5000_HP_VOL_LEFT_SHIFT		0
#define SGTL5000_HP_VOL_LEFT_WIDTH		7

/*
 * SGTL5000_CHIP_ANA_CTRL
 */
#define SGTL5000_LINE_OUT_MUTE		    0x0100
#define SGTL5000_HP_SEL_MASK			0x0040
#define SGTL5000_HP_SEL_SHIFT			6
#define SGTL5000_HP_SEL_WIDTH			1
#define SGTL5000_HP_SEL_DAC			    0x0
#define SGTL5000_HP_SEL_LINE_IN			0x1
#define SGTL5000_HP_ZCD_EN			    0x0020
#define SGTL5000_HP_MUTE			    0x0010
#define SGTL5000_ADC_SEL_MASK			0x0004
#define SGTL5000_ADC_SEL_SHIFT			2
#define SGTL5000_ADC_SEL_WIDTH			1
#define SGTL5000_ADC_SEL_MIC			0x0
#define SGTL5000_ADC_SEL_LINE_IN		0x1
#define SGTL5000_ADC_ZCD_EN			    0x0002
#define SGTL5000_ADC_MUTE			    0x0001

/*
 * SGTL5000_CHIP_LINREG_CTRL
 */
#define SGTL5000_VDDC_MAN_ASSN_MASK		0x0040
#define SGTL5000_VDDC_MAN_ASSN_SHIFT    6
#define SGTL5000_VDDC_MAN_ASSN_WIDTH    1
#define SGTL5000_VDDC_MAN_ASSN_VDDA		0x0
#define SGTL5000_VDDC_MAN_ASSN_VDDIO    0x1
#define SGTL5000_VDDC_ASSN_OVRD			0x0020
#define SGTL5000_LINREG_VDDD_MASK		0x000f
#define SGTL5000_LINREG_VDDD_SHIFT		0
#define SGTL5000_LINREG_VDDD_WIDTH		4

/*
 * SGTL5000_CHIP_REF_CTRL
 */
#define SGTL5000_ANA_GND_MASK			0x01f0
#define SGTL5000_ANA_GND_SHIFT			4
#define SGTL5000_ANA_GND_WIDTH			5
#define SGTL5000_ANA_GND_BASE			800	/* mv */
#define SGTL5000_ANA_GND_STP			25	/*mv */
#define SGTL5000_BIAS_CTRL_MASK			0x000e
#define SGTL5000_BIAS_CTRL_SHIFT		1
#define SGTL5000_BIAS_CTRL_WIDTH		3
#define SGTL5000_SMALL_POP			    0x0001

/*
 * SGTL5000_CHIP_MIC_CTRL
 */
#define SGTL5000_BIAS_R_MASK			0x0200
#define SGTL5000_BIAS_R_SHIFT			8
#define SGTL5000_BIAS_R_WIDTH			2
#define SGTL5000_BIAS_R_off			    0x0
#define SGTL5000_BIAS_R_2K			    0x1
#define SGTL5000_BIAS_R_4k			    0x2
#define SGTL5000_BIAS_R_8k			    0x3
#define SGTL5000_BIAS_VOLT_MASK			0x0070
#define SGTL5000_BIAS_VOLT_SHIFT		4
#define SGTL5000_BIAS_VOLT_WIDTH		3
#define SGTL5000_MIC_GAIN_MASK			0x0003
#define SGTL5000_MIC_GAIN_SHIFT			0
#define SGTL5000_MIC_GAIN_WIDTH			2

/*
 * SGTL5000_CHIP_LINE_OUT_CTRL
 */
#define SGTL5000_LINE_OUT_CURRENT_MASK		0x0f00
#define SGTL5000_LINE_OUT_CURRENT_SHIFT		8
#define SGTL5000_LINE_OUT_CURRENT_WIDTH		4
#define SGTL5000_LINE_OUT_CURRENT_180u		0x0
#define SGTL5000_LINE_OUT_CURRENT_270u		0x1
#define SGTL5000_LINE_OUT_CURRENT_360u		0x3
#define SGTL5000_LINE_OUT_CURRENT_450u		0x7
#define SGTL5000_LINE_OUT_CURRENT_540u		0xf
#define SGTL5000_LINE_OUT_GND_MASK		    0x003f
#define SGTL5000_LINE_OUT_GND_SHIFT		    0
#define SGTL5000_LINE_OUT_GND_WIDTH		    6
#define SGTL5000_LINE_OUT_GND_BASE		    800	/* mv */
#define SGTL5000_LINE_OUT_GND_STP		    25
#define SGTL5000_LINE_OUT_GND_MAX		    0x23

/*
 * SGTL5000_CHIP_LINE_OUT_VOL
 */
#define SGTL5000_LINE_OUT_VOL_RIGHT_MASK	0x1f00
#define SGTL5000_LINE_OUT_VOL_RIGHT_SHIFT	8
#define SGTL5000_LINE_OUT_VOL_RIGHT_WIDTH	5
#define SGTL5000_LINE_OUT_VOL_LEFT_MASK		0x001f
#define SGTL5000_LINE_OUT_VOL_LEFT_SHIFT	0
#define SGTL5000_LINE_OUT_VOL_LEFT_WIDTH	5

/*
 * SGTL5000_CHIP_ANA_POWER
 */
#define SGTL5000_DAC_STEREO			    0x4000
#define SGTL5000_LINREG_SIMPLE_POWERUP  0x2000
#define SGTL5000_STARTUP_POWERUP		0x1000
#define SGTL5000_VDDC_CHRGPMP_POWERUP   0x0800
#define SGTL5000_PLL_POWERUP			0x0400
#define SGTL5000_LINEREG_D_POWERUP		0x0200
#define SGTL5000_VCOAMP_POWERUP			0x0100
#define SGTL5000_VAG_POWERUP			0x0080
#define SGTL5000_ADC_STEREO			    0x0040
#define SGTL5000_REFTOP_POWERUP			0x0020
#define SGTL5000_HP_POWERUP			    0x0010
#define SGTL5000_DAC_POWERUP			0x0008
#define SGTL5000_CAPLESS_HP_POWERUP		0x0004
#define SGTL5000_ADC_POWERUP			0x0002
#define SGTL5000_LINE_OUT_POWERUP		0x0001

/*
 * SGTL5000_CHIP_PLL_CTRL
 */
#define SGTL5000_PLL_INT_DIV_MASK		0xf800
#define SGTL5000_PLL_INT_DIV_SHIFT		11
#define SGTL5000_PLL_INT_DIV_WIDTH		5
#define SGTL5000_PLL_FRAC_DIV_MASK		0x0700
#define SGTL5000_PLL_FRAC_DIV_SHIFT		0
#define SGTL5000_PLL_FRAC_DIV_WIDTH		11

/*
 * SGTL5000_CHIP_CLK_TOP_CTRL
 */
#define SGTL5000_INT_OSC_EN			    0x0800
#define SGTL5000_INPUT_FREQ_DIV2		0x0008

/*
 * SGTL5000_CHIP_ANA_STATUS
 */
#define SGTL5000_HP_LRSHORT			    0x0200
#define SGTL5000_CAPLESS_SHORT			0x0100
#define SGTL5000_PLL_LOCKED			    0x0010

/*
 * SGTL5000_CHIP_SHORT_CTRL
 */
#define SGTL5000_LVLADJR_MASK			0x7000
#define SGTL5000_LVLADJR_SHIFT			12
#define SGTL5000_LVLADJR_WIDTH			3
#define SGTL5000_LVLADJL_MASK			0x0700
#define SGTL5000_LVLADJL_SHIFT			8
#define SGTL5000_LVLADJL_WIDTH			3
#define SGTL5000_LVLADJC_MASK			0x0070
#define SGTL5000_LVLADJC_SHIFT			4
#define SGTL5000_LVLADJC_WIDTH			3
#define SGTL5000_LR_SHORT_MOD_MASK		0x000c
#define SGTL5000_LR_SHORT_MOD_SHIFT		2
#define SGTL5000_LR_SHORT_MOD_WIDTH		2
#define SGTL5000_CM_SHORT_MOD_MASK		0x0003
#define SGTL5000_CM_SHORT_MOD_SHIFT		0
#define SGTL5000_CM_SHORT_MOD_WIDTH		2

/*
 *SGTL5000_CHIP_ANA_TEST2
 */
#define SGTL5000_MONO_DAC			    0x1000

/*
 * SGTL5000_DAP_CTRL
 */
#define SGTL5000_DAP_MIX_EN			    0x0010
#define SGTL5000_DAP_EN				    0x0001


//////////////////////////////////////////////////////
#define SGTL5000_LINVOL 0x0
#define SGTL5000_RINVOL 0x1
#define SGTL5000_LOUT1 0x2
#define SGTL5000_ROUT1 0x3
#define SGTL5000_CLOCK1 0x4
#define SGTL5000_DACCTL1 0x5
#define SGTL5000_DACCTL2 0x6
#define SGTL5000_IFACE1 0x7
#define SGTL5000_CLOCK2 0x8
#define SGTL5000_IFACE2 0x9
#define SGTL5000_LDAC 0xa
#define SGTL5000_RDAC 0xb

#define SGTL5000_RESET 0xf
#define SGTL5000_3D 0x10
#define SGTL5000_ALC1 0x11
#define SGTL5000_ALC2 0x12
#define SGTL5000_ALC3 0x13
#define SGTL5000_NOISEG 0x14
#define SGTL5000_LADC 0x15
#define SGTL5000_RADC 0x16
#define SGTL5000_ADDCTL1 0x17
#define SGTL5000_ADDCTL2 0x18
#define SGTL5000_POWER1 0x19
#define SGTL5000_POWER2 0x1a
#define SGTL5000_ADDCTL3 0x1b
#define SGTL5000_APOP1 0x1c
#define SGTL5000_APOP2 0x1d

#define SGTL5000_LINPATH 0x20
#define SGTL5000_RINPATH 0x21
#define SGTL5000_LOUTMIX 0x22

#define SGTL5000_ROUTMIX 0x25
#define SGTL5000_MONOMIX1 0x26
#define SGTL5000_MONOMIX2 0x27
#define SGTL5000_LOUT2 0x28
#define SGTL5000_ROUT2 0x29
#define SGTL5000_MONO 0x2a
#define SGTL5000_INBMIX1 0x2b
#define SGTL5000_INBMIX2 0x2c
#define SGTL5000_BYPASS1 0x2d
#define SGTL5000_BYPASS2 0x2e
#define SGTL5000_POWER3 0x2f
#define SGTL5000_ADDCTL4 0x30
#define SGTL5000_CLASSD1 0x31

#define SGTL5000_CLASSD3 0x33
#define SGTL5000_PLL1 0x34
#define SGTL5000_PLL2 0x35
#define SGTL5000_PLL3 0x36
#define SGTL5000_PLL4 0x37

/*! @brief Cache register number */
#define SGTL5000_CACHEREGNUM 56

/*! @brief SGTL5000_IFACE1 FORMAT bits */
#define SGTL5000_IFACE1_FORMAT_MASK 0x03
#define SGTL5000_IFACE1_FORMAT_SHIFT 0x00
#define SGTL5000_IFACE1_FORMAT_RJ 0x00
#define SGTL5000_IFACE1_FORMAT_LJ 0x01
#define SGTL5000_IFACE1_FORMAT_I2S 0x02
#define SGTL5000_IFACE1_FORMAT_DSP 0x03
#define SGTL5000_IFACE1_FORMAT(x) ((x << SGTL5000_IFACE1_FORMAT_SHIFT) & SGTL5000_IFACE1_FORMAT_MASK)

/*! @brief SGTL5000_IFACE1 WL bits */
#define SGTL5000_IFACE1_WL_MASK 0x0C
#define SGTL5000_IFACE1_WL_SHIFT 0x02
#define SGTL5000_IFACE1_WL_16BITS 0x00
#define SGTL5000_IFACE1_WL_20BITS 0x01
#define SGTL5000_IFACE1_WL_24BITS 0x02
#define SGTL5000_IFACE1_WL_32BITS 0x03
#define SGTL5000_IFACE1_WL(x) ((x << SGTL5000_IFACE1_WL_SHIFT) & SGTL5000_IFACE1_WL_MASK)

/*! @brief SGTL5000_IFACE1 LRP bit */
#define SGTL5000_IFACE1_LRP_MASK 0x10
#define SGTL5000_IFACE1_LRP_SHIFT 0x04
#define SGTL5000_IFACE1_LRCLK_NORMAL_POL 0x00
#define SGTL5000_IFACE1_LRCLK_INVERT_POL 0x01
#define SGTL5000_IFACE1_DSP_MODEA 0x00
#define SGTL5000_IFACE1_DSP_MODEB 0x01
#define SGTL5000_IFACE1_LRP(x) ((x << SGTL5000_IFACE1_LRP_SHIFT) & SGTL5000_IFACE1_LRP_MASK)

/*! @brief SGTL5000_IFACE1 DLRSWAP bit */
#define SGTL5000_IFACE1_DLRSWAP_MASK 0x20
#define SGTL5000_IFACE1_DLRSWAP_SHIFT 0x05
#define SGTL5000_IFACE1_DACCH_NORMAL 0x00
#define SGTL5000_IFACE1_DACCH_SWAP 0x01
#define SGTL5000_IFACE1_DLRSWAP(x) ((x << SGTL5000_IFACE1_DLRSWAP_SHIFT) & SGTL5000_IFACE1_DLRSWAP_MASK)

/*! @brief SGTL5000_IFACE1 MS bit */
#define SGTL5000_IFACE1_MS_MASK 0x40
#define SGTL5000_IFACE1_MS_SHIFT 0x06
#define SGTL5000_IFACE1_SLAVE 0x00
#define SGTL5000_IFACE1_MASTER 0x01
#define SGTL5000_IFACE1_MS(x) ((x << SGTL5000_IFACE1_MS_SHIFT) & SGTL5000_IFACE1_MS_MASK)

/*! @brief SGTL5000_IFACE1 BCLKINV bit */
#define SGTL5000_IFACE1_BCLKINV_MASK 0x80
#define SGTL5000_IFACE1_BCLKINV_SHIFT 0x07
#define SGTL5000_IFACE1_BCLK_NONINVERT 0x00
#define SGTL5000_IFACE1_BCLK_INVERT 0x01
#define SGTL5000_IFACE1_BCLKINV(x) ((x << SGTL5000_IFACE1_BCLKINV_SHIFT) & SGTL5000_IFACE1_BCLKINV_MASK)

/*! @brief SGTL5000_IFACE1 ALRSWAP bit */
#define SGTL5000_IFACE1_ALRSWAP_MASK 0x100
#define SGTL5000_IFACE1_ALRSWAP_SHIFT 0x08
#define SGTL5000_IFACE1_ADCCH_NORMAL 0x00
#define SGTL5000_IFACE1_ADCCH_SWAP 0x01
#define SGTL5000_IFACE1_ALRSWAP(x) ((x << SGTL5000_IFACE1_ALRSWAP_SHIFT) & SGTL5000_IFACE1_ALRSWAP_MASK)

/*! @brief SGTL5000_POWER1 */
#define SGTL5000_POWER1_VREF_MASK 0x40
#define SGTL5000_POWER1_VREF_SHIFT 0x06

#define SGTL5000_POWER1_AINL_MASK 0x20
#define SGTL5000_POWER1_AINL_SHIFT 0x05

#define SGTL5000_POWER1_AINR_MASK 0x10
#define SGTL5000_POWER1_AINR_SHIFT 0x04

#define SGTL5000_POWER1_ADCL_MASK 0x08
#define SGTL5000_POWER1_ADCL_SHIFT 0x03

#define SGTL5000_POWER1_ADCR_MASK 0x04
#define SGTL5000_POWER1_ADCR_SHIFT 0x02

#define SGTL5000_POWER1_MICB_MASK 0x02
#define SGTL5000_POWER1_MICB_SHIFT 0x01

#define SGTL5000_POWER1_DIGENB_MASK 0x01
#define SGTL5000_POWER1_DIGENB_SHIFT 0x00

/*! @brief SGTL5000_POWER2 */
#define SGTL5000_POWER2_DACL_MASK 0x100
#define SGTL5000_POWER2_DACL_SHIFT 0x08

#define SGTL5000_POWER2_DACR_MASK 0x80
#define SGTL5000_POWER2_DACR_SHIFT 0x07

#define SGTL5000_POWER2_LOUT1_MASK 0x40
#define SGTL5000_POWER2_LOUT1_SHIFT 0x06

#define SGTL5000_POWER2_ROUT1_MASK 0x20
#define SGTL5000_POWER2_ROUT1_SHIFT 0x05

#define SGTL5000_POWER2_SPKL_MASK 0x10
#define SGTL5000_POWER2_SPKL_SHIFT 0x04

#define SGTL5000_POWER2_SPKR_MASK 0x08
#define SGTL5000_POWER2_SPKR_SHIFT 0x03

#define SGTL5000_POWER3_LMIC_MASK 0x20
#define SGTL5000_POWER3_LMIC_SHIFT 0x05
#define SGTL5000_POWER3_RMIC_MASK 0x10
#define SGTL5000_POWER3_RMIC_SHIFT 0x04
#define SGTL5000_POWER3_LOMIX_MASK 0x08
#define SGTL5000_POWER3_LOMIX_SHIFT 0x03
#define SGTL5000_POWER3_ROMIX_MASK 0x04
#define SGTL5000_POWER3_ROMIX_SHIFT 0x02

/* @brief SGTL5000 I2C address. */
#define SGTL5000_I2C_ADDR 0x0A

/*! @brief Modules in SGTL5000 board. */
typedef enum _SGTL5000_module
{
    kSGTL5000_ModuleADC = 0x0,     /*!< ADC module in SGTL5000 */
    kSGTL5000_ModuleDAC = 0x1,     /*!< DAC module in SGTL5000 */
    kSGTL5000_ModuleVREF = 0x2,    /*!< VREF module */
    kSGTL5000_ModuleHP = 0x03,     /*!< Headphone */
    kSGTL5000_ModuleMICB = 0x4,    /*!< Mic bias */
    kSGTL5000_ModuleMIC = 0x5,     /*!< Input Mic */
    kSGTL5000_ModuleLineIn = 0x6,  /*!< Analog in PGA  */
    kSGTL5000_ModuleLineOut = 0x7, /*!< Line out module */
    kSGTL5000_ModuleSpeaker = 0x8, /*!< Speaker module */
    kSGTL5000_ModuleOMIX = 0x9,    /*!< Output mixer */
} SGTL5000_module_t;

/*!
* @brief SGTL5000 data route.
* Only provide some typical data route, not all route listed.
* Note: Users cannot combine any routes, once a new route is set, the previous one would be replaced.
*/
typedef enum _SGTL5000_route
{
    kSGTL5000_RouteBypass = 0x0,            /*!< LINEIN->Headphone. */
    kSGTL5000_RoutePlayback = 0x1,          /*!<  I2SIN->DAC->Headphone. */
    kSGTL5000_RoutePlaybackandRecord = 0x2, /*!< I2SIN->DAC->Headphone, LINEIN->ADC->I2SOUT. */
    kSGTL5000_RouteRecord = 0x5             /*!< LINEIN->ADC->I2SOUT. */
} SGTL5000_route_t;

/*!
* @brief The audio data transfer protocol choice.
* SGTL5000 only supports I2S format and PCM format.
*/
typedef enum _SGTL5000_protocol
{
    kSGTL5000_BusI2S = 0x0,            /*!< I2S type */
    kSGTL5000_BusLeftJustified = 0x1,  /*!< Left justified mode */
    kSGTL5000_BusRightJustified = 0x2, /*!< Right justified mode */
    kSGTL5000_BusPCMA = 0x3,           /*!< PCM A mode */
    kSGTL5000_BusPCMB = 0x4            /*!< PCM B mode */
} SGTL5000_protocol_t;

typedef enum _SGTL5000_input
{
    kSGTL5000_InputClosed = 0x0,                /*!< Input device is closed */
    kSGTL5000_InputSingleEndedMic = 0x1,        /*!< Input as single ended mic, only use L/RINPUT1 */
    kSGTL5000_InputDifferentialMicInput2 = 0x2, /*!< Input as differential mic, use L/RINPUT1 and L/RINPUT2 */
    kSGTL5000_InputDifferentialMicInput3 = 0x3, /*!< Input as differential mic, use L/RINPUT1 and L/RINPUT3*/
    kSGTL5000_InputLineINPUT2 = 0x4,            /*!< Input as line input, only use L/RINPUT2 */
    kSGTL5000_InputLineINPUT3 = 0x5             /*!< Input as line input, only use L/RINPUT3 */
} SGTL5000_input_t;

/*! @brief Initialize structure of SGTL5000 */
typedef struct SGTL5000_config
{
    SGTL5000_route_t route;            /*!< Audio data route.*/
    SGTL5000_protocol_t bus;           /*!< Audio transfer protocol */
    bool master_slave;               /*!< Master or slave. */
    bool enableSpeaker;              /*!< True means enable class D speaker as output, false means no */
    SGTL5000_input_t leftInputSource;  /*!< Left input source for SGTL5000 */
    SGTL5000_input_t rightInputSource; /*!< Right input source for SGTL5000 */
} SGTL5000_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief SGTL5000 initialize function.
 *
 * The second parameter is NULL to SGTL5000 in this version. If users want
 * to change the settings, they have to use SGTL5000_write_reg() or SGTL5000_modify_reg()
 * to set the register value of SGTL5000.
 * Note: If the codec_config is NULL, it would initialize SGTL5000 using default settings.
 * The default setting:
 * codec_config->route = kSGTL5000_RoutePlaybackandRecord
 * codec_config->bus = kSGTL5000_BusI2S
 * codec_config->master = slave
 *
 * @param handle SGTL5000 handle structure.
 * @param SGTL5000_config SGTL5000 configuration structure.
 */
status_t SGTL5000_Init(codec_handle_t *handle, void *SGTL5000_config);

/*!
 * @brief Deinit the SGTL5000 codec.
 *
 * This function close all modules in SGTL5000 to save power.
 *
 * @param handle SGTL5000 handle structure pointer.
 */
status_t SGTL5000_Deinit(codec_handle_t *handle);

/*!
 * @brief Set audio data route in SGTL5000.
 *
 * This function would set the data route according to route. The route cannot be combined,
 * as all route would enable different modules.
 * Note: If a new route is set, the previous route would not work.
 *
 * @param handle SGTL5000 handle structure.
 * @param route Audio data route in SGTL5000.
 */
status_t SGTL5000_SetDataRoute(codec_handle_t *handle, SGTL5000_route_t route);

/*!
 * @brief Set left audio input source in SGTL5000.
 *
 * @param handle SGTL5000 handle structure.
 * @param input Audio input source.
 */
status_t SGTL5000_SetLeftInput(codec_handle_t *handle, SGTL5000_input_t input);

/*!
 * @brief Set right audio input source in SGTL5000.
 *
 * @param handle SGTL5000 handle structure.
 * @param input Audio input source.
 */
status_t SGTL5000_SetRightInput(codec_handle_t *handle, SGTL5000_input_t input);

/*!
 * @brief Set the audio transfer protocol.
 *
 * SGTL5000 only supports I2S, left justified, right justified, PCM A, PCM B format.
 *
 * @param handle SGTL5000 handle structure.
 * @param bus Audio data transfer protocol.
 */
status_t SGTL5000_SetProtocol(codec_handle_t *handle, SGTL5000_protocol_t protocol);

/*!
 * @brief Set SGTL5000 as master or slave.
 *
 * @param handle SGTL5000 handle structure.
 * @param master 1 represent master, 0 represent slave.
 */
void SGTL5000_SetMasterSlave(codec_handle_t *handle, bool master);

/*!
 * @brief Set the volume of different modules in SGTL5000.
 *
 * This function would set the volume of SGTL5000 modules. Uses need to appoint the module.
 * The function assume that left channel and right channel has the same volume.
 *
 * @param handle SGTL5000 handle structure.
 * @param module Module to set volume, it can be ADC, DAC, Headphone and so on.
 * @param volume Volume value need to be set.
 */
status_t SGTL5000_SetVolume(codec_handle_t *handle, SGTL5000_module_t module, uint32_t volume);

/*!
 * @brief Get the volume of different modules in SGTL5000.
 *
 * This function gets the volume of SGTL5000 modules. Uses need to appoint the module.
 * The function assume that left channel and right channel has the same volume.
 *
 * @param handle SGTL5000 handle structure.
 * @param module Module to set volume, it can be ADC, DAC, Headphone and so on.
 * @return Volume value of the module.
 */
uint32_t SGTL5000_GetVolume(codec_handle_t *handle, SGTL5000_module_t module);

/*!
 * @brief Mute modules in SGTL5000.
 *
 * @param handle SGTL5000 handle structure.
 * @param module Modules need to be mute.
 * @param isEnabled Mute or unmute, 1 represent mute.
 */
status_t SGTL5000_SetMute(codec_handle_t *handle, SGTL5000_module_t module, bool isEnabled);

/*!
 * @brief Enable/disable expected devices.
 *
 * @param handle SGTL5000 handle structure.
 * @param module Module expected to enable.
 * @param isEnabled Enable or disable moudles.
 */
status_t SGTL5000_SetModule(codec_handle_t *handle, SGTL5000_module_t module, bool isEnabled);

/*!
 * @brief Configure the data format of audio data.
 *
 * This function would configure the registers about the sample rate, bit depths.
 *
 * @param handle SGTL5000 handle structure pointer.
 * @param mclk Master clock frequency of I2S.
 * @param sample_rate Sample rate of audio file running in SGTL5000. SGTL5000 now
 * supports 8k, 11.025k, 12k, 16k, 22.05k, 24k, 32k, 44.1k, 48k and 96k sample rate.
 * @param bits Bit depth of audio file (SGTL5000 only supports 16bit, 20bit, 24bit
 * and 32 bit in HW).
 */
status_t SGTL5000_ConfigDataFormat(codec_handle_t *handle, uint32_t mclk, uint32_t sample_rate, uint32_t bits);

/*!
 * @brief Enable/disable jack detect feature.
 *
 * @param handle SGTL5000 handle structure.
 * @param isEnabled Enable or disable moudles.
 */
status_t SGTL5000_SetJackDetect(codec_handle_t *handle, bool isEnabled);

/*!
 * @brief Write register to SGTL5000 using I2C.
 *
 * @param handle SGTL5000 handle structure.
 * @param reg The register address in SGTL5000.
 * @param val Value needs to write into the register.
 */
status_t SGTL5000_WriteReg(codec_handle_t *handle, uint16_t reg, uint16_t val);

/*!
 * @brief Read register from SGTL5000 using I2C.
 * @param handle SGTL5000 handle structure.
 * @param reg The register address in SGTL5000.
 * @param val Value written to.
 */
status_t SGTL5000_ReadReg(codec_handle_t *handle, uint16_t reg, uint16_t *val);

/*!
 * @brief Modify some bits in the register using I2C.
 * @param handle SGTL5000 handle structure.
 * @param reg The register address in SGTL5000.
 * @param mask The mask code for the bits want to write. The bit you want to write should be 0.
 * @param val Value needs to write into the register.
 */
status_t SGTL5000_ModifyReg(codec_handle_t *handle, uint16_t reg, uint16_t mask, uint16_t val);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _FSL_SGTL5000_H_ */

/*******************************************************************************
 * API
 ******************************************************************************/
