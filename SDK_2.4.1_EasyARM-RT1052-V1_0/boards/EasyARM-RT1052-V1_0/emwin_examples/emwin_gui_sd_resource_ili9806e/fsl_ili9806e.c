/*
 * The Clear BSD License
 * Copyright 2017 NXP
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

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"

extern void ms_delay (uint32_t ms);

#define SIM_SPI_DELAY(us)       delay(us)

#define SIM_SPI_CS_SET()        GPIO_PinWrite(GPIO1, 28, 1U); SIM_SPI_DELAY(2)
#define SIM_SPI_CS_CLR()        GPIO_PinWrite(GPIO1, 28, 0U); SIM_SPI_DELAY(2)

#define SIM_SPI_SCK_SET()       {GPIO_PinWrite(GPIO1, 31, 1U); SIM_SPI_DELAY(1);}
#define SIM_SPI_SCK_CLR()       {GPIO_PinWrite(GPIO1, 31, 0U); SIM_SPI_DELAY(1);}

#define SIM_SPI_MOSI_SET()      {GPIO_PinWrite(GPIO1, 30, 1U); SIM_SPI_DELAY(1);}
#define SIM_SPI_MOSI_CLR()      {GPIO_PinWrite(GPIO1, 30, 0U); SIM_SPI_DELAY(1);}

#define SIM_SPI_MISO_GET()      GPIO_PinRead(GPIO1,29)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Static Code
 ******************************************************************************/
static void delay(uint32_t us)
{
    volatile uint32_t i = 0;
    while (us--)
        for (i = 0; i < 100; ++i)
        {
            __asm("NOP"); /* delay */
        }
}

//****************************************************************
static void spi_3wire_send_cmd (uint8_t cmd)
{
    unsigned char i;

    //SIM_SPI_CS_CLR();

    SIM_SPI_SCK_CLR();
    SIM_SPI_MOSI_CLR();
    SIM_SPI_SCK_SET();

    for(i = 0; i < 8; i++)
    {
        SIM_SPI_SCK_CLR(); 
        if (cmd & 0x80)
        {
            SIM_SPI_MOSI_SET();
        }
        else
        {
            SIM_SPI_MOSI_CLR();
        }
        SIM_SPI_SCK_SET();
        
        cmd = cmd << 1;
    }

    //SIM_SPI_CS_SET();
}

//***************************************************************
static void spi_3wire_send_data (uint8_t dat)
{
    unsigned char i;

    //SIM_SPI_CS_CLR();

    SIM_SPI_SCK_CLR();
    SIM_SPI_MOSI_SET();
    SIM_SPI_SCK_SET();

    for(i = 0; i < 8; i++)
    {
        SIM_SPI_SCK_CLR(); 
        if (dat & 0x80)
        {
            SIM_SPI_MOSI_SET();
        }
        else
        {
            SIM_SPI_MOSI_CLR();
        }
        SIM_SPI_SCK_SET();
        
        dat = dat << 1;
    }
    //SIM_SPI_CS_SET();
}

static uint8_t spi_3wire_read_byte (void)
{
    unsigned char i;
    uint8_t data = 0;

    //SIM_SPI_CS_CLR();

    for(i = 0; i < 8; i++)
    {
        data = data << 1;     

        SIM_SPI_SCK_CLR(); 
        SIM_SPI_SCK_SET();

        if (SIM_SPI_MISO_GET())
        {
            data += 1;
        }
    }
    //SIM_SPI_CS_SET();
    return data;
}


static void Ili9806e_ChangePage (uint8_t page)
{
    SIM_SPI_CS_CLR();
    spi_3wire_send_cmd(0xFF);        // Change Page
    spi_3wire_send_data(0xff); 
    spi_3wire_send_data(0x98); 
    spi_3wire_send_data(0x06); 
    spi_3wire_send_data(0x04); 
    spi_3wire_send_data(page);
    SIM_SPI_CS_SET(); 
}

static void Ili9806e_SendCmd (uint8_t cmd)
{
    SIM_SPI_CS_CLR(); 
    spi_3wire_send_cmd(cmd);
    SIM_SPI_CS_SET(); 
}

static void Ili9806e_WriteReg (uint8_t reg, uint8_t val)
{
    SIM_SPI_CS_CLR(); 
    spi_3wire_send_cmd(reg);  
    spi_3wire_send_data(val); 
    SIM_SPI_CS_SET(); 
}

static uint8_t Ili9806e_ReadReg (uint8_t reg_addr, int8_t parameter_index)
{
    uint8_t data = 0;

    Ili9806e_WriteReg(0xFE,(parameter_index | 0x80) );

    SIM_SPI_CS_CLR(); 
    spi_3wire_send_cmd(reg_addr); 
    data = spi_3wire_read_byte();      
    SIM_SPI_CS_SET(); 

    Ili9806e_WriteReg(0xFE,0x00 );

    return data;
}



static uint32_t Ili9806e_ReadId4 (void)
{
    uint8_t r_buf[4]; 

    Ili9806e_ChangePage(1);
    
    r_buf[0] = Ili9806e_ReadReg(0x00, 1);
    r_buf[1] = Ili9806e_ReadReg(0x01, 1);
    r_buf[2] = Ili9806e_ReadReg(0x02, 1);

    return ( (r_buf[0] << 16) | (r_buf[1] << 8) | (r_buf[2]) );
}


static void Ili9806e_PinInit (void)
{
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t pin_output = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
    gpio_pin_config_t pin_input = {kGPIO_DigitalInput, 1, kGPIO_NoIntmode};
    /* Init output LED GPIO. */
    GPIO_PinInit(GPIO1, 29, &pin_input);
    GPIO_PinInit(GPIO1, 28, &pin_output);
    GPIO_PinInit(GPIO1, 30, &pin_output);
    GPIO_PinInit(GPIO1, 31, &pin_output);
}
/*******************************************************************************
 * Code
 ******************************************************************************

//NOTE:VCC=3.3V,在进入RGB模式前必须要用SPI对屏进行初始化

/*************************************************/
void Ili9806e_Init(void)
{
    volatile uint32_t __ili9806e_id = 0;

    Ili9806e_PinInit();

#if 0  //Connect CAP_INIT Pin is RC Reset
    res=1;
    ms_ms_delay(1);
    res=0;
    ms_delay(10);
    res=1;
    ms_delay(200);
#endif
    __ili9806e_id = Ili9806e_ReadId4();

    if (__ili9806e_id != 0x00980604) {
        return;
    }

    /*************lcd init config************************************/

    // Change to Page 1 CMD 
    Ili9806e_ChangePage(0x01);      

    Ili9806e_WriteReg(0x08, 0x08); //Output    SDO

    Ili9806e_WriteReg(0x20, 0x00); //set DE/VSYNC mode: 01 VSYNC MODE; 00 : DE MODE

    Ili9806e_WriteReg(0x21, 0x01); //DE = 1 Active  

    Ili9806e_WriteReg(0x30, 0x02);//Resolution setting 0x02 : 480 X 800 

    Ili9806e_WriteReg(0x31, 0x00); //Inversion setting  00:Column inversion

    Ili9806e_WriteReg(0x40, 0x00); //BT DDVDH DDVDL 10,14,18 00	2XVCI

    Ili9806e_WriteReg(0x41, 0x33); //avdd +5.2v,avee-5.2v 33

    Ili9806e_WriteReg(0x42, 0x02); //VGL=DDVDH+VCIP -DDVDL,VGH=2DDVDL-VCIP  00

    Ili9806e_WriteReg(0x43, 0x89); //SET VGH clamp level +15v 

    Ili9806e_WriteReg(0x44, 0x86); //SET VGL clamp level -10v

    Ili9806e_WriteReg(0x46, 0x34); 

    Ili9806e_WriteReg(0x50, 0xA8); //VREG1 for positive Gamma

    Ili9806e_WriteReg(0x51, 0xA8); //VREG2 for negative Gamma

    Ili9806e_WriteReg(0x52, 0x00); //VCOM 

    Ili9806e_WriteReg(0x53, 0x66); //Forward Flicker VCOM 

    Ili9806e_WriteReg(0x54, 0x00); //VCOM

    Ili9806e_WriteReg(0x55, 0x66); //Backward Flicker

    Ili9806e_WriteReg(0x60, 0x07);

    Ili9806e_WriteReg(0x61, 0x04);

    Ili9806e_WriteReg(0x62, 0x08);

    Ili9806e_WriteReg(0x63, 0x04); 

    Ili9806e_WriteReg(0xA0, 0x00);  //Positive Gamma 
    Ili9806e_WriteReg(0xA1, 0x0F); 
    Ili9806e_WriteReg(0xA2, 0x17); 
    Ili9806e_WriteReg(0xA3, 0x0D); 
    Ili9806e_WriteReg(0xA4, 0x07); 
    Ili9806e_WriteReg(0xA5, 0x0B); 
    Ili9806e_WriteReg(0xA6, 0x07); 
    Ili9806e_WriteReg(0xA7, 0x06); 
    Ili9806e_WriteReg(0xA8, 0x06); 
    Ili9806e_WriteReg(0xA9, 0x09); 
    Ili9806e_WriteReg(0xAA, 0x12); 
    Ili9806e_WriteReg(0xAB, 0x0D); 
    Ili9806e_WriteReg(0xAC, 0x11); 
    Ili9806e_WriteReg(0xAD, 0x0F); 
    Ili9806e_WriteReg(0xAE, 0x0A); 
    Ili9806e_WriteReg(0xAF, 0x09); 

    Ili9806e_WriteReg(0xC0, 0x00); //Negative Gamma 
    Ili9806e_WriteReg(0xC1, 0x0F); 
    Ili9806e_WriteReg(0xC2, 0x17); 
    Ili9806e_WriteReg(0xC3, 0x0D); 
    Ili9806e_WriteReg(0xC4, 0x06); 
    Ili9806e_WriteReg(0xC5, 0x0B); 
    Ili9806e_WriteReg(0xC6, 0x07); 
    Ili9806e_WriteReg(0xC7, 0x06); 
    Ili9806e_WriteReg(0xC8, 0x06); 
    Ili9806e_WriteReg(0xC9, 0x09); 
    Ili9806e_WriteReg(0xCA, 0x12); 
    Ili9806e_WriteReg(0xCB, 0x0D); 
    Ili9806e_WriteReg(0xCC, 0x11); 
    Ili9806e_WriteReg(0xCD, 0x0F); 
    Ili9806e_WriteReg(0xCE, 0x0A); 
    Ili9806e_WriteReg(0xCF, 0x09); 

    // Change to Page 6 CMD for GIP timing   
    Ili9806e_ChangePage(0x06);       

    Ili9806e_WriteReg(0x00, 0x21); 
    Ili9806e_WriteReg(0x01, 0x0A); 
    Ili9806e_WriteReg(0x02, 0x00); 
    Ili9806e_WriteReg(0x03, 0x00); 
    Ili9806e_WriteReg(0x04, 0x32); 
    Ili9806e_WriteReg(0x05, 0x32); 
    Ili9806e_WriteReg(0x06, 0x98); 
    Ili9806e_WriteReg(0x07, 0x06); 
    Ili9806e_WriteReg(0x08, 0x05); 
    Ili9806e_WriteReg(0x09, 0x00); 
    Ili9806e_WriteReg(0x0A, 0x00); 
    Ili9806e_WriteReg(0x0B, 0x00); 
    Ili9806e_WriteReg(0x0C, 0x32); 
    Ili9806e_WriteReg(0x0D, 0x32); 
    Ili9806e_WriteReg(0x0E, 0x01); 
    Ili9806e_WriteReg(0x0F, 0x01);

    Ili9806e_WriteReg(0x10, 0xF0); 
    Ili9806e_WriteReg(0x11, 0xF0); 
    Ili9806e_WriteReg(0x12, 0x00); 
    Ili9806e_WriteReg(0x13, 0x00); 
    Ili9806e_WriteReg(0x14, 0x00); 
    Ili9806e_WriteReg(0x15, 0x43); 
    Ili9806e_WriteReg(0x16, 0x0B); 
    Ili9806e_WriteReg(0x17, 0x00); 
    Ili9806e_WriteReg(0x18, 0x00); 
    Ili9806e_WriteReg(0x19, 0x00); 
    Ili9806e_WriteReg(0x1A, 0x00); 
    Ili9806e_WriteReg(0x1B, 0x00); 
    Ili9806e_WriteReg(0x1C, 0x00); 
    Ili9806e_WriteReg(0x1D, 0x00); 

    Ili9806e_WriteReg(0x20, 0x01); 
    Ili9806e_WriteReg(0x21, 0x23); 
    Ili9806e_WriteReg(0x22, 0x45); 
    Ili9806e_WriteReg(0x23, 0x67); 
    Ili9806e_WriteReg(0x24, 0x01); 
    Ili9806e_WriteReg(0x25, 0x23); 
    Ili9806e_WriteReg(0x26, 0x45); 
    Ili9806e_WriteReg(0x27, 0x67); 

    Ili9806e_WriteReg(0x30, 0x01); 
    Ili9806e_WriteReg(0x31, 0x11); 
    Ili9806e_WriteReg(0x32, 0x00); 
    Ili9806e_WriteReg(0x33, 0x22); 
    Ili9806e_WriteReg(0x34, 0x22); 
    Ili9806e_WriteReg(0x35, 0xcb); 
    Ili9806e_WriteReg(0x36, 0xda); 
    Ili9806e_WriteReg(0x37, 0xad); 
    Ili9806e_WriteReg(0x38, 0xbc); 
    Ili9806e_WriteReg(0x39, 0x66); 
    Ili9806e_WriteReg(0x3A, 0x77); 
    Ili9806e_WriteReg(0x3B, 0x22); 
    Ili9806e_WriteReg(0x3C, 0x22); 
    Ili9806e_WriteReg(0x3D, 0x22); 
    Ili9806e_WriteReg(0x3E, 0x22); 
    Ili9806e_WriteReg(0x3F, 0x22); 
    Ili9806e_WriteReg(0x40, 0x22); 

    // Change to Page 7 CMD for GIP timing   
    Ili9806e_ChangePage(0x07);     

    Ili9806e_WriteReg(0x18, 0x1d); 
    Ili9806e_WriteReg(0x02, 0x77); 
    Ili9806e_WriteReg(0xE1, 0x79); 

    // Change to Page 0 CMD for Normal command 
    Ili9806e_ChangePage(0x00);     

    Ili9806e_WriteReg(0x36, 0x01);
    Ili9806e_WriteReg(0x3A, 0x70); //24BIT


    Ili9806e_SendCmd(0x11);
    ms_delay(120);
    Ili9806e_SendCmd(0x29);
    ms_delay(25);
}

//*******************************************
void Ili9806e_EnterSleep (void)
{
    spi_3wire_send_cmd(0x28);
    ms_delay(10);
    spi_3wire_send_cmd(0x10);
  
  }

//*********************************************************
void Ili9806e_ExitSleep (void)

 {
    spi_3wire_send_cmd(0x11);
    ms_delay(120);
    spi_3wire_send_cmd(0x29);

   }




