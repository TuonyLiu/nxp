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


#define HBPD        60
#define HFPD        72
#define HSPW        20

#define VBPD        27
#define VFPD        20
#define VSPW        4


///////////////////////////////
static void spi_delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 1000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

static void delay_ms(uint32_t ms)
{
    volatile uint32_t i = 0;
    for (i = 0; i < ms; ++i)
    {
        spi_delay(); /* delay */
    }
}

#define SIM_SPI_DELAY(us)     delay_ms(us)

#define SSD2828_RST_SET()       {GPIO_PinWrite(GPIO1,  8, 1U); SIM_SPI_DELAY(1);}
#define SSD2828_RST_CLR()       {GPIO_PinWrite(GPIO1,  8, 0U); SIM_SPI_DELAY(1);}

#define SSD2828_RS_SET()        {GPIO_PinWrite(GPIO1, 27, 1U); SIM_SPI_DELAY(1);}
#define SSD2828_RS_CLR()        {GPIO_PinWrite(GPIO1, 27, 0U); SIM_SPI_DELAY(1);}

#define SSD2828_CS_SET()        {GPIO_PinWrite(GPIO1, 28, 1U); SIM_SPI_DELAY(1);}
#define SSD2828_CS_CLR()        {GPIO_PinWrite(GPIO1, 28, 0U); SIM_SPI_DELAY(1);}

#define SSD2828_SCK_SET()       {GPIO_PinWrite(GPIO1, 31, 1U); SIM_SPI_DELAY(1);}
#define SSD2828_SCK_CLR()       {GPIO_PinWrite(GPIO1, 31, 0U); SIM_SPI_DELAY(1);}

#define SSD2828_MOSI_SET()      {GPIO_PinWrite(GPIO1, 30, 1U); SIM_SPI_DELAY(1);}
#define SSD2828_MOSI_CLR()      {GPIO_PinWrite(GPIO1, 30, 0U); SIM_SPI_DELAY(1);}

#define SSD2828_MISO_GET()      GPIO_PinRead(GPIO1,29)


typedef enum ssd2828_send_flg {
    SSD2828_SEND_CMD = 0,
    SSD2828_SEND_DATA = 1,
} ssd2828_send_flg_t;

void ssd2828_write_byte (uint8_t bData, ssd2828_send_flg_t data_flg)
{
    // RS=0
    int i;

    SSD2828_CS_CLR();

    if (data_flg == SSD2828_SEND_CMD) {
        SSD2828_MOSI_CLR();  //BIT8=0 CMD
    } else {
        SSD2828_MOSI_SET();  //BIT8=1 DATA 
    }

    SSD2828_SCK_CLR();
    SSD2828_SCK_SET();

    for(i = 0; i < 8; i++)
    {

        if (bData & 0x80) {      //MSB
            SSD2828_MOSI_SET();
        } else {
            SSD2828_MOSI_CLR();
        }

        SSD2828_SCK_CLR();
        SSD2828_SCK_SET();

        bData <<= 1;
    }

    SSD2828_CS_SET();
}


uint8_t ssd2828_read_byte (void)
{
    uint8_t rdT = 0;

    for(int i = 0; i < 8; i++)
    {
        rdT = rdT<<1;        
        
        SSD2828_SCK_CLR();
        SSD2828_SCK_SET();
        
        if(SSD2828_MISO_GET()) rdT |= 0x01;
        

    }
    
    return rdT;
}


void ssd2828_write_reg(uint8_t reg_addr,uint16_t value)
{    
    SSD2828_CS_CLR();

    ssd2828_write_byte((reg_addr & 0xFF), SSD2828_SEND_CMD);
    ssd2828_write_byte((value&0xff), SSD2828_SEND_DATA);
    ssd2828_write_byte(((value>>8)&0xff), SSD2828_SEND_DATA);

    SSD2828_CS_SET();
}

uint16_t ssd2828_read_reg (uint8_t reg_addr)
{
    uint8_t rdTl = 0;
    uint8_t rdTh = 0;
    
    ssd2828_write_reg(0xD4, 0x00FA);

    SSD2828_CS_CLR();
    
    ssd2828_write_byte((reg_addr & 0xFF), SSD2828_SEND_CMD);

    ssd2828_write_byte(0xFA, SSD2828_SEND_CMD);

    rdTl = ssd2828_read_byte();
    rdTh = ssd2828_read_byte();

    SSD2828_CS_SET();
    
    return ((rdTh << 8) | rdTl);
}


uint16_t ssd2828_read_data (uint8_t *p_buf, uint8_t len)
{
    uint8_t rdTl = 0;
    uint8_t rdTh = 0;

    SSD2828_CS_CLR();

    for (int i = 0; i < len; i++) {
        ssd2828_write_byte(0xFA, SSD2828_SEND_CMD);

        rdTl = ssd2828_read_byte();
        rdTh = ssd2828_read_byte();
        //p_buf[i] = ((rdTh << 8) | rdTl);
        
        p_buf[i] = rdTl;
    }

    SSD2828_CS_SET();
    
    return 0;
}


int mipi_dsi_read_byte (uint8_t mipi_reg_addr, uint8_t *p_buf)
{
    uint16_t sd2828_int_status = 0;

    do {
        ssd2828_write_reg(0xB7, 0x0382);
        
        ssd2828_write_reg(0xBB, 0x0006); /*"XX" is the register for LP freq. setting Rx internal clock freq. must be similar to Tx */
        
        ssd2828_write_reg(0xC1, (1 & 0xFF) );

        ssd2828_write_reg(0xC0, 0x0100);   //?????????

        ssd2828_write_reg(0xBC, 0x0001);

        ssd2828_write_reg(0xBF, (mipi_reg_addr & 0xFF) );
        delay_ms(20);
        
        sd2828_int_status = ssd2828_read_reg(0xC6);

    } while (!(sd2828_int_status & 0x01));

    SSD2828_CS_CLR();
    
    ssd2828_write_byte(0xFF,SSD2828_SEND_CMD);
    ssd2828_read_data((void *)(p_buf), 1);
    
    SSD2828_CS_SET();
    
    return 0;
}

int mipi_dsi_read_bytes (uint8_t mipi_reg_addr, uint8_t *p_buf, int len)
{
    for (int i = 0; i < len; i++) {
        mipi_dsi_read_byte(mipi_reg_addr++,p_buf++);        
    }
    return 0;
}


//pack[0] = len;
void mipi_pack_send(unsigned char *pbuf)
{
    unsigned int i;
    unsigned char  num;
    num = *pbuf;

    ssd2828_write_byte(0xB7, SSD2828_SEND_CMD);
    ssd2828_write_byte(0x50, SSD2828_SEND_DATA);
    ssd2828_write_byte(0x02, SSD2828_SEND_DATA);

    ssd2828_write_byte(0xBD, SSD2828_SEND_CMD);
    ssd2828_write_byte(0x00, SSD2828_SEND_DATA);
    ssd2828_write_byte(0x00, SSD2828_SEND_DATA);

    ssd2828_write_byte(0xBC, SSD2828_SEND_CMD);
    ssd2828_write_byte(num , SSD2828_SEND_DATA);
    ssd2828_write_byte(0x00, SSD2828_SEND_DATA);
    //delay1(1);

    ssd2828_write_byte(0xbf, SSD2828_SEND_CMD);

    for(i = 0;i < num; i++){
        ssd2828_write_byte(*(pbuf + i + 1), SSD2828_SEND_DATA);
    }
}


uint32_t dsi_read_lcd_id(void)
{
    uint32_t _lcd_id = 0;
    
    mipi_dsi_read_bytes(0x04, (uint8_t *)&_lcd_id, 3);
    
    return _lcd_id;
}

void mipi_lcd_init(void) 
{  

    volatile uint16_t sd2828_id = 0x00;
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t pin_output = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
    gpio_pin_config_t pin_input = {kGPIO_DigitalInput, 1, kGPIO_NoIntmode};
    /* Init output LED GPIO. */
    GPIO_PinInit(GPIO1,  8, &pin_output);
    GPIO_PinInit(GPIO1, 28, &pin_output);
    GPIO_PinInit(GPIO1, 30, &pin_output);
    GPIO_PinInit(GPIO1, 31, &pin_output);


    GPIO_PinInit(GPIO1, 29, &pin_input);

	//// Reset LCD Driver//// 复位
	SSD2828_RST_SET();
	delay_ms(10); // Delay 1ms
    SSD2828_RST_CLR();
    delay_ms(200); // Delay 10ms // This Delay time is necessary
    SSD2828_RST_SET();
	delay_ms(200); // Delay 50 ms

    //SSD2828_Read ID
    sd2828_id = ssd2828_read_reg(0xB0);

    //SSD2825_Initial
	ssd2828_write_byte(0xb7,SSD2828_SEND_CMD);
	ssd2828_write_byte(0x50,SSD2828_SEND_DATA);//50=TX_CLK 70=PCLK
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);   //Configuration Register

	ssd2828_write_byte(0xb8,SSD2828_SEND_CMD);
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);   //VC(Virtual ChannelID) Control Register

	ssd2828_write_byte(0xb9,SSD2828_SEND_CMD);
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);//1=PLL disable
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);
    
	//TX_CLK/MS should be between 5Mhz to100Mhz
	ssd2828_write_byte(0xBA,SSD2828_SEND_CMD); //PLL=(TX_CLK/MS)*NS 8228=480M 4428=240M  061E=120M 4214=240M 821E=360M 8219=300M
	ssd2828_write_byte(0x1E,SSD2828_SEND_DATA);//D7-0=NS(0x01 : NS=1)
	ssd2828_write_byte(0x82,SSD2828_SEND_DATA);//D15-14=PLL范围 00=62.5-125 01=126-250 10=251-500 11=501-1000  DB12-8=MS(01:MS=1)

	ssd2828_write_byte(0xBB,SSD2828_SEND_CMD);//LP Clock Divider LP clock = 400MHz / LPD / 8 = 240 / 8 / 4 = 7.5MHz
	ssd2828_write_byte(0x06,SSD2828_SEND_DATA);//D5-0=LPD=0x1 – Divide by 2
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);

	ssd2828_write_byte(0xb9,SSD2828_SEND_CMD);
	ssd2828_write_byte(0x01,SSD2828_SEND_DATA);//1=PLL enable
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);
    delay_ms(500);

	//MIPI lane configuration
	ssd2828_write_byte(0xDE,SSD2828_SEND_CMD);//通道数
	ssd2828_write_byte(0x03,SSD2828_SEND_DATA);//11=4LANE 10=3LANE 01=2LANE 00=1LANE
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);

#if TEST_READ_MIPI_ID
    //mipi read test
    volatile uint32_t mipi_lcd_id = 0x00;
    mipi_lcd_id = dsi_read_lcd_id();
#endif

#if 1    //init code
    unsigned char tab1[]={4,0xB9,0xFF,0x83,0x94};
    unsigned char tab2[]={11,0xB1,0x48,0x14,0x74,0x09,0x33,0x54,0x71,0x31,0x57,0x2F};
    unsigned char tab3[]={7,0xBA,0x63,0x03,0x68,0x6B,0xB2,0xC0};
    unsigned char tab4[]={7,0xB2,0x00,0x80,0x64,0x06,0x0E,0x2F};
    unsigned char tab5[]={22,0xB4,0x01,0x70,0x01,0x70,0x01,0x70,0x01,0x05,0x86,0x35,0x00,0x3F,0x1C,0x70,0x01,0x70,0x01,0x70,0x01,0x05,0x86};
    unsigned char tab6[]={34,0xD3,0x00,0x00,0x00,0x00,0x08,0x08,0x10,0x10,0x32,0x10,0x03,0x00,0x03,0x32,0x15,0x07,0x05,0x07,0x32,0x10,0x08,0x00,0x00,0x17,0x01,0x07,0x07,0x17,0x05,0x05,0x17,0x06,0x40};
    unsigned char tab7[]={45,0xD5,0x18,0x18,0x18,0x18,0x26,0x27,0x24,0x25,0x06,0x07,0x04,0x05,0x02,0x03,0x00,0x01,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x20,0x21,0x22,0x23,0x18,0x18,0x18,0x18};
    unsigned char tab8[]={45,0xD6,0x18,0x18,0x18,0x18,0x21,0x20,0x23,0x22,0x01,0x00,0x03,0x02,0x05,0x04,0x07,0x06,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x27,0x26,0x25,0x24,0x18,0x18,0x18,0x18};
    unsigned char tab9[]={59,0xE0,0x00,0x02,0x06,0x0A,0x0C,0x0E,0x10,0x0D,0x1A,0x28,0x38,0x36,0x3F,0x50,0x59,0x61,0x72,0x7B,0x7D,0x93,0xAC,0x56,0x56,0x5C,0x62,0x67,0x71,0x7A,0x7F,0x00,0x02,0x06,0x0A,0x0B,0x0E,0x10,0x0D,0x1A,0x28,0x38,0x36,0x3F,0x50,0x59,0x61,0x73,0x7C,0x7D,0x94,0xAC,0x57,0x56,0x5C,0x62,0x67,0x71,0x7A,0x7F};
    unsigned char tab10[]={2,0xCC,0x0b};
    unsigned char tab11[]={3,0xC0,0x1F,0x73};
    unsigned char tab12[]={3,0xB6,0x3C,0x3C};
    unsigned char tab13[]={2,0xD4,0x02};
    unsigned char tab14[]={2,0xBD,0x01};
    unsigned char tab15[]={2,0x36,0x00};
    unsigned char tab16[]={2,0xB1,0x00};
    unsigned char tab17[]={2,0xBD,0x00};
    unsigned char tab18[]={2,0xC6,0xEF};
    unsigned char tab19[]={1,0x11};
    unsigned char tab20[]={1,0x29};
    //unsigned char tab21[]={1,0x2C};

    mipi_pack_send (tab1 );
    mipi_pack_send (tab2 );
    mipi_pack_send (tab3 );
    mipi_pack_send (tab4 );
    mipi_pack_send (tab5 );
    mipi_pack_send (tab6 );
    mipi_pack_send (tab7 );
    mipi_pack_send (tab8 );
    mipi_pack_send (tab9 );
    mipi_pack_send (tab10 );

    mipi_pack_send (tab11 );
    mipi_pack_send (tab12 );
    mipi_pack_send (tab13 );
    mipi_pack_send (tab14 );
    mipi_pack_send (tab15 );
    mipi_pack_send (tab16 );
    mipi_pack_send (tab17 );
    mipi_pack_send (tab18 );
    mipi_pack_send (tab19 );
    delay_ms(150);
    mipi_pack_send (tab20 );
    delay_ms(50);
    
#else   //self test

    // Set EXTC
    unsigned char tab1[]={4,0xB9,0xFF,0x83,0x94};
    // Set Power  VSPR=VSNR=5.0  VGH=16V,VGL=-10V
    unsigned char tab2[]={11,0xB1,0x48,0x14,0x74,0x09,0x33,0x54,0x71,0x31,0x57,0x2F};
    // Set MIPI
    unsigned char tab3[]={7,0xBA,0x63,0x03,0x68,0x6B,0xB2,0xC0};
    // Set Display
    unsigned char tab4[]={7,0xB2,0x00,0x80,0x64,0x06,0x0E,0x2F};
    //Modified By GRG For Line Imagesticking
    unsigned char tab5[]={22,0xB4,0x01,0x70,0x01,0x70,0x01,0x70,0x01,0x05,0x86,0x35,0x00,0x3F,0x1C,0x70,0x01,0x70,0x01,0x70,0x01,0x05,0x86};
    // Set D3H
    unsigned char tab6[]={34,0xD3,0x00,0x00,0x00,0x00,0x08,0x08,0x10,0x10,0x32,0x10,0x03,0x00,0x03,0x32,0x15,0x07,0x05,0x07,0x32,0x10,0x08,0x00,0x00,0x17,0x01,0x07,0x07,0x17,0x05,0x05,0x17,0x06,0x40};
    // Set GIP
    unsigned char tab7[]={45,0xD6,0x18,0x18,0x18,0x18,0x21,0x20,0x23,0x22,0x01,0x00,0x03,0x02,0x05,0x04,0x07,0x06,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x27,0x26,0x25,0x24,0x18,0x18,0x18,0x18};
    //Set Gamma
    unsigned char tab8[]={59,0xE0,0x00,0x02,0x06,0x0A,0x0C,0x0E,0x10,0x0D,0x1A,0x28,0x38,0x36,0x3F,0x50,0x59,0x61,0x72,0x7B,0x7D,0x93,0xAC,0x56,0x56,0x5C,0x62,0x67,0x71,0x7A,0x7F,0x00,0x02,0x06,0x0A,0x0B,0x0E,0x10,0x0D,0x1A,0x28,0x38,0x36,0x3F,0x50,0x59,0x61,0x73,0x7C,0x7D,0x94,0xAC,0x57,0x56,0x5C,0x62,0x67,0x71,0x7A,0x7F};
    // Set Panel
    unsigned char tab9[]={2,0xCC,0x0b};
    // Set C0H
    unsigned char tab10[]={3,0xC0,0x1F,0x73};
    // Set VCOM
    unsigned char tab11[]={3,0xB6,0x3C,0x3C};
    // Set D4h
    unsigned char tab12[]={2,0xD4,0x02};
    // Set BDH
    unsigned char tab13[]={2,0xBD,0x01};

    unsigned char tab14[]={2,0x36,0x00};
    // Set GAS
    unsigned char tab15[]={2,0xB1,0x00};
    // Set BD
    unsigned char tab16[]={2,0xBD,0x00};
    
    unsigned char tab17[]={2,0xC6,0xEF};
    //Enable DISP_BIST_EN to start self test
    unsigned char tab18[]={12,0xB2,0x40,0x80,0x64,0x08,0x08,0x2F,0x00,0x00,0x00,0x00,0xC8};
    // Sleep Out
    unsigned char tab19[]={1,0x11};
    // Display ON
    unsigned char tab20[]={1,0x29};
    
    mipi_pack_send (tab1 );
    mipi_pack_send (tab2 );
    mipi_pack_send (tab3 );
    mipi_pack_send (tab4 );
    mipi_pack_send (tab5 );
    mipi_pack_send (tab6 );
    mipi_pack_send (tab7 );
    mipi_pack_send (tab8 );
    mipi_pack_send (tab9 );
    mipi_pack_send (tab10 );
    mipi_pack_send (tab11 );
    mipi_pack_send (tab12 );
    mipi_pack_send (tab13 );
    mipi_pack_send (tab14 );
    mipi_pack_send (tab15 );
    mipi_pack_send (tab16 );
    mipi_pack_send (tab17 );
    mipi_pack_send (tab18 );
    mipi_pack_send (tab19 );
    delay_ms(150);
    mipi_pack_send (tab20 );
    delay_ms(50);
    
    while(1);
#endif

	//SSD2825_Initial

	ssd2828_write_byte(0xc9,SSD2828_SEND_CMD);
	ssd2828_write_byte(0x02,SSD2828_SEND_DATA);
	ssd2828_write_byte(0x23,SSD2828_SEND_DATA);   //p1: HS-Data-zero  p2: HS-Data- prepare  --> 8031 issue

	ssd2828_write_byte(0xCA,SSD2828_SEND_CMD);
	ssd2828_write_byte(0x02,SSD2828_SEND_DATA);   //CLK Prepare
	ssd2828_write_byte(0x23,SSD2828_SEND_DATA);   //Clk Zero

	ssd2828_write_byte(0xCB,SSD2828_SEND_CMD); //local_write_reg(addr=0xCB,data=0x0510)
	ssd2828_write_byte(0x02,SSD2828_SEND_DATA); //Clk Post
	ssd2828_write_byte(0x23,SSD2828_SEND_DATA); //Clk Per

	ssd2828_write_byte(0xCC,SSD2828_SEND_CMD); //local_write_reg(addr=0xCC,data=0x100A)
	ssd2828_write_byte(0x05,SSD2828_SEND_DATA); //HS Trail
	ssd2828_write_byte(0x10,SSD2828_SEND_DATA); //Clk Trail

	ssd2828_write_byte(0xD0,SSD2828_SEND_CMD);
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);

	//RGB interface configuration
	ssd2828_write_byte(0xB1,SSD2828_SEND_CMD);
	ssd2828_write_byte(HSPW,SSD2828_SEND_DATA);
	ssd2828_write_byte(VSPW,SSD2828_SEND_DATA);

	ssd2828_write_byte(0xB2,SSD2828_SEND_CMD);
	ssd2828_write_byte(HBPD,SSD2828_SEND_DATA);
	ssd2828_write_byte(VBPD,SSD2828_SEND_DATA);

	ssd2828_write_byte(0xB3,SSD2828_SEND_CMD);
	ssd2828_write_byte(HFPD,SSD2828_SEND_DATA);
	ssd2828_write_byte(VFPD,SSD2828_SEND_DATA);

	ssd2828_write_byte(0xB4,SSD2828_SEND_CMD);//Horizontal active period 720=02D0
	ssd2828_write_byte(0xD0,SSD2828_SEND_DATA);//013F=319 02D0=720
	ssd2828_write_byte(0x02,SSD2828_SEND_DATA);//HACT=0x01E0=480	,0x021c=540

	ssd2828_write_byte(0xB5,SSD2828_SEND_CMD);//Vertical active period 1280=0500
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);//01DF=479 0500=1280
	ssd2828_write_byte(0x05,SSD2828_SEND_DATA);//VACT=0x0320=800	,0x03c0=960	 ,0x0356=854

	ssd2828_write_byte(0xB6,SSD2828_SEND_CMD);//RGB CLK  16BPP=00 18BPP=01
	ssd2828_write_byte(0x24,SSD2828_SEND_DATA);//D7=0 D6=0 D5=0  D1-0=11 – 24bpp
	ssd2828_write_byte(0x21,SSD2828_SEND_DATA);//D15=VS D14=HS D13=CLK D12-9=NC D8=0=Video with blanking packet. 00-F0

	//MIPI lane configuration
	ssd2828_write_byte(0xDE,SSD2828_SEND_CMD);//通道数
	ssd2828_write_byte(0x03,SSD2828_SEND_DATA);//11=4LANE 10=3LANE 01=2LANE 00=1LANE
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);

	ssd2828_write_byte(0xD6,SSD2828_SEND_CMD);//  05=BGR  04=RGB
	ssd2828_write_byte(0x04,SSD2828_SEND_DATA);//D0=0=RGB 1:BGR D1=1=Most significant byte sent first
	ssd2828_write_byte(0x00,SSD2828_SEND_DATA);

	ssd2828_write_byte(0xB7,SSD2828_SEND_CMD);
	ssd2828_write_byte(0x4b,SSD2828_SEND_DATA);    //5b
	ssd2828_write_byte(0x03,SSD2828_SEND_DATA);  //02

	ssd2828_write_byte(0x2C,SSD2828_SEND_CMD);
	//W_C(0x3C);

}  

void Board_MipiPinInit (void)
{
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08,0U);
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_GPIO1_IO26,0U);
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_11_GPIO1_IO27,0U);
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_12_GPIO1_IO28,0U);
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_GPIO1_IO29,0U);
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_GPIO1_IO30,0U);
  IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_GPIO1_IO31,0U);

  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0x10B0u);
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_10_GPIO1_IO26, 0x10B0u);
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_11_GPIO1_IO27, 0x10B0u);
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_12_GPIO1_IO28, 0x10B0u);
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_13_GPIO1_IO29, 0x10B0u);
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_14_GPIO1_IO30, 0x10B0u);
  IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_15_GPIO1_IO31, 0x10B0u);
}


