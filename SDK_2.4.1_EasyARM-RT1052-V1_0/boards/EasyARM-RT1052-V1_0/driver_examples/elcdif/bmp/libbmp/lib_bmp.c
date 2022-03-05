/*******************************************************************************
* -----------------------------------------------------------------------------
*									     									 
* i2c.h - definitions for the i2c-bus interface			     			 
*									     
* -----------------------------------------------------------------------------
* Copyright (C) Damon Zhang
* All rights reserved.
*
* Author : YunFeng Zhang
* Website: https://damon-yun.github.io/blog.github.io/
* E-mail : damonzhang92@foxmail.com
*
* -----------------------------------------------------------------------------
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
*
*******************************************************************************/

/**
 * \file
 * \brief sim spi driver
 * 
 * \internal
 * \par Modification History
 * - 1.00 16-10-13  yunfeng.zhang, first implementation.
 * \endinternal
 */


#include "lib_bmp.h"
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
/*******************************************************************************
 * Private define region: constant & MACRO defined here                              
 ******************************************************************************/
bmp_file_t __bmp_file;

/*******************************************************************************
 * extern region: extern global variable & function prototype                 
 ******************************************************************************/
extern int bmp_hw_file_read(void *p_desc, void *p_src, uint32_t offsize, uint32_t size);
extern int bmp_hw_file_write(void *p_desc, void *p_src, uint32_t offsize, uint32_t size);

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void __bmp_head_info_printf (bmp_file_t *p_bmp)
{
    BMP_PRINTF("--------BMP FILE HEADER--------\r\n");
    BMP_PRINTF("header|-bfType-----------    %c%c\r\n",(p_bmp->bmpHeader.bfType)&0xFF, \
                                                       (p_bmp->bmpHeader.bfType >> 8)&0xFF);
    BMP_PRINTF("      |-bfSize-----------%6d\r\n",(p_bmp->bmpHeader.bfSize));
    BMP_PRINTF("      |-bfOffBits--------%6d\r\n",(p_bmp->bmpHeader.bfOffBits));
    BMP_PRINTF("-------------------------------\r\n");
    BMP_PRINTF("info--|-biSize-----------%6d\r\n",(p_bmp->bmpInfo.biSize));
    BMP_PRINTF("      |-biWidth----------%6d\r\n",(p_bmp->bmpInfo.biWidth));
    BMP_PRINTF("      |-biHeight---------%6d\r\n",(p_bmp->bmpInfo.biHeight));
    BMP_PRINTF("      |-biPlanes---------%6d\r\n",(p_bmp->bmpInfo.biPlanes));
    BMP_PRINTF("      |-biBitCount-------%6d\r\n",(p_bmp->bmpInfo.biBitCount));
    BMP_PRINTF("      |-biCompression----%6d\r\n",(p_bmp->bmpInfo.biCompression));
    BMP_PRINTF("      |-biSizeImage------%6d\r\n",(p_bmp->bmpInfo.biSizeImage));
    BMP_PRINTF("      |-biXPelsPerMeter--%6d\r\n",(p_bmp->bmpInfo.biXPelsPerMeter));
    BMP_PRINTF("      |-biYPelsPerMeter--%6d\r\n",(p_bmp->bmpInfo.biYPelsPerMeter));
    BMP_PRINTF("      |-biClrUsed--------%6d\r\n",(p_bmp->bmpInfo.biClrUsed));
    BMP_PRINTF("      |-biClrImportant---%6d\r\n",(p_bmp->bmpInfo.biClrImportant));
    BMP_PRINTF("-------------------------------\r\n"); 
}


int bmp_hw_file_read(void *p_desc, void *p_src, uint32_t offsize, uint32_t size)
{
    memcpy(p_desc, p_src, sizeof(bmp_header_t) + sizeof(bmp_info_t));
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * \brief  
 */
int bmp_open (uint8_t *p_bmp_file)
{
    bmp_hw_file_read((&__bmp_file),p_bmp_file, 0, sizeof(bmp_header_t) + sizeof(bmp_info_t));

    if (__bmp_file.bmpHeader.bfType != 0x4d42) {
        BMP_PRINTF("Err: The file is not a valid BMP file!\r\n"); 
        return -1;
    }

    if ((__bmp_file.bmpInfo.biBitCount) == 24 ||
        (__bmp_file.bmpInfo.biBitCount) == 16 ||
        (__bmp_file.bmpInfo.biBitCount) == 32) {   /* Ture Color */

        __bmp_file.pbiClrTable = NULL;
        __bmp_file.biClrTableCount = 0;

    } else {    /* Init bitmap color table */
        __bmp_file.biClrTableCount = (1 << (__bmp_file.bmpInfo.biBitCount));

        __bmp_file.pbiClrTable =  \
                (bmp_clr_table_t *) malloc( (__bmp_file.biClrTableCount) * sizeof(bmp_clr_table_t) );

        if (__bmp_file.pbiClrTable != NULL) {
            bmp_hw_file_read( __bmp_file.pbiClrTable, \
                              p_bmp_file , \
                              (sizeof(bmp_header_t) + __bmp_file.bmpInfo.biSize), \
                              ((__bmp_file.biClrTableCount) * sizeof(bmp_clr_table_t)) );
        } else {
            BMP_PRINTF("Err: Malloc mem fail\r\n"); 
            return -1;
        }
    }
   // __bmp_file.biLinePixelBytes = __bmp_file.bmpInfo.biSizeImage / __bmp_file.bmpInfo.biHeight;
    __bmp_file.biLinePixelBytes = (((__bmp_file.bmpInfo.biWidth * __bmp_file.bmpInfo.biBitCount) + 31) >> 5) << 2;
    __bmp_file.biSkipBytesPerLine = 4 - ( (__bmp_file.bmpInfo.biWidth * __bmp_file.bmpInfo.biBitCount) >> 3 ) & 3;

    __bmp_head_info_printf(&__bmp_file);

    return 0;
}

/**
 * \brief  
 */
void bmp_line_printf (uint8_t *p_file, uint32_t line, void *pVramBuf)
{
    int i = 0;
    uint8_t *p_read = p_file + __bmp_file.bmpHeader.bfOffBits;

    if (__bmp_file.bmpInfo.biHeight > 0) {
        p_read = p_read + (__bmp_file.bmpInfo.biHeight - line - 1) * __bmp_file.biLinePixelBytes;
    } else {
        p_read = p_read + (line * __bmp_file.biLinePixelBytes);
    }
    if ( __bmp_file.bmpInfo.biBitCount == 16 ) {
        uint16_t *p_read16 = (uint16_t *)p_read;

        for (i = 0; i < __bmp_file.bmpInfo.biWidth; i++) {
            ((uint16_t *)pVramBuf)[i] = p_read16[i];
        }
    }
}

/**
 * \brief  
 */
void bmp_all_printf (uint8_t *p_file, void *pVramBuf)
{
    uint16_t *p_write = (uint16_t *)pVramBuf;
    uint32_t  line = 0;
    
    if (__bmp_file.bmpInfo.biHeight < 0)
    {
        line = - (__bmp_file.bmpInfo.biHeight);
    }else{
        line =   (__bmp_file.bmpInfo.biHeight);
    }
    
    for (int i = 0; i < line; i++) {
        bmp_line_printf(p_file, i, p_write+ (__bmp_file.bmpInfo.biWidth) * i);
    }
    
}

/**
 * \brief  
 */
void bmp_close (void)
{
    memset( &__bmp_file, 0, sizeof(__bmp_file) );
}



/*******************************************************************************
 * main code region: function implement                                        
 ******************************************************************************/


/* end of file */


