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
 * \brief 
 * 
 * \internal
 * \par Modification History
 * - 1.00 16-10-13  yunfeng.zhang, first implementation.
 * \endinternal
 */


#ifndef __LIB_BMP_H
#define __LIB_BMP_H


#ifdef __cplusplus
extern "C" {
#endif
/*******************************************************************************
 * Header file
 ******************************************************************************/
#include <stdint.h>
#include <stdio.h>


#define BMP_PRINTF    PRINTF
/**
 * \addtogroup am_if_uart
 * \copydoc am_uart.h
 * @{
 */

/**
 * \name  
 * \note  
 * @{
 */
 
/** @} */
 
 
 
/** \brief   */
                            /**< \brief  */
 
 
 
/*******************************************************************************
 * Public define region: constant & MACRO defined here                              
 ******************************************************************************/

/*******************************************************************************
 * extern region: extern global variable & function prototype                 
 ******************************************************************************/

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

#ifdef _WIN32
    #pragma pack(show)                      // Output info：warning C4810: value of pragma pack(show) == 8  
    #pragma pack(push, alignmentDEfault)    // push default pack in alignmentDEfault
    #pragma pack(1)                         // set pack 1
    #pragma pack(show)  
#else

    #pragma pack(1)

#endif

/** \brief  bmp file header */
typedef struct bmp_header 
{  
    uint16_t bfType;               /* ASCII: BM */
    uint32_t bfSize; 
    uint16_t bfReserved0; 
    uint16_t bfReserved1; 
    uint32_t bfOffBits;
} bmp_header_t; 

/** \brief  bmp file info */
typedef struct bmp_info 
{  
    uint32_t biSize; 
    uint32_t biWidth;
    int32_t  biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    uint32_t biXPelsPerMeter;
    uint32_t biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;

} bmp_info_t; 

/** \brief  bmp color table struct */
typedef union bmp_clr_table {
    uint32_t brgcolor;
    struct {
        uint8_t rgbBlue;
        uint8_t rgbGreen;
        uint8_t rgbRed;
        uint8_t rgbReserved;    
    };
} bmp_clr_table_t;

/** \brief  bmp file struct */
typedef struct bmp_file {

    bmp_header_t bmpHeader;
    bmp_info_t   bmpInfo;
    uint32_t     biLinePixelBytes;
    uint32_t     biSkipBytesPerLine;    
    bmp_clr_table_t *pbiClrTable;
    uint32_t     biClrTableCount;


} bmp_file_t;


#ifdef _WIN32
    #pragma pack(pop, alignmentDEfault) //recovery pack
    #pragma pack(show)  
#endif

/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #pragma pop
#elif defined(__CWCC__)
  #pragma pop
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif
/*****************************************************************************
 * Private functions
 ****************************************************************************/


/*****************************************************************************
 * Public functions
 ****************************************************************************/
 
/**
 * \brief 
 *
 * \param[in] handle         :  
 *
 * \retval  AM_OK      :  
 * \retval -AM_EINVAL  : 
 * \note  
 *        
 */
extern int bmp_open (uint8_t *p_bmp_file);
/**
 * \brief 
 *
 * \param[in] handle  :  
 *
 * \return  
 */
extern void bmp_line_printf (uint8_t *p_file, uint32_t line, void *pVramBuf);

extern void bmp_all_printf (uint8_t *p_file, void *pVramBuf);
/**
 * @}
 */
 
 #ifdef __cplusplus
}
#endif

#endif /* __LIB_BMP_H */

/* end of file */


