#ifndef __MJPEG_H
#define __MJPEG_H 
#include "stdio.h" 
#include "stdint.h"
#include <cdjpeg.h> 
#include <setjmp.h>


/**
 * \brief mjpeg错误管理结构体
 */
struct my_error_mgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
};

typedef struct my_error_mgr * my_error_ptr;


/* mjpeg信息  */
typedef struct mjpeg_info {

    struct jpeg_decompress_struct *cinfo;
    struct my_error_mgr *jerr;

    uint8_t   *jmem_buf;          /**< \brief mjpeg解码使用的内存池 */
    uint32_t   jmempos;           /**< \brief mjpeg解码内存池分配索引 */
    uint16_t   imgoffx;           /**< \brief 图像在X轴上的偏移 */
    uint16_t   imgoffy;           /**< \brief 图像在Y轴上的偏移 */
    uint32_t   jbufsize;          /**< \brief jpeg buf大小 */
    uint8_t   *jpegbuf;           /**< \brief jpeg数据缓存指针 */

} mjpeg_info_t;


/**
 * \brief  mjpeg 解码初始化
 *
 * \param[in] p_mjpeg  mjpeg解码相关信息
 * \param[in] offx     视频在屏幕上的X轴偏移
 * \param[in] offy     视频在屏幕上的y轴偏移
 *
 * \retval     0       成功
 *            -1       失败
 */
int  mjpegdec_init(mjpeg_info_t *p_mjpeg, uint16_t offx, uint16_t offy);


/**
 * \brief  释放mjpeg解码相关的内存
 *
 * \param[in] p_mjpeg  mjpeg解码相关信息
 *
 * \return    无
 */
void mjpegdec_free(mjpeg_info_t *p_mjpeg);


/**
 * \brief  解码一副JPEG图片
 *
 * \param[in] p_mjpeg  mjpeg解码相关信息
 * \param[in] p_buf    视频数据流输入缓冲
 * \param[in] bsize    视频数据流输入缓冲大小
 *
 * \retval     0       成功
 *            -1       失败
 */
int  mjpegdec_decode(mjpeg_info_t *p_mjpeg, uint8_t *p_buf, uint32_t bsize);


#endif

/* end of file */

