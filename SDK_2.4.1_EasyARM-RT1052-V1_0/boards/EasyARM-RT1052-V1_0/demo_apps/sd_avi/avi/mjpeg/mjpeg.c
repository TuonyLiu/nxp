#include "mjpeg.h" 
#include "jpeglib.h"

#include <stdio.h>
#include <string.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_elcdif.h"

#define AVI_PRINTF  PRINTF

/** \brief mjpeg内存池大小 (38K字节) */
#define __MJPEG_MAX_MALLOC_SIZE   (38 * 1024)

/** \brief 外部全局变量，mjpeg解码相关信息 */
extern mjpeg_info_t  mjpeg_info;

/** \brief 定义全局变量，lcd显存地址 */
static uint8_t *gp_lcd_addr;

/** \brief 定义全局变量，显存切换计数标志 */
static uint8_t g_fram_cnt;


/**
 * \brief mjpeg申请内存
 */
void *mjpeg_malloc(uint32_t num)
{
    uint32_t curpos = mjpeg_info.jmempos;   /* 此次分配的起始地址 */
    mjpeg_info.jmempos += num;              /* 下一次分配的起始地址 */

    if (mjpeg_info.jmempos > __MJPEG_MAX_MALLOC_SIZE) {
        AVI_PRINTF("mem error:%d,%d", curpos, num);
    }

    /* 返回申请到的内存首地址 */
    return (void *)&mjpeg_info.jmem_buf[curpos];
}

/**
 * \brief 错误退出
 */
static void my_error_exit(j_common_ptr cinfo)
{ 
    my_error_ptr myerr = (my_error_ptr) cinfo->err;
    (*cinfo->err->output_message) (cinfo);
    longjmp(myerr->setjmp_buffer, 1);
} 


METHODDEF(void) my_emit_message(j_common_ptr cinfo, int msg_level)
{
    my_error_ptr myerr = (my_error_ptr) cinfo->err;

    if (msg_level < 0) {
        AVI_PRINTF("emit msg:%d\r\n", msg_level);
        longjmp(myerr->setjmp_buffer, 1);
    }
}


/**
 * \brief 始化资源,不执行任何操作
 */
static void init_source(j_decompress_ptr cinfo)
{
    //不需要做任何事情.
    return;
} 


/**
 * \brief 填充输入缓冲区,一次性读取整帧数据
 */
static boolean fill_input_buffer(j_decompress_ptr p_cinfo)
{  

    if (mjpeg_info.jbufsize == 0) {   /* 填充结束 */

        AVI_PRINTF("jd read off \r\n");

        /* 添加填充结束符 */
        mjpeg_info.jpegbuf[0] = (uint8_t) 0xFF;
        mjpeg_info.jpegbuf[1] = (uint8_t) JPEG_EOI;
        p_cinfo->src->next_input_byte = mjpeg_info.jpegbuf;
        p_cinfo->src->bytes_in_buffer = 2;
    } else {

        p_cinfo->src->next_input_byte = mjpeg_info.jpegbuf;
        p_cinfo->src->bytes_in_buffer = mjpeg_info.jbufsize;
        mjpeg_info.jbufsize -= mjpeg_info.jbufsize;
    }
    return TRUE;
}


/**
 * \brief 在文件里面,跳过num_bytes个数据
 */
static void skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{ 
    /* Just a dumb implementation for now.  Could use fseek() except
    * it doesn't work on pipes.  Not clear that being smart is worth
    * any trouble anyway --- large skips are infrequent.
    */
    if (num_bytes > 0)
    {
        while(num_bytes>(long) cinfo->src->bytes_in_buffer)
        {
            num_bytes-=(long)cinfo->src->bytes_in_buffer;
            (void)cinfo->src->fill_input_buffer(cinfo);
            /* note we assume that fill_input_buffer will never
            * return FALSE, so suspension need not be handled.
            */
        }
        cinfo->src->next_input_byte += (size_t) num_bytes;
        cinfo->src->bytes_in_buffer -= (size_t) num_bytes;
    }
} 


/**
 * \brief 在解码结束后,被jpeg_finish_decompress函数调用
 */
static void term_source(j_decompress_ptr cinfo)
{
    //不做任何处理
    return;
}


/**
 * \brief 初始化jpeg解码数据源
 */
static void jpeg_filerw_src_init(j_decompress_ptr cinfo)
{ 
    if (cinfo->src == NULL)     /* first time for this JPEG object? */
    {
        cinfo->src = (struct jpeg_source_mgr *)
                     (*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT,
                                              sizeof(struct jpeg_source_mgr)); 
    } 
    cinfo->src->init_source = init_source;
    cinfo->src->fill_input_buffer = fill_input_buffer;
    cinfo->src->skip_input_data = skip_input_data;
    cinfo->src->resync_to_restart = jpeg_resync_to_restart; /* use default method */
    cinfo->src->term_source = term_source;
    cinfo->src->bytes_in_buffer = 0; /* forces fill_input_buffer on first read */
    cinfo->src->next_input_byte = NULL; /* until buffer loaded */
} 

/**
 * \brief  mjpeg解码初始化
 */

uint8_t  mj_pool_buf[__MJPEG_MAX_MALLOC_SIZE];

struct my_error_mgr error_mgr;

struct jpeg_decompress_struct jpeg_decompress;



int mjpegdec_init(mjpeg_info_t *p_mjpeg, uint16_t offx, uint16_t offy)
{
#if 0
    p_mjpeg->cinfo = (struct jpeg_decompress_struct *) aw_mem_alloc(
                      sizeof(struct jpeg_decompress_struct));
    p_mjpeg->jerr = (struct my_error_mgr *) aw_mem_alloc(sizeof(struct my_error_mgr));
    p_mjpeg->jmem_buf = (uint8_t *)aw_mem_alloc(__MJPEG_MAX_MALLOC_SIZE);   /* MJPEG解码内存池申请 */
#else

    p_mjpeg->cinfo = &jpeg_decompress;

    p_mjpeg->jerr = &error_mgr;

    p_mjpeg->jmem_buf = mj_pool_buf;   /* MJPEG解码内存池申请 */

#endif
    if((p_mjpeg->cinfo == 0)    || (p_mjpeg->jerr == 0) ||
       (p_mjpeg->jmem_buf == 0)) {

        mjpegdec_free(p_mjpeg);
        return -1;
    }

    /* 设置图像在x, y方向的偏移量 */
    p_mjpeg->imgoffx = offx;
    p_mjpeg->imgoffy = offy;

    return 0;
}

/**
 * \brief  mjpeg结束,释放内存
 */
void mjpegdec_free(mjpeg_info_t *p_mjpeg)
{    
#if 0
    aw_mem_free(p_mjpeg->cinfo);
    aw_mem_free(p_mjpeg->jerr);
    aw_mem_free(p_mjpeg->jmem_buf);
#endif
}

#define APP_ELCDIF LCDIF
extern uint8_t *gp_vam_addr0;
extern uint8_t *gp_vam_addr1;
/**
 * \brief 解码一副JPEG图片
 */
int  mjpegdec_decode(mjpeg_info_t *p_mjpeg, uint8_t *p_buf, uint32_t bsize)
{
    JSAMPARRAY buffer[2];

    if ((bsize == 0) || (p_buf == NULL)) {
        return -1;
    }

    p_mjpeg->jpegbuf  = p_buf;
    p_mjpeg->jbufsize = bsize;
    p_mjpeg->jmempos  = 0;       /* MJEPG解码,重新从0开始分配内存 */
    p_mjpeg->cinfo->err=jpeg_std_error(&p_mjpeg->jerr->pub);
    p_mjpeg->jerr->pub.error_exit = my_error_exit;
    p_mjpeg->jerr->pub.emit_message = my_emit_message;

    if (setjmp(p_mjpeg->jerr->setjmp_buffer))  {  /* 错误处理 */

        jpeg_abort_decompress(p_mjpeg->cinfo);
        jpeg_destroy_decompress(p_mjpeg->cinfo);
        return -1;
    }
    jpeg_create_decompress(p_mjpeg->cinfo);
    jpeg_filerw_src_init(p_mjpeg->cinfo);
    jpeg_read_header(p_mjpeg->cinfo, TRUE);
    p_mjpeg->cinfo->dct_method = JDCT_IFAST;
    p_mjpeg->cinfo->do_fancy_upsampling = 0;
    p_mjpeg->cinfo->max_v_samp_factor = 2;

    jpeg_start_decompress(p_mjpeg->cinfo);

    /* 双缓冲切换 */
    if (g_fram_cnt == 0) {
        g_fram_cnt = 1;
        
        ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)gp_lcd_addr);        
        gp_lcd_addr = gp_vam_addr0;
        
    } else if (g_fram_cnt == 1) {
        g_fram_cnt = 0;
        ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)gp_lcd_addr);        
        gp_lcd_addr = gp_vam_addr1;
    }

    while (p_mjpeg->cinfo->output_scanline < p_mjpeg->cinfo->output_height) {

        buffer[0] = (JSAMPARRAY)(gp_lcd_addr + ((p_mjpeg->cinfo->output_scanline   +   p_mjpeg->imgoffy) * 480  + p_mjpeg->imgoffx) * 2);


        buffer[1] = (JSAMPARRAY)(gp_lcd_addr + ((p_mjpeg->cinfo->output_scanline + 1 + p_mjpeg->imgoffy) * 480  + p_mjpeg->imgoffx) * 2);

        /* 解码两行的数据 */
        jpeg_read_scanlines(p_mjpeg->cinfo, (JSAMPARRAY)buffer, 2);
    }

    /* 解码结束 */
    jpeg_finish_decompress(p_mjpeg->cinfo);

    /* 释放资源 */
    jpeg_destroy_decompress(p_mjpeg->cinfo);
    return 0;
}

/* end of file */
