
#include <stdio.h>
#include <string.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"

#include "avi.h"
#include "mjpeg.h"

#include "ff.h"
#include "diskio.h"

/** \brief 定义全局mjpeg解码信息结构体 */
mjpeg_info_t  mjpeg_info;

/** \brief 定义音频、视频buf的大小 */
#define AVI_AUDIO_BUF_SIZE    1024 * 5
#define AVI_VIDEO_BUF_SIZE    1024 * 128


/******************************************************************************/

uint8_t p_mj_buf[AVI_VIDEO_BUF_SIZE];


int video_play_mjpeg(uint8_t *pname)
{
    FIL aviFil;
    int fd      = 0;
    int speed   = 0;
    int ret = 0;

    uint8_t *framebuf;
    uint8_t *pbuf;

    uint16_t offset=0;
    uint32_t read_len;
    uint32_t tick1, tick2;
    uint16_t frame;         /* 帧计数 */
    uint32_t t1, t2;

    /* 申请视频缓冲区内存 */

  //  framebuf = (uint8_t *)aw_mem_alloc(AVI_VIDEO_BUF_SIZE);
    framebuf = p_mj_buf;

    if (framebuf == NULL) {
        AVI_PRINTF("Memory error!\r\n");
        ret  = -1;
        goto clean_up;
    }

    //fd = aw_open((char *)pname, O_RDWR, 0777);
    // Open file to check
    fd = f_open(&aviFil, (char *)pname, FA_READ);
    if (fd != FR_OK)
    {
        AVI_PRINTF("No %s file!\r\n",(char *)pname);
        return -4;
    } else 
    //if (fd >= 0) 
    {
        pbuf = framebuf;

        //read_len = aw_read(fd, pbuf, AVI_VIDEO_BUF_SIZE); //开始读取
        f_read(&aviFil, pbuf, AVI_VIDEO_BUF_SIZE, &read_len);

        if(read_len == 0) {
            AVI_PRINTF("File read failed !\r\n");
            f_close(&aviFil);
            goto clean_up;
        }
//        ret = mjpegdec_init(&mjpeg_info, 0, 0);
//        ret = mjpegdec_decode(&mjpeg_info, pbuf, AVI_VIDEO_BUF_SIZE);
        
        /* 开始avi文件头解析 */
        ret = avi_init(pbuf, AVI_VIDEO_BUF_SIZE);
        if (ret != 0) {
            AVI_PRINTF("AVI video file header parsing failed !\r\n");
            f_close(&aviFil);
            goto clean_up;
        }

        offset = avi_srarch_id(pbuf, AVI_VIDEO_BUF_SIZE, "movi"); /* 寻找movi ID */
        avi_get_streaminfo(pbuf + offset + 4);     /* 获取流信息 */
        f_lseek(&aviFil, offset + 12);       /* 跳过标志ID,读地址偏移到流数据开始处 */

        /* JPG解码初始化, 并设置图像的起始坐标为(0， 0) */
        ret = mjpegdec_init(&mjpeg_info, 0, 0);

        /* 如果有音频信息,可初始化音频相关设备 */
        if(avix.sample_rate) {
            //在此可对音频设备进行初始化...

        }
        while (1) {  /* 播放循环 */

            if (avix.stream_id == AVI_VIDS_FLAG)  {  /* 如果是视频流 */

                pbuf = framebuf;

                /* 读入整帧+下一数据流ID信息 */
                //read_len = aw_read(fd, pbuf, avix.stream_size + 8);
                f_read(&aviFil, pbuf, avix.stream_size + 8, &read_len);

//                t1 = aw_sys_tick_get();

                ret = mjpegdec_decode(&mjpeg_info, pbuf, avix.stream_size);
                if (ret == -1) {
                    AVI_PRINTF("Video decoding failed !\r\n");
                }
#if 0
                t2 = aw_sys_tick_get();
                //AVI_PRINTF("The time for decoding a frame is:  %dms \n", t2 - t1);
                AVI_PRINTF("dtime:  %dms \n", t2 - t1);

                if (frame % 10 == 0) {
                    tick1 = aw_sys_tick_get();
                    speed = (10 * 1000) / (tick1 - tick2);
                   // AVI_PRINTF("Video decoding frame rate is: %d \r\n", speed);
                    AVI_PRINTF("f_rate: %d \r\n", speed);
                }
#endif
                tick2 =  tick1;
                frame++;   /* 视频帧计数 */

                /* 得到stream流信息 */
                if(avi_get_streaminfo(pbuf + avix.stream_size)) {
                    AVI_PRINTF("Failed to get stream information !\r\n");
                    break;
                }
            } else {  /* 如果是音频流 */

                //aw_lseek(fd, avix.stream_size, SEEK_CUR);
                f_lseek(&aviFil, avix.stream_size + aviFil.fptr);

                /* 读入整帧+下一数据流ID信息 */
                //read_len = aw_read(fd, pbuf, 8);
                f_read(&aviFil, pbuf, 8, &read_len);

                if (avi_get_streaminfo(pbuf)) {
                    AVI_PRINTF("Failed to get stream information !\r\n");
                    break;
                }
            }
        }

        /* 释放内存 */
        mjpegdec_free(&mjpeg_info);
        f_close(&aviFil);

    }
clean_up:

   // aw_mem_free(framebuf);

    return ret;
}


#if 0
/**
 * \brief 测试视频播放，只适合MJPEG编码格式AVI视频
 */
int  test_avi (void)
{
    int ret = 0;
    uint8_t  rty = 255;

    fb_init();

    /* 检测SD卡是否插入 */
    AW_INFOF(("wait for sdcard insert...\n"));
    while(rty-- > 0) {
        if (!aw_sdcard_is_insert("/dev/sd0")) {
           aw_mdelay(1000);
        } else {
           break;
        }
    }

    if (rty <= 0 ) {
        AW_ERRF(("no sdcard found.\n"));
        return -1;
    }

    /* 挂载 */
    ret = aw_mount( "/sd", "/dev/sd0", "vfat", 0 );
    if (ret != 0) {
        while(1);
    }

    while(1) {
        video_play_mjpeg("/sd/kongfu.avi");
        video_play_mjpeg("/sd/zhou.avi");
        video_play_mjpeg("/sd/tian.avi");
        video_play_mjpeg("/sd/hai.avi");
    }

    /* 卸载 */
    ret = aw_umount( "/dev/sd0", 0 );

    return ret;
}

#endif

/* end of file */

