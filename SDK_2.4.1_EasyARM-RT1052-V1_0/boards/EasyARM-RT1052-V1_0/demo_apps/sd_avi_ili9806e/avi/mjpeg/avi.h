#ifndef __AVI_H
#define __AVI_H 

#include "stdint.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"

#define AVI_PRINTF  PRINTF

/**
 * \brief AVI视频文件解析返回值
 */
typedef enum {
    AVI_OK = 0,           /**< \brief 成功 */
    AVI_RIFF_ERR,         /**< \brief RIFF ID读取失败 */
    AVI_AVI_ERR,          /**< \brief AVI ID读取失败 */
    AVI_LIST_ERR,         /**< \brief LIST ID读取失败 */
    AVI_HDRL_ERR,         /**< \brief HDRL ID读取失败 */
    AVI_AVIH_ERR,         /**< \brief AVIH ID读取失败 */
    AVI_STRL_ERR,         /**< \brief STRL ID读取失败 */
    AVI_STRH_ERR,         /**< \brief STRH ID读取失败 */
    AVI_STRF_ERR,         /**< \brief STRF ID读取失败 */
    AVI_MOVI_ERR,         /**< \brief MOVI ID读取失败 */
    AVI_FORMAT_ERR,       /**< \brief 格式错误 */
    AVI_STREAM_ERR,       /**< \brief 流错误 */
} avi_status_t;


#define AVI_RIFF_ID         0X46464952
#define AVI_AVI_ID          0X20495641
#define AVI_LIST_ID         0X5453494C
#define AVI_HDRL_ID         0X6C726468      /**< \brief 信息块标志 */
#define AVI_MOVI_ID         0X69766F6D      /**< \brief 数据块标志 */
#define AVI_STRL_ID         0X6C727473      /**< \brief strl标志 */
#define AVI_AVIH_ID         0X68697661      /**< \brief avih子块∈AVI_HDRL_ID */
#define AVI_STRH_ID         0X68727473      /**< \brief strh(流头)子块∈AVI_STRL_ID */
#define AVI_STRF_ID         0X66727473      /**< \brief strf(流格式)子块∈AVI_STRL_ID */
#define AVI_STRD_ID         0X64727473      /**< \brief strd子块∈AVI_STRL_ID (可选的) */
#define AVI_VIDS_STREAM     0X73646976      /**< \brief 视频流 */
#define AVI_AUDS_STREAM     0X73647561      /**< \brief 音频流 */
#define AVI_VIDS_FLAG       0X6463          /**< \brief 视频流标志 */
#define AVI_AUDS_FLAG       0X7762          /**< \brief 音频流标志 */
#define AVI_FORMAT_MJPG     0X47504A4D

#pragma pack(push)
#pragma pack(1)


/**
 * \brief AVI信息结构体
 */
typedef struct{

    uint32_t sec_per_frame;      /**< \brief 视频帧间隔时间(单位为us) */
    uint32_t total_frame;        /**< \brief 文件总帧数 */
    uint32_t width;              /**< \brief 图像宽 */
    uint32_t height;             /**< \brief 图像高 */
    uint32_t sample_rate;        /**< \brief 音频采样率 */
    uint16_t channels;           /**< \brief 声道数,一般为2,表示立体声 */
    uint16_t audio_buf_size;     /**< \brief 音频缓冲区大小 */
    uint16_t audio_type;         /**< \brief 音频类型:0X0001=PCM;0X0050=MP2;0X0055=MP3;0X2000=AC3; */
    uint16_t stream_id;          /**< \brief 流类型ID,StreamID=='dc'==0X6463 /StreamID=='wb'==0X7762 */
    uint32_t stream_size;        /**< \brief 流大小,必须是偶数,如果读取到为奇数,则加1.补为偶数. */
    uint8_t* video_flag;         /**< \brief 视频帧标记,VideoFLAG="00dc"/"01dc" */
    uint8_t* audio_flag;         /**< \brief 音频帧标记,AudioFLAG="00wb"/"01wb" */
} avi_info_t;

#pragma pack(pop)


/**
 * \brief AVI块信息
 */
typedef struct {
    uint32_t riff_id;            /**< \brief RiffID=='RIFF'==0X61766968 */
    uint32_t file_size;          /**< \brief AVI文件大小(不包含最初的8字节,也RIFFID和FileSize不计算在内) */
    uint32_t avi_id;             /**< \brief AviID=='AVI '==0X41564920 */
} avi_header_t;


/**
 * \brief FRAME块信息
 */
typedef struct {
    uint32_t frame_id;           /**< \brief 帧ID,FrameID=='RIFF'==0X61766968 */
    uint32_t frame_size;         /**< \brief 帧大小 */
} frame_header_t;



/**
 * \brief LIST块信息
 */
typedef struct {
    uint32_t list_id;             /**< \brief ListID=='LIST'==0X4c495354 */
    uint32_t block_size;          /**< \brief 块大小(不包含最初的8字节,也ListID和BlockSize不计算在内) */
    uint32_t list_type;           /**< \brief LIST子块类型:hdrl(信息块)/movi(数据块)/idxl(索引块,非必须,是可选的) */
} list_header_t;

/**< \brief avih 子块信息 */
typedef struct {
    uint32_t block_id;            /**< \brief 块标志:avih==0X61766968 */
    uint32_t block_size;          /**< \brief 块大小(不包含最初的8字节,也就是BlockID和BlockSize不计算在内) */
    uint32_t sec_per_frame;       /**< \brief 视频帧间隔时间(单位为us) */
    uint32_t max_byte_sec;        /**< \brief 最大数据传输率,字节/秒 */
    uint32_t padding_granularity; /**< \brief 数据填充的粒度 */
    uint32_t flags;               /**< \brief AVI文件的全局标记，比如是否含有索引块等 */
    uint32_t total_frame;         /**< \brief 文件总帧数 */
    uint32_t init_frames;         /**< \brief 为交互格式指定初始帧数（非交互格式应该指定为0） */
    uint32_t streams;             /**< \brief 包含的数据流种类个数,通常为2 */
    uint32_t ref_buf_size;        /**< \brief 建议读取本文件的缓存大小（应能容纳最大的块）默认可能是1M字节!!! */
    uint32_t width;               /**< \brief 图像宽 */
    uint32_t height;              /**< \brief 图像高 */
    uint32_t reserved[4];         /**< \brief 保留 */
} avih_header_t;

/**< \brief strh 流头子块信息(strh∈strl) */
typedef struct {
    uint32_t block_id;            /**< \brief 块标志:strh==0X73747268 */
    uint32_t block_size;          /**< \brief 块大小(不包含最初的8字节,也就是BlockID和BlockSize不计算在内) */
    uint32_t stream_type;         /**< \brief 数据流种类，vids(0X73646976):视频;auds(0X73647561):音频 */
    uint32_t handler;             /**< \brief 指定流的处理者，对于音视频来说就是解码器,比如MJPG/H264之类的. */
    uint32_t flags;               /**< \brief 标记：是否允许这个流输出？调色板是否变化？ */
    uint16_t priority;            /**< \brief 流的优先级（当有多个相同类型的流时优先级最高的为默认流） */
    uint16_t language;            /**< \brief 音频的语言代号 */
    uint32_t init_frames;         /**< \brief 为交互格式指定初始帧数 */
    uint32_t scale;               /**< \brief 数据量, 视频每桢的大小或者音频的采样大小 */
    uint32_t rate;                /**< \brief Scale/Rate=每秒采样数 */
    uint32_t start;               /**< \brief 数据流开始播放的位置，单位为Scale */
    uint32_t length;              /**< \brief 数据流的数据量，单位为Scale */
    uint32_t ref_buf_size;        /**< \brief 建议使用的缓冲区大小 */
    uint32_t quality;             /**< \brief 解压缩质量参数，值越大，质量越好 */
    uint32_t sampleSize;          /**< \brief 音频的样本大小 */
    struct {                      /**< \brief 视频帧所占的矩形 */

        short left;
        short top;
        short right;
        short bottom;
    } frame;
} strh_header_t;


/**
 * \brief BMP结构体
 */
typedef struct {
    uint32_t  bmp_size;           /**< \brief bmp结构体大小,包含(BmpSize在内) */
    long      width;              /**< \brief 图像宽 */
    long      height;             /**< \brief 图像高 */
    uint16_t  planes;             /**< \brief 平面数，必须为1 */
    uint16_t  bit_count;          /**< \brief 像素位数,0X0018表示24位 */
    uint32_t  compression;        /**< \brief 压缩类型，比如:MJPG/H264等 */
    uint32_t  size_image;         /**< \brief 图像大小 */
    long      xpix_per_meter;     /**< \brief 水平分辨率 */
    long      ypix_per_meter;     /**< \brief 垂直分辨率 */
    uint32_t  clr_used;           /**< \brief 实际使用了调色板中的颜色数,压缩格式中不使用 */
    uint32_t  clr_important;      /**< \brief 重要的颜色 */
} bmp_header_t;


/**
 * \brief 颜色表
 */
typedef struct {
    uint8_t  rgb_blue;            /**< \brief 蓝色的亮度(值范围为0-255) */
    uint8_t  rgb_green;           /**< \brief 绿色的亮度(值范围为0-255) */
    uint8_t  rgb_red;             /**< \brief 红色的亮度(值范围为0-255) */
    uint8_t  rgb_reserved;        /**< \brief 保留，必须为0 */
} avi_rgb_quad_t;


/**
 * \brief 对于strh,如果是视频流,strf(流格式)使STRH_BMPHEADER块
 */
typedef struct  {
    uint32_t block_id;              /**< \brief 块标志,strf==0X73747266 */
    uint32_t block_size;            /**< \brief 块大小(不包含最初的8字节,也就是BlockID和本BlockSize不计算在内) */
    bmp_header_t   bmp_header;      /**< \brief 位图信息头 */
    avi_rgb_quad_t bmp_colors[1];   /**< \brief 颜色表 */
} strf_bmp_header_t;


/**
 * \brief 对于strh,如果是音频流,strf(流格式)使STRH_WAVHEADER块
 */
typedef struct 
{
    uint32_t block_id;             /**< \brief 块标志,strf==0X73747266 */
    uint32_t block_size;           /**< \brief 块大小(不包含最初的8字节,也就是BlockID和本BlockSize不计算在内) */
    uint16_t format_tag;           /**< \brief 格式标志:0X0001=PCM,0X0055=MP3... */
    uint16_t channels;             /**< \brief 声道数,一般为2,表示立体声 */
    uint32_t sample_rate;          /**< \brief 音频采样率 */
    uint32_t baud_rate;            /**< \brief 波特率 */
    uint16_t block_align;          /**< \brief 数据块对齐标志 */
    uint16_t size;                 /**< \brief 该结构大小 */
} strf_wav_header_t;


/**
 * \brief  AVI视频解码初始化
 * \param[in] p_buf  输入缓冲区
 * \param[in] size   缓冲区的大小
 *
 * \return avi_status_t成员
 *
 */
avi_status_t avi_init(uint8_t *p_buf, uint16_t size);        /**< \brief 初始化avi解码器 */


/**
 * \brief 查找ID (要查找的ID必须是4字节长度)
 *
 * \param[in] p_buf  待查缓存区
 * \param[in] size   缓冲区的大小
 *
 * \retval  0   查找失败
 *          >0  ID的偏移量
 */
uint16_t avi_srarch_id (uint8_t *p_buf, uint16_t size, const char *id);


/**
 * \brief 得到stream流信息
 *
 * \param[in] p_buf 流开始的地址
 *
 * \return  avi_status_t成员
 *
 */
avi_status_t avi_get_streaminfo (uint8_t *p_buf);


extern avi_info_t avix;       /**< \brief avi文件相关信息 */


#endif

