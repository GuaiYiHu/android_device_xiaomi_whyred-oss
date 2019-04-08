/* Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#define LOG_TAG "QCamera3PostProc"

// To remove
#include <cutils/properties.h>

// System dependencies
#include <stdio.h>

// Camera dependencies
#include "QCamera3Channel.h"
#include "QCamera3HWI.h"
#include "QCamera3PostProc.h"
#include "QCamera3Stream.h"
#include "QCameraTrace.h"
#include "QCameraPprocManager.h"
#include "QCameraMem.h"

extern "C" {
#include "mm_camera_dbg.h"
}

#define ENABLE_MODEL_INFO_EXIF

namespace qcamera {

static const char ExifAsciiPrefix[] =
    { 0x41, 0x53, 0x43, 0x49, 0x49, 0x0, 0x0, 0x0 };          // "ASCII\0\0\0"

__unused
static const char ExifUndefinedPrefix[] =
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };   // "\0\0\0\0\0\0\0\0"

#define EXIF_ASCII_PREFIX_SIZE           8   //(sizeof(ExifAsciiPrefix))
#define FOCAL_LENGTH_DECIMAL_PRECISION   1000

/*===========================================================================
 * FUNCTION   : QCamera3PostProcessor
 *
 * DESCRIPTION: constructor of QCamera3PostProcessor.
 *
 * PARAMETERS :
 *   @cam_ctrl : ptr to HWI object
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3PostProcessor::QCamera3PostProcessor(QCamera3ProcessingChannel* ch_ctrl)
    : m_parent(ch_ctrl),
      mJpegCB(NULL),
      mJpegUserData(NULL),
      mJpegClientHandle(0),
      mJpegSessionId(0),
      m_bThumbnailNeeded(TRUE),
      m_ppChannelCnt(1),
      m_inputPPQ(releasePPInputData, this),
      m_inputFWKPPQ(NULL, this),
      m_inputMultiReprocQ(NULL, this),  // add release job data func here
      m_ongoingPPQ(releaseOngoingPPData, this),
      m_inputJpegQ(releaseJpegData, this),
      m_ongoingJpegQ(releaseJpegData, this),
      m_inputMetaQ(releaseMetadata, this),
      m_jpegSettingsQ(NULL, this),
      m_pHalPPManager(NULL),
      mChannelStop(TRUE)
{
    memset(&mJpegHandle, 0, sizeof(mJpegHandle));
    memset(&mJpegMetadata, 0, sizeof(mJpegMetadata));
    memset(m_pReprocChannel, 0, sizeof(m_pReprocChannel));
    mReprocessNode.clear();
    pthread_mutex_init(&mReprocJobLock, NULL);
    pthread_mutex_init(&mHDRJobLock, NULL);
    pthread_cond_init(&mProcChStopCond, NULL);
}

/*===========================================================================
 * FUNCTION   : ~QCamera3PostProcessor
 *
 * DESCRIPTION: deconstructor of QCamera3PostProcessor.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3PostProcessor::~QCamera3PostProcessor()
{
    for (int8_t i = 0; i < m_ppChannelCnt; i++) {
        QCamera3Channel *pChannel = m_pReprocChannel[i];
        if (pChannel != NULL ) {
            pChannel->stop();
            delete pChannel;
            m_pReprocChannel[i] = NULL;
        }
    }
    if (m_pHalPPManager != NULL) {
        m_pHalPPManager->release();
        m_pHalPPManager = NULL;
    }
    m_ppChannelCnt = 0;

    pthread_mutex_destroy(&mReprocJobLock);
    pthread_mutex_destroy(&mHDRJobLock);
    pthread_cond_destroy(&mProcChStopCond);
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of postprocessor
 *
 * PARAMETERS :
 *   @memory              : output buffer memory
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::init(QCamera3StreamMem *memory)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PPROC_INIT);
    mOutputMem = memory;
    m_dataProcTh.launch(dataProcessRoutine, this);
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    if (hal_obj->isDualCamera()) {
        LOGH("Check and create HAL PP manager if not present");
        createHalPPManager();
    }
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : deinit
 *
 * DESCRIPTION: de-initialization of postprocessor
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::deinit()
{
    int rc = NO_ERROR;
    m_dataProcTh.exit();

    for (int8_t i = 0; i < m_ppChannelCnt; i++) {
        QCamera3Channel *pChannel = m_pReprocChannel[i];
        if (pChannel != NULL ) {
            pChannel->stop();
            delete pChannel;
            m_pReprocChannel[i] = NULL;
        }
    }
    if (m_pHalPPManager != NULL) {
        LOGH("DeInit PP Manager");
        m_pHalPPManager->deinit();
        m_pHalPPManager = NULL;
    }
    m_ppChannelCnt = 0;

    if(mJpegClientHandle > 0) {
        rc = mJpegHandle.close(mJpegClientHandle);
        LOGH("Jpeg closed, rc = %d, mJpegClientHandle = %x",
               rc, mJpegClientHandle);
        mJpegClientHandle = 0;
        memset(&mJpegHandle, 0, sizeof(mJpegHandle));
    }

    mOutputMem = NULL;
    return rc;
}

/*===========================================================================
 * FUNCTION   : initJpeg
 *
 * DESCRIPTION: initialization of jpeg through postprocessor
 *
 * PARAMETERS :
 *   @jpeg_cb      : callback to handle jpeg event from mm-camera-interface
 *   @max_pic_dim  : max picture dimensions
 *   @user_data    : user data ptr for jpeg callback
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::initJpeg(jpeg_encode_callback_t jpeg_cb,
                                            mpo_encode_callback_t mpo_cb,
                                            cam_dimension_t* max_pic_dim,
                                            void *user_data)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PPROC_INIT_JPEG);
    mJpegCB = jpeg_cb;
    mMpoCB = mpo_cb;
    mJpegUserData = user_data;
    mm_dimension max_size;

    mMpoInputData.clear();

    if ((0 > max_pic_dim->width) || (0 > max_pic_dim->height)) {
        LOGE("Negative dimension %dx%d",
                max_pic_dim->width, max_pic_dim->height);
        return BAD_VALUE;
    }

    // set max pic size
    memset(&max_size, 0, sizeof(mm_dimension));
    max_size.w =  max_pic_dim->width;
    max_size.h =  max_pic_dim->height;

    // Pass OTP calibration data to JPEG
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    mJpegMetadata.default_sensor_flip = FLIP_NONE;
    mJpegMetadata.sensor_mount_angle = hal_obj->getSensorMountAngle();
    memcpy(&mJpegMetadata.otp_calibration_data,
            hal_obj->getRelatedCalibrationData(),
            sizeof(cam_related_system_calibration_data_t));
    mJpegClientHandle = jpeg_open(&mJpegHandle, &mMpoHandle, max_size, &mJpegMetadata);

    if (!mJpegClientHandle) {
        LOGE("jpeg_open did not work");
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: start postprocessor. Data process thread and data notify thread
 *              will be launched.
 *
 * PARAMETERS :
 *   @config        : reprocess configuration
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : if any reprocess is needed, a reprocess channel/stream
 *              will be started.
 *==========================================================================*/
int32_t QCamera3PostProcessor::start(const reprocess_config_t &config)
{
    int32_t rc = NO_ERROR;
    pthread_mutex_lock(&mHDRJobLock);
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    if(mChannelStop == false)
        pthread_cond_wait(&mProcChStopCond, &mHDRJobLock);
    pthread_mutex_unlock(&mHDRJobLock);
    if (config.reprocess_type != REPROCESS_TYPE_NONE) {
        for (int8_t i = 0; i < m_ppChannelCnt; i++) {
            QCamera3Channel *pChannel = m_pReprocChannel[i];
            if (pChannel != NULL ) {
                pChannel->stop();
                delete pChannel;
                m_pReprocChannel[i] = NULL;
            }
        }
        m_ppChannelCnt = 0;

        m_ppChannelCnt = hal_obj->getReprocChannelCnt();
        LOGH("m_ppChannelCnt:%d", m_ppChannelCnt);

        reprocess_config_t local_cfg = config;
        for (int8_t i = 0; i < m_ppChannelCnt; i++) {
            LOGD("src channel:%p, input channel:%p", local_cfg.src_channel, m_parent);
            m_pReprocChannel[i] = hal_obj->addOfflineReprocChannel(local_cfg, m_parent, i);
            if (m_pReprocChannel[i] == NULL) {
                LOGE("cannot add reprocess channel, idx:%d", i);
                return UNKNOWN_ERROR;
            }
            local_cfg.src_channel = (QCamera3Channel *)m_pReprocChannel[i];
        }

        /*start the reprocess channel only if buffers are already allocated, thus
          only start it in an intermediate reprocess type, defer it for others*/
        if (config.reprocess_type == REPROCESS_TYPE_JPEG) {
            for (int8_t i = 0; i < m_ppChannelCnt; i++) {
                rc = m_pReprocChannel[i]->start();
                if (rc != 0) {
                    LOGE("cannot start reprocess channel, idx:%d", i);
                    delete m_pReprocChannel[i];
                    m_pReprocChannel[i] = NULL;
                    return rc;
                }
            }
        }

        if (hal_obj->isDualCamera() &&
        (hal_obj->getHalPPType() != CAM_HAL_PP_TYPE_NONE)) {
            // HALPP type might have changed, ensure we have right pp block
            rc = initHalPPManager();
            if (rc != NO_ERROR) {
                LOGE("Initializing PP manager failed");
                return rc;
            }
            if (m_pHalPPManager != NULL) {
                LOGH("HALPP is need, call QCameraHALPPManager::start() here");
                rc = m_pHalPPManager->start();
                if (rc != NO_ERROR) {
                    LOGE("start PP manager failed");
                    return rc;
                }
            }
        }
    }
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_START_DATA_PROC, TRUE, FALSE);
    return rc;
}

/*===========================================================================
 * FUNCTION   : flush
 *
 * DESCRIPTION: stop ongoing postprocess and jpeg jobs
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 *==========================================================================*/
int32_t QCamera3PostProcessor::flush()
{
    int32_t rc = NO_ERROR;
    qcamera_hal3_jpeg_data_t *jpeg_job =
            (qcamera_hal3_jpeg_data_t *)m_ongoingJpegQ.dequeue();
    while (jpeg_job != NULL) {
        rc = mJpegHandle.abort_job(jpeg_job->jobId);
        releaseJpegJobData(jpeg_job);
        free(jpeg_job);

        jpeg_job = (qcamera_hal3_jpeg_data_t *)m_ongoingJpegQ.dequeue();
    }
    rc = releaseOfflineBuffers(true);
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop postprocessor. Data process and notify thread will be stopped.
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : reprocess channel will be stopped and deleted if there is any
 *==========================================================================*/
int32_t QCamera3PostProcessor::stop(bool isHDR)
{
    if ((m_pHalPPManager != NULL)) {
        m_pHalPPManager->stop();
    }

    if(isHDR == true)
        m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC, FALSE, TRUE);
    else
        m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_STOP_DATA_PROC, TRUE, TRUE);

    for (int8_t i = 0; i < m_ppChannelCnt; i++) {
        QCamera3Channel *pChannel = m_pReprocChannel[i];
        if (pChannel != NULL ) {
            pChannel->stop();
            delete pChannel;
            m_pReprocChannel[i] = NULL;
        }
    }
    m_ppChannelCnt = 0;

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getFWKJpegEncodeConfig
 *
 * DESCRIPTION: function to prepare encoding job information
 *
 * PARAMETERS :
 *   @encode_parm   : param to be filled with encoding configuration
 *   @frame         : framework input buffer
 *   @jpeg_settings : jpeg settings to be applied for encoding
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::getFWKJpegEncodeConfig(
        mm_jpeg_encode_params_t& encode_parm,
        qcamera_fwk_input_pp_data_t *frame,
        jpeg_settings_t *jpeg_settings)
{
    LOGD("E");
    int32_t ret = NO_ERROR;

    if ((NULL == frame) || (NULL == jpeg_settings)) {
        return BAD_VALUE;
    }

    ssize_t bufSize = mOutputMem->getSize(jpeg_settings->out_buf_index);
    if (BAD_INDEX == bufSize) {
        LOGE("cannot retrieve buffer size for buffer %u",
                jpeg_settings->out_buf_index);
        return BAD_VALUE;
    }

    encode_parm.jpeg_cb = mJpegCB;
    encode_parm.userdata = mJpegUserData;

    if (jpeg_settings->thumbnail_size.width > 0 &&
            jpeg_settings->thumbnail_size.height > 0)
        m_bThumbnailNeeded = TRUE;
    else
        m_bThumbnailNeeded = FALSE;
    encode_parm.encode_thumbnail = m_bThumbnailNeeded;

    // get color format
    cam_format_t img_fmt = frame->reproc_config.stream_format;
    encode_parm.color_format = getColorfmtFromImgFmt(img_fmt);

    // get jpeg quality
    encode_parm.quality = jpeg_settings->jpeg_quality;
    if (encode_parm.quality <= 0) {
        encode_parm.quality = 85;
    }

    // get jpeg thumbnail quality
    encode_parm.thumb_quality = jpeg_settings->jpeg_thumb_quality;

    cam_frame_len_offset_t main_offset =
            frame->reproc_config.input_stream_plane_info.plane_info;

    encode_parm.num_src_bufs = 1;
    encode_parm.src_main_buf[0].index = 0;
    encode_parm.src_main_buf[0].buf_size = frame->input_buffer.frame_len;
    encode_parm.src_main_buf[0].buf_vaddr = (uint8_t *) frame->input_buffer.buffer;
    encode_parm.src_main_buf[0].fd = frame->input_buffer.fd;
    encode_parm.src_main_buf[0].format = MM_JPEG_FMT_YUV;
    encode_parm.src_main_buf[0].offset = main_offset;

    //Pass input thumbnail buffer info to encoder.
    //Note: Use main buffer to encode thumbnail
    if (m_bThumbnailNeeded == TRUE) {
        encode_parm.num_tmb_bufs = 1;
        encode_parm.src_thumb_buf[0] = encode_parm.src_main_buf[0];
    }

    //Pass output jpeg buffer info to encoder.
    //mOutputMem is allocated by framework.
    encode_parm.num_dst_bufs = 1;
    encode_parm.dest_buf[0].index = 0;
    encode_parm.dest_buf[0].buf_size = (size_t)bufSize;
    encode_parm.dest_buf[0].buf_vaddr = (uint8_t *)mOutputMem->getPtr(
            jpeg_settings->out_buf_index);
    encode_parm.dest_buf[0].fd = mOutputMem->getFd(
            jpeg_settings->out_buf_index);
    encode_parm.dest_buf[0].format = MM_JPEG_FMT_YUV;
    encode_parm.dest_buf[0].offset = main_offset;

    LOGD("X");
    return NO_ERROR;

    LOGD("X with error %d", ret);
    return ret;
}

/*===========================================================================
 * FUNCTION   : getJpegEncodeConfig
 *
 * DESCRIPTION: function to prepare encoding job information
 *
 * PARAMETERS :
 *   @encode_parm   : param to be filled with encoding configuration
 *   #main_stream   : stream object where the input buffer comes from
 *   @jpeg_settings : jpeg settings to be applied for encoding
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::getJpegEncodeConfig(
                mm_jpeg_encode_params_t& encode_parm,
                QCamera3Stream *main_stream,
                jpeg_settings_t *jpeg_settings,
                mm_camera_buf_def_t *input_buf)
{
    LOGD("E");
    int32_t ret = NO_ERROR;
    ssize_t bufSize = 0;
    QCamera3StreamMem *outputMem = mOutputMem;

    if(jpeg_settings->encode_type == MM_JPEG_TYPE_JPEG)
    {
        encode_parm.jpeg_cb = mJpegCB;
        encode_parm.userdata = mJpegUserData;
    } else {
        // MPO usecase.
        encode_parm.jpeg_cb = processJpegData;
        encode_parm.userdata = this;
        outputMem = &m_parent->getJpegMemory();
    }

    if (jpeg_settings->thumbnail_size.width > 0 &&
            jpeg_settings->thumbnail_size.height > 0)
        m_bThumbnailNeeded = TRUE;
    else
        m_bThumbnailNeeded = FALSE;
    encode_parm.encode_thumbnail = m_bThumbnailNeeded;

    // get color format
    cam_format_t img_fmt = CAM_FORMAT_YUV_420_NV12;  //default value
    if(jpeg_settings->is_format_valid)
    {
        img_fmt = jpeg_settings->format;
    } else {
        if(main_stream->getFormat(img_fmt) < 0)
        {
            LOGE("Error: failed to get image format");
        }
    }
    encode_parm.color_format = getColorfmtFromImgFmt(img_fmt);

    // get jpeg quality
    encode_parm.quality = jpeg_settings->jpeg_quality;
    if (encode_parm.quality <= 0) {
        encode_parm.quality = 85;
    }

    // get jpeg thumbnail quality
    encode_parm.thumb_quality = jpeg_settings->jpeg_thumb_quality;

    cam_frame_len_offset_t main_offset;
    memset(&main_offset, 0, sizeof(cam_frame_len_offset_t));
    if(jpeg_settings->is_offset_valid)
    {
        main_offset = jpeg_settings->offset;
    } else {
        main_stream->getFrameOffset(main_offset);
    }

    // src buf config
    //Pass input main image buffer info to encoder.
    QCamera3StreamMem *pStreamMem = main_stream->getStreamBufs();
    if (pStreamMem == NULL) {
        LOGE("cannot get stream bufs from main stream");
        ret = BAD_VALUE;
        goto on_error;
    }
    if(input_buf == NULL)
    {
        encode_parm.num_src_bufs = MIN(pStreamMem->getCnt(), MM_JPEG_MAX_BUF);
        for (uint32_t i = 0; i < encode_parm.num_src_bufs; i++) {
            if (pStreamMem != NULL) {
                encode_parm.src_main_buf[i].index = i;
                bufSize = pStreamMem->getSize(i);
                if (BAD_INDEX == bufSize) {
                    LOGE("cannot retrieve buffer size for buffer %u", i);
                    ret = BAD_VALUE;
                    goto on_error;
                }
                encode_parm.src_main_buf[i].buf_size = (size_t)bufSize;
                encode_parm.src_main_buf[i].buf_vaddr = (uint8_t *)pStreamMem->getPtr(i);
                encode_parm.src_main_buf[i].fd = pStreamMem->getFd(i);
                encode_parm.src_main_buf[i].format = MM_JPEG_FMT_YUV;
                encode_parm.src_main_buf[i].offset = main_offset;
            }
        }
    } else {
        encode_parm.num_src_bufs = 1;
        encode_parm.src_main_buf[0].index = 0;
        encode_parm.src_main_buf[0].buf_size = input_buf->frame_len;
        encode_parm.src_main_buf[0].buf_vaddr = (uint8_t *)input_buf->buffer;
        encode_parm.src_main_buf[0].fd = input_buf->fd;
        encode_parm.src_main_buf[0].format = MM_JPEG_FMT_YUV;
        encode_parm.src_main_buf[0].offset = main_offset;
    }

    //Pass input thumbnail buffer info to encoder.
    //Note: Use main buffer to encode thumbnail
    if (m_bThumbnailNeeded == TRUE) {
        pStreamMem = main_stream->getStreamBufs();
        if (pStreamMem == NULL) {
            LOGE("cannot get stream bufs from thumb stream");
            ret = BAD_VALUE;
            goto on_error;
        }
        cam_frame_len_offset_t thumb_offset;
        memset(&thumb_offset, 0, sizeof(cam_frame_len_offset_t));
        main_stream->getFrameOffset(thumb_offset);
        encode_parm.num_tmb_bufs = MIN(pStreamMem->getCnt(), MM_JPEG_MAX_BUF);
        for (uint32_t i = 0; i < encode_parm.num_tmb_bufs; i++) {
            if (pStreamMem != NULL) {
                encode_parm.src_thumb_buf[i].index = i;
                bufSize = pStreamMem->getSize(i);
                if (BAD_INDEX == bufSize) {
                    LOGE("cannot retrieve buffer size for buffer %u", i);
                    ret = BAD_VALUE;
                    goto on_error;
                }
                encode_parm.src_thumb_buf[i].buf_size = (uint32_t)bufSize;
                encode_parm.src_thumb_buf[i].buf_vaddr = (uint8_t *)pStreamMem->getPtr(i);
                encode_parm.src_thumb_buf[i].fd = pStreamMem->getFd(i);
                encode_parm.src_thumb_buf[i].format = MM_JPEG_FMT_YUV;
                encode_parm.src_thumb_buf[i].offset = thumb_offset;
            }
        }
    }

    //Pass output jpeg buffer info to encoder.
    //mJpegMem is allocated by framework.
    bufSize = outputMem->getSize(jpeg_settings->out_buf_index);
    if (BAD_INDEX == bufSize) {
        LOGE("cannot retrieve buffer size for buffer %u",
                jpeg_settings->out_buf_index);
        ret = BAD_VALUE;
        goto on_error;
    }

    encode_parm.num_dst_bufs = 1;
    encode_parm.dest_buf[0].index = 0;
    encode_parm.dest_buf[0].buf_size = (size_t)bufSize;
    encode_parm.dest_buf[0].buf_vaddr = (uint8_t *)outputMem->getPtr(
            jpeg_settings->out_buf_index);
    encode_parm.dest_buf[0].fd = outputMem->getFd(
            jpeg_settings->out_buf_index);
    encode_parm.dest_buf[0].format = MM_JPEG_FMT_YUV;
    encode_parm.dest_buf[0].offset = main_offset;

    LOGD("X");
    return NO_ERROR;

on_error:
    LOGD("X with error %d", ret);
    return ret;
}

int32_t QCamera3PostProcessor::processData(mm_camera_super_buf_t *input) {
    return processData(input, NULL, 0);
}

/*===========================================================================
 * FUNCTION   : processData
 *
 * DESCRIPTION: enqueue data into dataProc thread
 *
 * PARAMETERS :
 *   @frame   : process input frame
 *   @output  : process output frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : depends on if offline reprocess is needed, received frame will
 *              be sent to either input queue of postprocess or jpeg encoding
 *==========================================================================*/
int32_t QCamera3PostProcessor::processData(mm_camera_super_buf_t *input,
        buffer_handle_t *output, uint32_t frameNumber)
{
    LOGD("E");
    pthread_mutex_lock(&mReprocJobLock);

    // enqueue to post proc input queue
    qcamera_hal3_pp_buffer_t *pp_buffer = (qcamera_hal3_pp_buffer_t *)malloc(
            sizeof(qcamera_hal3_pp_buffer_t));
    if (NULL == pp_buffer) {
        LOGE("out of memory");
        return NO_MEMORY;
    }
    memset(pp_buffer, 0, sizeof(*pp_buffer));
    pp_buffer->input = input;
    pp_buffer->output = output;
    pp_buffer->frameNumber = frameNumber;
    if (!(m_inputMetaQ.isEmpty())) {
        qcamera_hal3_meta_pp_buffer_t *meta_job = isMetaMatched(frameNumber);
        if(meta_job != NULL) {
            ReprocessBuffer reproc;
            reproc.metaBuffer = meta_job;
            reproc.reprocBuf = pp_buffer;
            mReprocessNode.push_back(reproc);
            LOGD("meta queue is not empty, do next job");
            m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
        } else {
            m_inputPPQ.enqueue((void *)pp_buffer);
        }
    } else {
        LOGD("metadata queue is empty");
        m_inputPPQ.enqueue((void *)pp_buffer);
    }
    pthread_mutex_unlock(&mReprocJobLock);

    return NO_ERROR;
}


/*===========================================================================
 * FUNCTION   : isMetaMatched
 *
 * DESCRIPTION: corresponding meta detection
 *
 * PARAMETERS :
 *   @frame   : frame number
 *
 * RETURN     :
 *  TRUE if Frame is present
 *  FALSE if Frame is not released.
 *
 *==========================================================================*/
qcamera_hal3_meta_pp_buffer_t* QCamera3PostProcessor::isMetaMatched(uint32_t resultFrameNumber)
{
    qcamera_hal3_meta_pp_buffer_t *meta_job =
            (qcamera_hal3_meta_pp_buffer_t *)
            m_inputMetaQ.dequeue(matchMetaFrameNum, (void*)&resultFrameNumber);
    if(meta_job != NULL) {
        return meta_job;
    }
    return NULL;
}


/*===========================================================================
 * FUNCTION   : releaseReprocMetaBuffer
 *
 * DESCRIPTION: Release reprocessed Meta.
 *
 * PARAMETERS :
*   @frame   : meta frame number
 *
 * RETURN     :
 *  TRUE if Meta is released.
 *  FALSE if Meta is not released.
 *
 *==========================================================================*/
bool QCamera3PostProcessor::releaseReprocMetaBuffer(uint32_t resultFrameNumber)
{
    pthread_mutex_lock(&mReprocJobLock);
    if (!(m_inputMetaQ.isEmpty())) {
        mm_camera_super_buf_t *meta_buffer = NULL;
        qcamera_hal3_meta_pp_buffer_t *meta_job =
                (qcamera_hal3_meta_pp_buffer_t *)
                m_inputMetaQ.dequeue(matchMetaFrameNum, (void*)&resultFrameNumber);
        if(meta_job != NULL) {
            meta_buffer = meta_job->metabuf;
            m_parent->metadataBufDone(meta_buffer);
            free(meta_job);
            pthread_mutex_unlock(&mReprocJobLock);
            return true;
        }
    }
    pthread_mutex_unlock(&mReprocJobLock);
    return false;
}


bool QCamera3PostProcessor::matchMetaFrameNum(void *data, void *, void *match_data)
{
    qcamera_hal3_meta_pp_buffer_t *job = (qcamera_hal3_meta_pp_buffer_t *) data;
    uint32_t frame_num = *((uint32_t *) match_data);
    LOGD(" Matching MetaFrameNum :%d and %d", frame_num, job->metaFrameNumber);
    return job->metaFrameNumber == frame_num;
}


bool QCamera3PostProcessor::matchReprocessFrameNum(void *data, void *, void *match_data)
{
    qcamera_hal3_pp_buffer_t *job = (qcamera_hal3_pp_buffer_t *) data;
    uint32_t frame_num = *((uint32_t *) match_data);
    LOGD(" Matching FrameNum :%d and %d",frame_num,job->frameNumber);
    return job->frameNumber == frame_num;
}


/*===========================================================================
 * FUNCTION   : needsReprocess
 *
 * DESCRIPTION: Determine if reprocess is needed.
 *
 * PARAMETERS :
 *   @frame   : process frame
 *
 * RETURN     :
 *  TRUE if frame needs to be reprocessed
 *  FALSE is frame does not need to be reprocessed
 *
 *==========================================================================*/
bool QCamera3PostProcessor::needsReprocess(qcamera_fwk_input_pp_data_t *frame)
{
    metadata_buffer_t* meta = (metadata_buffer_t *) frame->metadata_buffer.buffer;
    bool edgeModeOn = FALSE;
    bool noiseRedModeOn = FALSE;
    bool reproNotDone = TRUE;

    if (frame->reproc_config.reprocess_type == REPROCESS_TYPE_NONE) {
        return FALSE;
    }

    // edge detection
    IF_META_AVAILABLE(cam_edge_application_t, edgeMode,
            CAM_INTF_META_EDGE_MODE, meta) {
        edgeModeOn = (CAM_EDGE_MODE_OFF != edgeMode->edge_mode);
    }

    // noise reduction
    IF_META_AVAILABLE(uint32_t, noiseRedMode,
            CAM_INTF_META_NOISE_REDUCTION_MODE, meta) {
        noiseRedModeOn = (CAM_NOISE_REDUCTION_MODE_OFF != *noiseRedMode);
    }

    IF_META_AVAILABLE(uint8_t, reprocess_flags,
            CAM_INTF_META_REPROCESS_FLAGS, meta) {
        reproNotDone = FALSE;
    }

    return (edgeModeOn || noiseRedModeOn || reproNotDone);
}

/*===========================================================================
 * FUNCTION   : processData
 *
 * DESCRIPTION: enqueue data into dataProc thread
 *
 * PARAMETERS :
 *   @frame   : process frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : depends on if offline reprocess is needed, received frame will
 *              be sent to either input queue of postprocess or jpeg encoding
 *==========================================================================*/
int32_t QCamera3PostProcessor::processData(qcamera_fwk_input_pp_data_t *frame)
{
    if (needsReprocess(frame)) {
        ATRACE_INT("Camera:Reprocess", 1);
        LOGH("scheduling framework reprocess");
        pthread_mutex_lock(&mReprocJobLock);
        // enqueu to post proc input queue
        m_inputFWKPPQ.enqueue((void *)frame);
        m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
        pthread_mutex_unlock(&mReprocJobLock);
    } else {
        jpeg_settings_t *jpeg_settings = (jpeg_settings_t *)m_jpegSettingsQ.dequeue();

        if (jpeg_settings == NULL) {
            LOGE("Cannot find jpeg settings");
            return BAD_VALUE;
        }

        LOGH("no need offline reprocess, sending to jpeg encoding");
        qcamera_hal3_jpeg_data_t *jpeg_job =
            (qcamera_hal3_jpeg_data_t *)malloc(sizeof(qcamera_hal3_jpeg_data_t));
        if (jpeg_job == NULL) {
            LOGE("No memory for jpeg job");
            return NO_MEMORY;
        }

        memset(jpeg_job, 0, sizeof(qcamera_hal3_jpeg_data_t));
        jpeg_job->fwk_frame = frame;
        jpeg_job->jpeg_settings = jpeg_settings;
        jpeg_job->metadata =
                (metadata_buffer_t *) frame->metadata_buffer.buffer;

        // enqueu to jpeg input queue
        m_inputJpegQ.enqueue((void *)jpeg_job);
        m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : processPPMetadata
 *
 * DESCRIPTION: enqueue data into dataProc thread
 *
 * PARAMETERS :
 *   @frame   : process metadata frame received from pic channel
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 *==========================================================================*/
int32_t QCamera3PostProcessor::processPPMetadata(mm_camera_super_buf_t *reproc_meta,
                               uint32_t framenum, bool dropFrame)
{
    LOGD("E");
    pthread_mutex_lock(&mReprocJobLock);

    qcamera_hal3_meta_pp_buffer_t *ppMetaBuf =
        (qcamera_hal3_meta_pp_buffer_t *)malloc(sizeof(qcamera_hal3_meta_pp_buffer_t));

    // enqueue to metadata input queue
    if (ppMetaBuf) {
        ppMetaBuf->metabuf = reproc_meta;
        ppMetaBuf->metaFrameNumber = framenum;
        ppMetaBuf->dropFrame = dropFrame;
    }
    /* Need to send notifyError before meta for Error Buffer */
    if (!(m_inputPPQ.isEmpty())) {
        qcamera_hal3_pp_buffer_t *reproc_job = isFrameMatched(framenum);
        if(reproc_job != NULL) {
            ReprocessBuffer reproc;
            reproc.metaBuffer = ppMetaBuf;
            reproc.reprocBuf = reproc_job;
            mReprocessNode.push_back(reproc);
            LOGD("pp queue is not empty, do next job");
            m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
        } else {
             m_inputMetaQ.enqueue((void *)ppMetaBuf);
        }
    } else {
             m_inputMetaQ.enqueue((void *)ppMetaBuf);
       LOGD("pp queue is empty, not calling do next job");
    }
    pthread_mutex_unlock(&mReprocJobLock);
    return NO_ERROR;
}


/*===========================================================================
 * FUNCTION   : isFrameMatched
 *
 * DESCRIPTION: corresponding meta detection
 *
 * PARAMETERS :
 *   @frame   : frame number
 *
 * RETURN     :
 *  TRUE if Frame is present
 *  FALSE if Frame is not released.
 *
 *==========================================================================*/
qcamera_hal3_pp_buffer_t* QCamera3PostProcessor::isFrameMatched(uint32_t resultFrameNumber)
{
    qcamera_hal3_pp_buffer_t *reprocess_job =
            (qcamera_hal3_pp_buffer_t *)
            m_inputPPQ.dequeue(matchReprocessFrameNum, (void*)&resultFrameNumber);
    if(reprocess_job != NULL)
        return reprocess_job;
    return NULL;
}


/*===========================================================================
 * FUNCTION   : processJpegSettingData
 *
 * DESCRIPTION: enqueue jpegSetting into dataProc thread
 *
 * PARAMETERS :
 *   @jpeg_settings : jpeg settings data received from pic channel
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 *==========================================================================*/
int32_t QCamera3PostProcessor::processJpegSettingData(
        jpeg_settings_t *jpeg_settings)
{
    if (!jpeg_settings) {
        LOGE("invalid jpeg settings pointer");
        return -EINVAL;
    }
    return m_jpegSettingsQ.enqueue((void *)jpeg_settings);
}

List<qcamera_hal3_mpo_data_t> QCamera3PostProcessor::mMpoInputData;

/*===========================================================================
 * FUNCTION   : processJpegData
 *
 * DESCRIPTION: handled Jpeg callback from encoder (BOKEH snapshot case).
 *              After receiving BOKEH, MAIN, DEPTH images composeMpo.
 *              handles releasing of snapshot buffer, reprocess frame.
 *
 * PARAMETERS :
 *   @status     : Jpeg error or success status.
 *   @client_hdl : __unused.
 *   @jobId      : Jpeg job id.
 *   @p_output   : Jpeg output image buffer.
 *   @userdata   : user handle for this callback.
 *
 * RETURN     : none
 *
 *==========================================================================*/
void QCamera3PostProcessor::processJpegData(jpeg_job_status_t status,
                                              uint32_t /*client_hdl*/,
                                              uint32_t jobId,
                                              mm_jpeg_output_t *p_output,
                                              void *userdata)
{
    QCamera3PostProcessor *obj = (QCamera3PostProcessor *)userdata;
    if(obj)
    {
        qcamera_hal3_jpeg_data_t *job = obj->findJpegJobByJobId(jobId);
        if((job == NULL) || (status == JPEG_JOB_STATUS_ERROR))
        {
            //Send Error buffer with status to MpoEvtHandle.
            LOGE("ERROR: JPEG Encoding failed");
        return;
        }

        qcamera_hal3_mpo_data_t mpo_input;

        mpo_input.jpeg_job = job;
        mpo_input.jpeg_image = *p_output;
        mpo_input.user_data = obj->mJpegUserData; //setting pic channel as userdata

        mMpoInputData.push_back(mpo_input);

        if(mMpoInputData.size() < MM_JPEG_MAX_MPO_IMAGES)
        {
            LOGH("need %d more jpeg images to compose mpo",
                                MM_JPEG_MAX_MPO_IMAGES - mMpoInputData.size());
            obj->doNextJob();
            return;
        } else{
            LOGI("Received %d images to compose MPO", mMpoInputData.size());
        }

        //getting the mpo output buffer from the main pic channel.
        QCamera3PicChannel *pic_channel = NULL;
        for(auto it = mMpoInputData.begin(); it != mMpoInputData.end(); it++)
        {
            qcamera_hal3_jpeg_data_t *job = it->jpeg_job;
            if(job->jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_MAIN)
            {
                pic_channel = (QCamera3PicChannel *)it->user_data;
                break;
            }
        }

        if(pic_channel == NULL)
        {
            LOGE("unexpected case: no pic channel to get mpo output buffer");
            return;
        }

        mm_jpeg_output_t mpo_output;
        memset (&mpo_output, 0, sizeof(mm_jpeg_output_t));
        pic_channel->getMpoOutputBuffer(&mpo_output);

        //main image should be bokeh output.
        //first aux will be main pic channel output.
        //second aux will be aux pic channel output.
        qcamera_hal3_mpo_compose_info_t mpo_info;
        memset (&mpo_info, 0, sizeof(qcamera_hal3_mpo_compose_info_t));
        for (auto it = mMpoInputData.begin(); it != mMpoInputData.end(); it++)
        {
            qcamera_hal3_jpeg_data_t *job = it->jpeg_job;
            if(job->jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_BOKEH)
            {
                LOGH("Received bokeh processed image");
                mpo_info.main_image = (it->jpeg_image);
            }
            else if(job->jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_MAIN) {
                LOGH("Received main image");
                mpo_info.aux_images[0] = (it->jpeg_image);
            }
            else if (job->jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_DEPTH) {
                LOGH("Received aux image");
                mpo_info.aux_images[1] = (it->jpeg_image);
            }
            cam_frame_len_offset_t offset;
            memset(&offset, 0, sizeof(cam_frame_len_offset_t));
            mm_camera_buf_def_t *jpeg_dump_buffer = NULL;
            //Dumping images.
            cam_dimension_t dim;
            dim.width = pic_channel->mCamera3Stream->width;
            dim.height = pic_channel->mCamera3Stream->height;
            jpeg_dump_buffer = (mm_camera_buf_def_t *)malloc(sizeof(mm_camera_buf_def_t));
            if(!jpeg_dump_buffer) {
                LOGE("Could not allocate jpeg dump buffer");
            } else {
                jpeg_dump_buffer->buffer = it->jpeg_image.buf_vaddr;
                jpeg_dump_buffer->frame_len = it->jpeg_image.buf_filled_len;
                jpeg_dump_buffer->frame_idx =
                    pic_channel->mJpegMemory.getFrameNumber(job->jpeg_settings->out_buf_index);
                pic_channel->dumpYUV(jpeg_dump_buffer, dim, offset, QCAMERA_DUMP_FRM_OUTPUT_JPEG);
                free(jpeg_dump_buffer);
            }
        }

        mpo_info.num_of_aux_image = MM_JPEG_MAX_MPO_IMAGES - 1;
        mpo_info.output = mpo_output;
        mpo_info.output.buf_filled_len = 0;

        //relaease snapshot buffer before sending mpo callback.
        for(auto it = mMpoInputData.begin(); it != mMpoInputData.end(); it++)
        {
            qcamera_hal3_jpeg_data_t *job = it->jpeg_job;
            QCamera3PicChannel * obj = (QCamera3PicChannel *)it->user_data;
            if(job->jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_MAIN
                 || job->jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_DEPTH)
            {
                obj->releaseSnapshotBuffer(job->src_reproc_frame);
            }
        }

        //compose MPO.
        //sends MPO event callback to pic channel;
        obj->composeMpo(mpo_info, pic_channel);

        //release JPEG JOB.
        auto it = mMpoInputData.begin();
        do {
            if(it == mMpoInputData.end())
            {
                break;
            }
            qcamera_hal3_jpeg_data_t *job = it->jpeg_job;
            QCamera3PicChannel * obj = (QCamera3PicChannel *)it->user_data;
            int bufIdx = job->jpeg_settings->out_buf_index;
             int frameNumber = obj->mJpegMemory.getFrameNumber(bufIdx);
            if((job->fwk_src_buffer != NULL) || (job->fwk_frame != NULL))
            {
                obj->releaseOfflineMemory(frameNumber);
            } else {
                //release offline buffers for MAIN and DEPTH (AUX channel).
                if(job->jpeg_settings->image_type != CAM_HAL3_JPEG_TYPE_BOKEH)
                {
                   obj->m_postprocessor.releaseOfflineBuffers(false);
                }
            }
            obj->freeBufferForJpeg(bufIdx);
            obj->m_postprocessor.releaseJpegJobData(job);
            free(job->jpeg_settings);
            free(job);
            mMpoInputData.erase(it);
            it = mMpoInputData.begin();
        }while(it != mMpoInputData.end());
    } else {
        LOGE("Error: Received NULL JPEG userdata");
    }
}

/*===========================================================================
 * FUNCTION   : processPPData
 *
 * DESCRIPTION: process received frame after reprocess.
 *
 * PARAMETERS :
 *   @frame   : received frame from reprocess channel.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : The frame after reprocess need to send to jpeg encoding.
 *==========================================================================*/
int32_t QCamera3PostProcessor::processPPData(mm_camera_super_buf_t *frame,
        const metadata_buffer_t *p_metadata)
{
    qcamera_hal3_pp_data_t *job = (qcamera_hal3_pp_data_t *)m_ongoingPPQ.dequeue();
    qcamera_hal3_pp_data_t *pending_job;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    bool needHalPP = hal_obj->needHALPP();
    LOGH("needHalPP:%d", needHalPP);
    ATRACE_INT("Camera:Reprocess", 0);
    if (job == NULL || ((NULL == job->src_frame) && (NULL == job->fwk_src_frame))) {
        LOGE("Cannot find reprocess job");
        return BAD_VALUE;
    }
    LOGD("jpeg settings is :%p and %d",job->jpeg_settings, m_ongoingPPQ.getCurrentSize());
    if (job->jpeg_settings == NULL )
    {
        //If needHALPP is true, checking for ouput jpeg settings != NULL
        if(hal_obj->needHALPP() && job->ppOutput_jpeg_settings == NULL) {
            LOGE("Cannot find jpeg settings");
            return BAD_VALUE;
        }
    }

    bool hdr_snapshot = FALSE;
    if((job->jpeg_settings != NULL && job->jpeg_settings->hdr_snapshot == 1))
    {
        hdr_snapshot = TRUE;
    }

    while((hdr_snapshot == TRUE) && (!m_ongoingPPQ.isEmpty()) ) {
        LOGD(" Checking if empty");
        pending_job = (qcamera_hal3_pp_data_t *)m_ongoingPPQ.dequeue();
        if ((pending_job != NULL)) {
            LOGD("free reprocessed buffer");
            m_parent->freeBufferForFrame(pending_job->src_frame);
            m_parent->metadataBufDone(pending_job->src_metadata);
        }
    }

    if ((m_pHalPPManager != NULL) && needHalPP &&
                (hal_obj->getHalPPType() != CAM_HAL_PP_TYPE_NONE)) {
        qcamera_hal_pp_data_t *hal_pp_job =
            (qcamera_hal_pp_data_t*) malloc(sizeof(qcamera_hal_pp_data_t));
        if (hal_pp_job == NULL) {
            LOGE("No memory for qcamera_hal_pp_data_t data");
            return NO_MEMORY;
        }
        memset(hal_pp_job, 0, sizeof(qcamera_hal_pp_data_t));


        // find snapshot frame
        QCamera3Channel * srcChannel = getChannelByHandle(frame->ch_id);
        if (srcChannel == NULL) {
            LOGE("No corresponding channel (ch_id = %d) exist, return here",
                frame->ch_id);
            return BAD_VALUE;
        }
        QCamera3Stream *pSnapStream = NULL;
        for (uint32_t i = 0; i < frame->num_bufs; i++) {
            QCamera3Stream *pStream =
                srcChannel->getStreamByHandle(frame->bufs[i]->stream_id);
            if ((pStream != NULL) &&
                (pStream->getMyType() == CAM_STREAM_TYPE_SNAPSHOT ||
                 pStream->getMyType() == CAM_STREAM_TYPE_OFFLINE_PROC)) {
                pSnapStream = pStream;
                break;
            }
        }

        //get snapshot offset info
        cam_frame_len_offset_t snap_offset, meta_offset;
        memset(&snap_offset, 0, sizeof(cam_frame_len_offset_t));
        memset(&meta_offset, 0, sizeof(cam_frame_len_offset_t));
        if (pSnapStream != NULL) {
            pSnapStream->getFrameOffset(snap_offset);
        }

        // find meta frame
        srcChannel = getChannelByHandle(job->src_frame->ch_id);
        if (srcChannel == NULL) {
            LOGE("No corresponding channel (ch_id = %d) exist, return here",
                job->src_frame->ch_id);
            return BAD_VALUE;
        }
        QCamera3Stream *pMetaStream = NULL;
        for (uint32_t i = 0; i < job->src_frame->num_bufs; i++) {
            QCamera3Stream *pStream =
                srcChannel->getStreamByHandle(frame->bufs[i]->stream_id);
            if ((pStream != NULL) &&
                (pStream->getMyType() == CAM_STREAM_TYPE_SNAPSHOT ||
                 pStream->getMyType() == CAM_STREAM_TYPE_OFFLINE_PROC)) {
                pMetaStream = pStream;
                break;
            }
        }
        if (pMetaStream != NULL) {
            pMetaStream->getFrameOffset(meta_offset);
        }

        hal_pp_job->frame = frame;
        hal_pp_job->snap_offset = snap_offset;
        hal_pp_job->meta_offset = meta_offset;
        hal_pp_job->frameIndex = frame->bufs[0]->frame_idx;
        hal_pp_job->src_reproc_frame = job ? job->src_frame : NULL;
        hal_pp_job->metadata = job ? job->metadata : NULL;
        hal_pp_job->jpeg_settings = job ? job->jpeg_settings : NULL;
        hal_pp_job->output_jpeg_settings = job ? job->ppOutput_jpeg_settings : NULL;
        hal_pp_job->src_metadata = job ? job->src_metadata : NULL;
        hal_pp_job->pUserData = this;
        //adding blur information for BOKEH process.
        if(hal_pp_job->metadata != NULL)
        {
            QCamera3HardwareInterface * hal_obj = (QCamera3HardwareInterface *)m_parent->mUserData;
            uint32_t blurlevel = hal_obj->getBlurLevel();
            cam_rtb_blur_info_t blurInfo;
            blurInfo.blur_level = blurlevel;
            blurInfo.blur_max_value = MAX_BLUR;
            blurInfo.blur_min_value = MIN_BLUR;
            ADD_SET_PARAM_ENTRY_TO_BATCH(hal_pp_job->metadata,
                                         CAM_INTF_PARAM_BOKEH_BLUR_LEVEL, blurInfo);
        }
        LOGH("Feeding input to Manager");
        m_pHalPPManager->feedInput(hal_pp_job);
        free(job);
        return NO_ERROR;
    }

    LOGH("pp_ch_idx:%d, total_pp_count:%d, frame number:%d", job->pp_ch_idx,
            m_ppChannelCnt, job->frameNumber);
    if ((job->pp_ch_idx+1) < m_ppChannelCnt) {
        job->pp_ch_idx++;
        LOGH("next pp index:%d.", job->pp_ch_idx);

        if (job->fwk_src_frame != NULL) {
            LOGD("reprocess for fwk input frame.");
            if (p_metadata != NULL) {
                memcpy(job->fwk_src_frame->metadata_buffer.buffer, p_metadata,
                            sizeof(metadata_buffer_t));
            }
        } else if (job->src_frame != NULL) {
            LOGD("reprocess for non-fwk input frame.");
            if (p_metadata != NULL && job->metadata != NULL) {
                memcpy(job->metadata, p_metadata, sizeof(metadata_buffer_t));
            }
        }

        // the src frame here should be from output of previous reprocess channel
        job->reprocessed_src_frame = frame;
        m_inputMultiReprocQ.enqueue(job);

        m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
        return NO_ERROR;
    }

    qcamera_hal3_jpeg_data_t *jpeg_job =
        (qcamera_hal3_jpeg_data_t *)malloc(sizeof(qcamera_hal3_jpeg_data_t));
    if (jpeg_job == NULL) {
        LOGE("No memory for jpeg job");
        return NO_MEMORY;
    }

    memset(jpeg_job, 0, sizeof(qcamera_hal3_jpeg_data_t));
    jpeg_job->src_frame = frame;
    if(frame != job->src_frame)
        jpeg_job->src_reproc_frame = job->src_frame;
    if (NULL == job->fwk_src_frame) {
        jpeg_job->metadata = job->metadata;
    } else {
        jpeg_job->metadata =
                (metadata_buffer_t *) job->fwk_src_frame->metadata_buffer.buffer;
        jpeg_job->fwk_src_buffer = job->fwk_src_frame;
    }
    if (p_metadata != NULL) {
        // update metadata content with input buffer
        memcpy(jpeg_job->metadata, p_metadata, sizeof(metadata_buffer_t));
    }
    jpeg_job->src_metadata = job->src_metadata;
    jpeg_job->jpeg_settings = job->jpeg_settings;

    if (job->reprocessed_src_frame != NULL) {
        LOGD("release output buffers of previous reprocess channel");
        for (int8_t i = 0; i < m_ppChannelCnt; i++) {
            if (m_pReprocChannel[i] != NULL &&
                m_pReprocChannel[i]->getMyHandle() == job->reprocessed_src_frame->ch_id) {
                int32_t rc = m_pReprocChannel[i]->bufDone(job->reprocessed_src_frame);
                if (NO_ERROR != rc) {
                    LOGE("bufDone error: %d", rc);
                }
                break;
            }
        }
        free(job->reprocessed_src_frame);
        job->reprocessed_src_frame = NULL;
    }

    // free pp job buf
    free(job);

    // enqueu reprocessed frame to jpeg input queue
    m_inputJpegQ.enqueue((void *)jpeg_job);

    // wait up data proc thread
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : dequeuePPJob
 *
 * DESCRIPTION: find a postprocessing job from ongoing pp queue by frame number
 *
 * PARAMETERS :
 *   @frameNumber : frame number for the pp job
 *
 * RETURN     : ptr to a pp job struct. NULL if not found.
 *==========================================================================*/
qcamera_hal3_pp_data_t *QCamera3PostProcessor::dequeuePPJob(uint32_t frameNumber) {
    qcamera_hal3_pp_data_t *pp_job = NULL;
    pp_job = (qcamera_hal3_pp_data_t *)m_ongoingPPQ.dequeue();

    if (pp_job == NULL) {
        LOGE("Fatal: ongoing PP queue is empty");
        return NULL;
    }
    if (pp_job->fwk_src_frame &&
            (pp_job->fwk_src_frame->frameNumber != frameNumber)) {
        LOGE("head of pp queue doesn't match requested frame number");
    }
    return pp_job;
}

/*===========================================================================
 * FUNCTION   : findJpegJobByJobId
 *
 * DESCRIPTION: find a jpeg job from ongoing Jpeg queue by its job ID
 *
 * PARAMETERS :
 *   @jobId   : job Id of the job
 *
 * RETURN     : ptr to a jpeg job struct. NULL if not found.
 *
 * NOTE       : Currently only one job is sending to mm-jpeg-interface for jpeg
 *              encoding. Therefore simply dequeue from the ongoing Jpeg Queue
 *              will serve the purpose to find the jpeg job.
 *==========================================================================*/
qcamera_hal3_jpeg_data_t *QCamera3PostProcessor::findJpegJobByJobId(uint32_t jobId)
{
    qcamera_hal3_jpeg_data_t * job = NULL;
    if (jobId == 0) {
        LOGE("not a valid jpeg jobId");
        return NULL;
    }

    // currely only one jpeg job ongoing, so simply dequeue the head
    job = (qcamera_hal3_jpeg_data_t *)m_ongoingJpegQ.dequeue();
    return job;
}

/*===========================================================================
 * FUNCTION   : releasePPInputData
 *
 * DESCRIPTION: callback function to release post process input data node
 *
 * PARAMETERS :
 *   @data      : ptr to post process input data
 *   @user_data : user data ptr (QCamera3Reprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releasePPInputData(void *data, void *user_data)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)user_data;
    if (NULL != pme) {
        qcamera_hal3_pp_buffer_t *buf = (qcamera_hal3_pp_buffer_t *)data;
        if (NULL != buf) {
            if (buf->input) {
                pme->releaseSuperBuf(buf->input);
                free(buf->input);
                buf->input = NULL;
            }
        }
    }
}

/*===========================================================================
 * FUNCTION   : timeoutFrame
 *
 * DESCRIPTION: Function to handle timeouts in reprocess
 *
 * PARAMETERS :
 *   @frameNumber      : reprocess frame number that timed out
 *
 * RETURN     : NO_ERROR or valid error number
 *==========================================================================*/
int32_t QCamera3PostProcessor::timeoutFrame(uint32_t frameNumber)
{
    int32_t rc = NO_ERROR;

    if(NULL != m_pReprocChannel[0]) {
        rc = m_pReprocChannel[0]->timeoutFrame(frameNumber);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseMetaData
 *
 * DESCRIPTION: callback function to release metadata camera buffer
 *
 * PARAMETERS :
 *   @data      : ptr to post process input data
 *   @user_data : user data ptr (QCamera3Reprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releaseMetadata(void *data, void *user_data)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)user_data;
    if (NULL != pme) {
        qcamera_hal3_meta_pp_buffer_t *buf  = (qcamera_hal3_meta_pp_buffer_t *)data;
        pme->m_parent->metadataBufDone((mm_camera_super_buf_t *)buf->metabuf);
        free(buf);
    }
}

/*===========================================================================
 * FUNCTION   : releaseJpegData
 *
 * DESCRIPTION: callback function to release jpeg job node
 *
 * PARAMETERS :
 *   @data      : ptr to ongoing jpeg job data
 *   @user_data : user data ptr (QCamera3Reprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releaseJpegData(void *data, void *user_data)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)user_data;
    if (NULL != pme) {
        pme->releaseJpegJobData((qcamera_hal3_jpeg_data_t *)data);
    }
}

/*===========================================================================
 * FUNCTION   : releaseOngoingPPData
 *
 * DESCRIPTION: callback function to release ongoing postprocess job node
 *
 * PARAMETERS :
 *   @data      : ptr to onging postprocess job
 *   @user_data : user data ptr (QCamera3Reprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releaseOngoingPPData(void *data, void *user_data)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)user_data;
    if (NULL != pme) {
        qcamera_hal3_pp_data_t *pp_data = (qcamera_hal3_pp_data_t *)data;

        if (pp_data && pp_data->src_frame)
          pme->releaseSuperBuf(pp_data->src_frame);

        pme->releasePPJobData(pp_data);

    }
}

/*===========================================================================
 * FUNCTION   : releaseSuperBuf
 *
 * DESCRIPTION: function to release a superbuf frame by returning back to kernel
 *
 * PARAMETERS :
 *   @super_buf : ptr to the superbuf frame
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::releaseSuperBuf(mm_camera_super_buf_t *super_buf)
{
    if (NULL != super_buf) {
        if (m_parent != NULL) {
            m_parent->bufDone(super_buf);
        }
    }
}

/*===========================================================================
 * FUNCTION   : releaseOfflineBuffers
 *
 * DESCRIPTION: function to release/unmap offline buffers if any
 *
 * PARAMETERS :
 * @allBuffers : flag that asks to release all buffers or only one
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::releaseOfflineBuffers(bool allBuffers)
{
    int32_t rc = NO_ERROR;

    for (int8_t i = 0; i < m_ppChannelCnt; i++) {
        QCamera3ReprocessChannel *pChannel = m_pReprocChannel[i];
        if (pChannel != NULL ) {
            rc |= pChannel->unmapOfflineBuffers(allBuffers);
        }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : releaseJpegJobData
 *
 * DESCRIPTION: function to release internal resources in jpeg job struct
 *
 * PARAMETERS :
 *   @job     : ptr to jpeg job struct
 *
 * RETURN     : None
 *
 * NOTE       : original source frame need to be queued back to kernel for
 *              future use. Output buf of jpeg job need to be released since
 *              it's allocated for each job. Exif object need to be deleted.
 *==========================================================================*/
void QCamera3PostProcessor::releaseJpegJobData(qcamera_hal3_jpeg_data_t *job)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PPROC_REL_JPEG_JOB_DATA);
    int32_t rc = NO_ERROR;
    LOGD("E");
    if (NULL != job) {
        if (NULL != job->src_reproc_frame) {
            free(job->src_reproc_frame);
            job->src_reproc_frame = NULL;
        }

        if (NULL != job->src_frame) {
            if(!job->halPPAllocatedBuf)
            {
                // find the correct reprocess channel for the super buffer
                for (int8_t i = 0; i < m_ppChannelCnt; i++) {
                    if (m_pReprocChannel[i] != NULL &&
                        m_pReprocChannel[i]->getMyHandle() == job->src_frame->ch_id) {
                        rc = m_pReprocChannel[i]->bufDone(job->src_frame);
                        if (NO_ERROR != rc) {
                            LOGE("bufDone error: %d", rc);
                        }
                        break;
                    }
                }
            } else {
               if (job->hal_pp_bufs) {
                    free(job->hal_pp_bufs);
                    job->hal_pp_bufs = NULL;
                }
                if (job->snapshot_heap) {
                    job->snapshot_heap->deallocate();
                    delete job->snapshot_heap;
                    job->snapshot_heap = NULL;
                }
                if (job->metadata_heap) {
                    job->metadata_heap->deallocate();
                    delete job->metadata_heap;
                    job->metadata_heap = NULL;
                }
            }

            free(job->src_frame);
            job->src_frame = NULL;
        }

        if (NULL != job->fwk_src_buffer) {
            free(job->fwk_src_buffer);
            job->fwk_src_buffer = NULL;
        } else if (NULL != job->src_metadata) {
            m_parent->metadataBufDone(job->src_metadata);
            free(job->src_metadata);
            job->src_metadata = NULL;
        }

        if (NULL != job->fwk_frame) {
            free(job->fwk_frame);
            job->fwk_frame = NULL;
        }

        if (NULL != job->pJpegExifObj) {
            delete job->pJpegExifObj;
            job->pJpegExifObj = NULL;
        }

        if (NULL != job->jpeg_settings) {
            free(job->jpeg_settings);
            job->jpeg_settings = NULL;
        }
    }
    /* Additional trigger to process any pending jobs in the input queue */
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    LOGD("X");
}

/*===========================================================================
 * FUNCTION   : releasePPJobData
 *
 * DESCRIPTION: function to release internal resources in p pjob struct
 *
 * PARAMETERS :
 *   @job     : ptr to pp job struct
 *
 * RETURN     : None
 *
 * NOTE       : Original source metadata buffer needs to be released and
 *              queued back to kernel for future use. src_frame, src_metadata,
 *              and fwk_src_frame structures need to be freed.
 *==========================================================================*/
void QCamera3PostProcessor::releasePPJobData(qcamera_hal3_pp_data_t *pp_job)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PPROC_REL_PP_JOB_DATA);
    LOGD("E");
    if (NULL != pp_job) {
        if (NULL != pp_job->src_frame) {
            free(pp_job->src_frame);
            if (NULL != pp_job->src_metadata) {
                m_parent->metadataBufDone(pp_job->src_metadata);
                free(pp_job->src_metadata);
            }
            pp_job->src_frame = NULL;
            pp_job->metadata = NULL;
        }

        if (NULL != pp_job->fwk_src_frame) {
            free(pp_job->fwk_src_frame);
            pp_job->fwk_src_frame = NULL;
        }
    }

    /* Additional trigger to process any pending jobs in the input queue */
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
    LOGD("X");
}

/*===========================================================================
 * FUNCTION   : getColorfmtFromImgFmt
 *
 * DESCRIPTION: function to return jpeg color format based on its image format
 *
 * PARAMETERS :
 *   @img_fmt : image format
 *
 * RETURN     : jpeg color format that can be understandable by omx lib
 *==========================================================================*/
mm_jpeg_color_format QCamera3PostProcessor::getColorfmtFromImgFmt(cam_format_t img_fmt)
{
    switch (img_fmt) {
    case CAM_FORMAT_YUV_420_NV21:
    case CAM_FORMAT_YUV_420_NV21_VENUS:
        return MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
    case CAM_FORMAT_YUV_420_NV21_ADRENO:
        return MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
    case CAM_FORMAT_YUV_420_NV12:
    case CAM_FORMAT_YUV_420_NV12_VENUS:
        return MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2;
    case CAM_FORMAT_YUV_420_YV12:
        return MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2;
    case CAM_FORMAT_YUV_422_NV61:
        return MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V1;
    case CAM_FORMAT_YUV_422_NV16:
        return MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V1;
    case CAM_FORMAT_Y_ONLY:
        return MM_JPEG_COLOR_FORMAT_MONOCHROME;
    default:
        return MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
    }
}

/*===========================================================================
 * FUNCTION   : getJpegImgTypeFromImgFmt
 *
 * DESCRIPTION: function to return jpeg encode image type based on its image format
 *
 * PARAMETERS :
 *   @img_fmt : image format
 *
 * RETURN     : return jpeg source image format (YUV or Bitstream)
 *==========================================================================*/
mm_jpeg_format_t QCamera3PostProcessor::getJpegImgTypeFromImgFmt(cam_format_t img_fmt)
{
    switch (img_fmt) {
    case CAM_FORMAT_YUV_420_NV21:
    case CAM_FORMAT_YUV_420_NV21_ADRENO:
    case CAM_FORMAT_YUV_420_NV12:
    case CAM_FORMAT_YUV_420_NV12_VENUS:
    case CAM_FORMAT_YUV_420_NV21_VENUS:
    case CAM_FORMAT_YUV_420_YV12:
    case CAM_FORMAT_YUV_422_NV61:
    case CAM_FORMAT_YUV_422_NV16:
        return MM_JPEG_FMT_YUV;
    default:
        return MM_JPEG_FMT_YUV;
    }
}

/*===========================================================================
 * FUNCTION   : encodeFWKData
 *
 * DESCRIPTION: function to prepare encoding job information and send to
 *              mm-jpeg-interface to do the encoding job
 *
 * PARAMETERS :
 *   @jpeg_job_data : ptr to a struct saving job related information
 *   @needNewSess   : flag to indicate if a new jpeg encoding session need
 *                    to be created. After creation, this flag will be toggled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::encodeFWKData(qcamera_hal3_jpeg_data_t *jpeg_job_data,
        uint8_t &needNewSess)
{
    LOGD("E");
    int32_t ret = NO_ERROR;
    mm_jpeg_job_t jpg_job;
    uint32_t jobId = 0;
    qcamera_fwk_input_pp_data_t *recvd_frame = NULL;
    metadata_buffer_t *metadata = NULL;
    jpeg_settings_t *jpeg_settings = NULL;
    QCamera3HardwareInterface* hal_obj = NULL;
    mm_jpeg_debug_exif_params_t *exif_debug_params = NULL;
    bool needJpegExifRotation = false;

    if (NULL == jpeg_job_data) {
        LOGE("Invalid jpeg job");
        return BAD_VALUE;
    }

    recvd_frame = jpeg_job_data->fwk_frame;
    if (NULL == recvd_frame) {
        LOGE("Invalid input buffer");
        return BAD_VALUE;
    }

    metadata = jpeg_job_data->metadata;
    if (NULL == metadata) {
        LOGE("Invalid metadata buffer");
        return BAD_VALUE;
    }

    jpeg_settings = jpeg_job_data->jpeg_settings;
    if (NULL == jpeg_settings) {
        LOGE("Invalid jpeg settings buffer");
        return BAD_VALUE;
    }

    if ((NULL != jpeg_job_data->fwk_frame) && (NULL != jpeg_job_data->src_frame)) {
        LOGE("Unsupported case both framework and camera source buffers are invalid!");
        return BAD_VALUE;
    }

    hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    if (hal_obj == NULL) {
        LOGE("hal_obj is NULL, Error");
        return BAD_VALUE;
    }

    if (mJpegClientHandle <= 0) {
        LOGE("Error: bug here, mJpegClientHandle is 0");
        return UNKNOWN_ERROR;
    }

    cam_dimension_t src_dim;
    memset(&src_dim, 0, sizeof(cam_dimension_t));
    src_dim.width = recvd_frame->reproc_config.input_stream_dim.width;
    src_dim.height = recvd_frame->reproc_config.input_stream_dim.height;

    cam_dimension_t dst_dim;
    memset(&dst_dim, 0, sizeof(cam_dimension_t));
    dst_dim.width = recvd_frame->reproc_config.output_stream_dim.width;
    dst_dim.height = recvd_frame->reproc_config.output_stream_dim.height;

    cam_rect_t crop;
    memset(&crop, 0, sizeof(cam_rect_t));
    //TBD_later - Zoom event removed in stream
    //main_stream->getCropInfo(crop);
    // Set JPEG encode crop in reprocess frame metadata
    // If this JPEG crop info exist, encoder should do cropping
    IF_META_AVAILABLE(cam_stream_crop_info_t, jpeg_crop,
            CAM_INTF_PARM_JPEG_ENCODE_CROP, metadata) {
        memcpy(&crop, &(jpeg_crop->crop), sizeof(cam_rect_t));
    }

    // Set JPEG encode crop in reprocess frame metadata
    // If this JPEG scale info exist, encoder should do scaling
    IF_META_AVAILABLE(cam_dimension_t, scale_dim,
            CAM_INTF_PARM_JPEG_SCALE_DIMENSION, metadata) {
        if (scale_dim->width != 0 && scale_dim->height != 0) {
            dst_dim.width = scale_dim->width;
            dst_dim.height = scale_dim->height;
        }
    }

    needJpegExifRotation = (hal_obj->needJpegExifRotation() || !needsReprocess(recvd_frame));

    // If EXIF rotation metadata is added and used to match the JPEG orientation,
    // it means CPP rotation is not involved, whether it is because CPP does not
    // support rotation, or the reprocessed frame is not sent to CPP.
    // Override CAM_INTF_PARM_ROTATION to 0 to avoid wrong CPP rotation info
    // to be filled in to JPEG metadata.
    if (needJpegExifRotation) {
        cam_rotation_info_t rotation_info;
        memset(&rotation_info, 0, sizeof(rotation_info));
        rotation_info.rotation = ROTATE_0;
        rotation_info.streamId = 0;
        ADD_SET_PARAM_ENTRY_TO_BATCH(metadata, CAM_INTF_PARM_ROTATION, rotation_info);
    }

    LOGH("Need new session?:%d jpeg_orientation %d needJpegExifRotation %d useExifRotation %d",
            needNewSess, jpeg_settings->jpeg_orientation, needJpegExifRotation,
            hal_obj->useExifRotation());
    if (needNewSess) {
        //creating a new session, so we must destroy the old one
        if ( 0 < mJpegSessionId ) {
            ret = mJpegHandle.destroy_session(mJpegSessionId);
            if (ret != NO_ERROR) {
                LOGE("Error destroying an old jpeg encoding session, id = %d",
                       mJpegSessionId);
                return ret;
            }
            mJpegSessionId = 0;
        }
        // create jpeg encoding session
        mm_jpeg_encode_params_t encodeParam;
        memset(&encodeParam, 0, sizeof(mm_jpeg_encode_params_t));
        getFWKJpegEncodeConfig(encodeParam, recvd_frame, jpeg_settings);
        QCamera3StreamMem *memObj = (QCamera3StreamMem *)(recvd_frame->input_buffer.mem_info);
        if (NULL == memObj) {
            LOGE("Memeory Obj of main frame is NULL");
            return NO_MEMORY;
        }
        // clean and invalidate cache ops through mem obj of the frame
        memObj->cleanInvalidateCache(recvd_frame->input_buffer.buf_idx);

        LOGH("#src bufs:%d # tmb bufs:%d #dst_bufs:%d",
                     encodeParam.num_src_bufs,encodeParam.num_tmb_bufs,encodeParam.num_dst_bufs);
        if (!needJpegExifRotation &&
            (jpeg_settings->jpeg_orientation == 90 ||
            jpeg_settings->jpeg_orientation == 270)) {
            // swap src width and height, stride and scanline due to rotation
            encodeParam.main_dim.src_dim.width = src_dim.height;
            encodeParam.main_dim.src_dim.height = src_dim.width;
            encodeParam.thumb_dim.src_dim.width = src_dim.height;
            encodeParam.thumb_dim.src_dim.height = src_dim.width;

            int32_t temp = encodeParam.src_main_buf[0].offset.mp[0].stride;
            encodeParam.src_main_buf[0].offset.mp[0].stride =
                encodeParam.src_main_buf[0].offset.mp[0].scanline;
            encodeParam.src_main_buf[0].offset.mp[0].scanline = temp;

            temp = encodeParam.src_thumb_buf[0].offset.mp[0].stride;
            encodeParam.src_thumb_buf[0].offset.mp[0].stride =
                encodeParam.src_thumb_buf[0].offset.mp[0].scanline;
            encodeParam.src_thumb_buf[0].offset.mp[0].scanline = temp;
        } else {
            encodeParam.main_dim.src_dim = src_dim;
            encodeParam.thumb_dim.src_dim = src_dim;
        }
        encodeParam.main_dim.dst_dim = dst_dim;
        encodeParam.thumb_dim.dst_dim = jpeg_settings->thumbnail_size;

        if (!hal_obj->useExifRotation() && needJpegExifRotation) {
            encodeParam.rotation = jpeg_settings->jpeg_orientation;
            encodeParam.thumb_rotation = jpeg_settings->jpeg_orientation;
        }

        LOGI("Src Buffer cnt = %d, res = %dX%d len = %d rot = %d "
            "src_dim = %dX%d dst_dim = %dX%d",
            encodeParam.num_src_bufs,
            encodeParam.src_main_buf[0].offset.mp[0].stride,
            encodeParam.src_main_buf[0].offset.mp[0].scanline,
            encodeParam.src_main_buf[0].offset.frame_len,
            encodeParam.rotation,
            src_dim.width, src_dim.height,
            dst_dim.width, dst_dim.height);
        LOGI("Src THUMB buf_cnt = %d, res = %dX%d len = %d rot = %d "
            "src_dim = %dX%d, dst_dim = %dX%d",
            encodeParam.num_tmb_bufs,
            encodeParam.src_thumb_buf[0].offset.mp[0].stride,
            encodeParam.src_thumb_buf[0].offset.mp[0].scanline,
            encodeParam.src_thumb_buf[0].offset.frame_len,
            encodeParam.thumb_rotation,
            encodeParam.thumb_dim.src_dim.width,
            encodeParam.thumb_dim.src_dim.height,
            encodeParam.thumb_dim.dst_dim.width,
            encodeParam.thumb_dim.dst_dim.height);

        LOGH("#src bufs:%d # tmb bufs:%d #dst_bufs:%d",
                     encodeParam.num_src_bufs,encodeParam.num_tmb_bufs,encodeParam.num_dst_bufs);

        ret = mJpegHandle.create_session(mJpegClientHandle, &encodeParam, &mJpegSessionId);
        if (ret != NO_ERROR) {
            LOGE("Error creating a new jpeg encoding session, ret = %d", ret);
            return ret;
        }
        needNewSess = FALSE;
    }

    // Fill in new job
    memset(&jpg_job, 0, sizeof(mm_jpeg_job_t));
    jpg_job.job_type = JPEG_JOB_TYPE_ENCODE;
    jpg_job.encode_job.session_id = mJpegSessionId;
    jpg_job.encode_job.src_index = 0;
    jpg_job.encode_job.dst_index = 0;

    // Set main dim job parameters and handle rotation
    if (!needJpegExifRotation && (jpeg_settings->jpeg_orientation == 90 ||
            jpeg_settings->jpeg_orientation == 270)) {

        jpg_job.encode_job.main_dim.src_dim.width = src_dim.height;
        jpg_job.encode_job.main_dim.src_dim.height = src_dim.width;

        jpg_job.encode_job.main_dim.dst_dim.width = dst_dim.height;
        jpg_job.encode_job.main_dim.dst_dim.height = dst_dim.width;

        jpg_job.encode_job.main_dim.crop.width = crop.height;
        jpg_job.encode_job.main_dim.crop.height = crop.width;
        jpg_job.encode_job.main_dim.crop.left = crop.top;
        jpg_job.encode_job.main_dim.crop.top = crop.left;
    } else {
        jpg_job.encode_job.main_dim.src_dim = src_dim;
        jpg_job.encode_job.main_dim.dst_dim = dst_dim;
        jpg_job.encode_job.main_dim.crop = crop;
    }

    // get 3a sw version info
    cam_q3a_version_t sw_version;
    memset(&sw_version, 0, sizeof(sw_version));
    if (hal_obj)
        hal_obj->get3AVersion(sw_version);

    // get exif data
    QCamera3Exif *pJpegExifObj = getExifData(metadata, jpeg_settings,
            (needJpegExifRotation && hal_obj->useExifRotation()));
    jpeg_job_data->pJpegExifObj = pJpegExifObj;
    if (pJpegExifObj != NULL) {
        jpg_job.encode_job.exif_info.exif_data = pJpegExifObj->getEntries();
        jpg_job.encode_job.exif_info.numOfEntries =
            pJpegExifObj->getNumOfEntries();
        jpg_job.encode_job.exif_info.debug_data.sw_3a_version[0] =
            sw_version.major_version;
        jpg_job.encode_job.exif_info.debug_data.sw_3a_version[1] =
            sw_version.minor_version;
        jpg_job.encode_job.exif_info.debug_data.sw_3a_version[2] =
            sw_version.patch_version;
        jpg_job.encode_job.exif_info.debug_data.sw_3a_version[3] =
            sw_version.new_feature_des;
    }

    if (!hal_obj->useExifRotation() && needJpegExifRotation) {
        jpg_job.encode_job.rotation= jpeg_settings->jpeg_orientation;
    }

    // thumbnail dim
    LOGH("Thumbnail needed:%d", m_bThumbnailNeeded);
    if (m_bThumbnailNeeded == TRUE) {
        jpg_job.encode_job.thumb_dim.dst_dim =
                jpeg_settings->thumbnail_size;

        if (!needJpegExifRotation && (jpeg_settings->jpeg_orientation == 90 ||
                jpeg_settings->jpeg_orientation == 270)) {
            //swap the thumbnail destination width and height if it has
            //already been rotated
            int temp = jpg_job.encode_job.thumb_dim.dst_dim.width;
            jpg_job.encode_job.thumb_dim.dst_dim.width =
                    jpg_job.encode_job.thumb_dim.dst_dim.height;
            jpg_job.encode_job.thumb_dim.dst_dim.height = temp;

            jpg_job.encode_job.thumb_dim.src_dim.width = src_dim.height;
            jpg_job.encode_job.thumb_dim.src_dim.height = src_dim.width;

            jpg_job.encode_job.thumb_dim.crop.width = crop.height;
            jpg_job.encode_job.thumb_dim.crop.height = crop.width;
            jpg_job.encode_job.thumb_dim.crop.left = crop.top;
            jpg_job.encode_job.thumb_dim.crop.top = crop.left;
        } else {
        jpg_job.encode_job.thumb_dim.src_dim = src_dim;
        jpg_job.encode_job.thumb_dim.crop = crop;
        }
        jpg_job.encode_job.thumb_index = 0;
        LOGI("Thumbnail idx = %d src w/h (%dx%d), dst w/h (%dx%d)",
                jpg_job.encode_job.thumb_index,
                jpg_job.encode_job.thumb_dim.src_dim.width,
                jpg_job.encode_job.thumb_dim.src_dim.height,
                jpg_job.encode_job.thumb_dim.dst_dim.width,
                jpg_job.encode_job.thumb_dim.dst_dim.height);
    }

    LOGI("Main image idx = %d src w/h (%dx%d), dst w/h (%dx%d) rot = %d",
            jpg_job.encode_job.src_index,
            jpg_job.encode_job.main_dim.src_dim.width,
            jpg_job.encode_job.main_dim.src_dim.height,
            jpg_job.encode_job.main_dim.dst_dim.width,
            jpg_job.encode_job.main_dim.dst_dim.height,
            jpg_job.encode_job.rotation);
    // Allocate for a local copy of debug parameters
    jpg_job.encode_job.cam_exif_params.debug_params =
            (mm_jpeg_debug_exif_params_t *) malloc (sizeof(mm_jpeg_debug_exif_params_t));
    if (!jpg_job.encode_job.cam_exif_params.debug_params) {
        LOGE("Out of Memory. Allocation failed for 3A debug exif params");
        return NO_MEMORY;
    }

    memset(jpg_job.encode_job.cam_exif_params.debug_params, 0,
            sizeof(mm_jpeg_debug_exif_params_t));
    exif_debug_params = jpg_job.encode_job.cam_exif_params.debug_params;

    jpg_job.encode_job.mobicat_mask = hal_obj->getMobicatMask();

    if (metadata != NULL) {
        // Fill in the metadata passed as parameter
        jpg_job.encode_job.p_metadata = metadata;

        // Fill in exif debug data
        if (exif_debug_params) {
            // AE
            IF_META_AVAILABLE(cam_ae_exif_debug_t, ae_exif_debug_params,
                    CAM_INTF_META_EXIF_DEBUG_AE, metadata) {
                memcpy(&exif_debug_params->ae_debug_params, ae_exif_debug_params,
                        sizeof(cam_ae_exif_debug_t));
                memcpy(&jpg_job.encode_job.p_metadata->statsdebug_ae_data,
                        ae_exif_debug_params, sizeof(cam_ae_exif_debug_t));
                exif_debug_params->ae_debug_params_valid = TRUE;
                jpg_job.encode_job.p_metadata->is_statsdebug_ae_params_valid = TRUE;
            }
            // AWB
            IF_META_AVAILABLE(cam_awb_exif_debug_t, awb_exif_debug_params,
                    CAM_INTF_META_EXIF_DEBUG_AWB, metadata) {
                memcpy(&exif_debug_params->awb_debug_params, awb_exif_debug_params,
                        sizeof(cam_awb_exif_debug_t));
                memcpy(&jpg_job.encode_job.p_metadata->statsdebug_awb_data,
                        awb_exif_debug_params, sizeof(cam_awb_exif_debug_t));
                exif_debug_params->awb_debug_params_valid = TRUE;
                jpg_job.encode_job.p_metadata->is_statsdebug_awb_params_valid = TRUE;
            }
            // AF
            IF_META_AVAILABLE(cam_af_exif_debug_t, af_exif_debug_params,
                    CAM_INTF_META_EXIF_DEBUG_AF, metadata) {
                memcpy(&exif_debug_params->af_debug_params, af_exif_debug_params,
                        sizeof(cam_af_exif_debug_t));
                memcpy(&jpg_job.encode_job.p_metadata->statsdebug_af_data,
                        af_exif_debug_params, sizeof(cam_af_exif_debug_t));
                exif_debug_params->af_debug_params_valid = TRUE;
                jpg_job.encode_job.p_metadata->is_statsdebug_af_params_valid = TRUE;
            }
            // ASD
            IF_META_AVAILABLE(cam_asd_exif_debug_t, asd_exif_debug_params,
                    CAM_INTF_META_EXIF_DEBUG_ASD, metadata) {
                memcpy(&exif_debug_params->asd_debug_params, asd_exif_debug_params,
                        sizeof(cam_asd_exif_debug_t));
                memcpy(&jpg_job.encode_job.p_metadata->statsdebug_asd_data,
                        asd_exif_debug_params, sizeof(cam_asd_exif_debug_t));
                exif_debug_params->asd_debug_params_valid = TRUE;
                jpg_job.encode_job.p_metadata->is_statsdebug_asd_params_valid = TRUE;
            }
            // STATS
            IF_META_AVAILABLE(cam_stats_buffer_exif_debug_t, stats_exif_debug_params,
                    CAM_INTF_META_EXIF_DEBUG_STATS, metadata) {
                memcpy(&exif_debug_params->stats_debug_params, stats_exif_debug_params,
                        sizeof(cam_stats_buffer_exif_debug_t));
                memcpy(&jpg_job.encode_job.p_metadata->statsdebug_stats_buffer_data,
                        stats_exif_debug_params, sizeof(cam_stats_buffer_exif_debug_t));
                exif_debug_params->stats_debug_params_valid = TRUE;
                jpg_job.encode_job.p_metadata->is_statsdebug_stats_params_valid = TRUE;
            }
            // BE STATS
            IF_META_AVAILABLE(cam_bestats_buffer_exif_debug_t, bestats_exif_debug_params,
                    CAM_INTF_META_EXIF_DEBUG_BESTATS, metadata) {
                memcpy(&exif_debug_params->bestats_debug_params, bestats_exif_debug_params,
                        sizeof(cam_bestats_buffer_exif_debug_t));
                memcpy(&jpg_job.encode_job.p_metadata->statsdebug_bestats_buffer_data,
                        bestats_exif_debug_params, sizeof(cam_bestats_buffer_exif_debug_t));
                exif_debug_params->bestats_debug_params_valid = TRUE;
                jpg_job.encode_job.p_metadata->is_statsdebug_bestats_params_valid = TRUE;
            }
            // BHIST
            IF_META_AVAILABLE(cam_bhist_buffer_exif_debug_t, bhist_exif_debug_params,
                    CAM_INTF_META_EXIF_DEBUG_BHIST, metadata) {
                memcpy(&exif_debug_params->bhist_debug_params, bhist_exif_debug_params,
                        sizeof(cam_bhist_buffer_exif_debug_t));
                memcpy(&jpg_job.encode_job.p_metadata->statsdebug_bhist_data,
                        bhist_exif_debug_params, sizeof(cam_bhist_buffer_exif_debug_t));
                exif_debug_params->bhist_debug_params_valid = TRUE;
                jpg_job.encode_job.p_metadata->is_statsdebug_bhist_params_valid = TRUE;
            }
            // Q3A
            IF_META_AVAILABLE(cam_q3a_tuning_info_t, q3a_tuning_exif_debug_params,
                    CAM_INTF_META_EXIF_DEBUG_3A_TUNING, metadata) {
                memcpy(&exif_debug_params->q3a_tuning_debug_params, q3a_tuning_exif_debug_params,
                        sizeof(cam_q3a_tuning_info_t));
                memcpy(&jpg_job.encode_job.p_metadata->statsdebug_3a_tuning_data,
                        q3a_tuning_exif_debug_params, sizeof(cam_q3a_tuning_info_t));
                exif_debug_params->q3a_tuning_debug_params_valid = TRUE;
                jpg_job.encode_job.p_metadata->is_statsdebug_3a_tuning_params_valid = TRUE;
            }
        }
    } else {
       LOGW("Metadata is null");
    }

    // Multi image info
    if (hal_obj->isDeviceLinked() == TRUE) {
        jpg_job.encode_job.multi_image_info.type = MM_JPEG_TYPE_JPEG;
        jpg_job.encode_job.multi_image_info.num_of_images = 1;
        jpg_job.encode_job.multi_image_info.enable_metadata = 1;
        if (hal_obj->isMainCamera() == TRUE) {
            jpg_job.encode_job.multi_image_info.is_primary = 1;
        } else {
            jpg_job.encode_job.multi_image_info.is_primary = 0;
        }
    }

    jpg_job.encode_job.hal_version = CAM_HAL_V3;

    //Start jpeg encoding
    ret = mJpegHandle.start_job(&jpg_job, &jobId);
    if (jpg_job.encode_job.cam_exif_params.debug_params) {
        free(jpg_job.encode_job.cam_exif_params.debug_params);
    }
    if (ret == NO_ERROR) {
        // remember job info
        jpeg_job_data->jobId = jobId;
    }

    LOGD("X");
    return ret;
}

/*===========================================================================
 * FUNCTION   : encodeData
 *
 * DESCRIPTION: function to prepare encoding job information and send to
 *              mm-jpeg-interface to do the encoding job
 *
 * PARAMETERS :
 *   @jpeg_job_data : ptr to a struct saving job related information
 *   @needNewSess   : flag to indicate if a new jpeg encoding session need
 *                    to be created. After creation, this flag will be toggled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PostProcessor::encodeData(qcamera_hal3_jpeg_data_t *jpeg_job_data,
                          uint8_t &needNewSess)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PPROC_ENCODEDATA);
    LOGD("E");
    int32_t ret = NO_ERROR;
    mm_jpeg_job_t jpg_job;
    uint32_t jobId = 0;
    QCamera3Stream *main_stream = NULL;
    mm_camera_buf_def_t *main_frame = NULL;
    cam_stream_parm_buffer_t param;
    QCamera3Channel *srcChannel = NULL;
    mm_camera_super_buf_t *recvd_frame = NULL;
    metadata_buffer_t *metadata = NULL;
    jpeg_settings_t *jpeg_settings = NULL;
    QCamera3HardwareInterface* hal_obj = NULL;
    mm_jpeg_debug_exif_params_t *exif_debug_params = NULL;
    if (m_parent != NULL) {
        hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
        if (hal_obj == NULL) {
            LOGE("hal_obj is NULL, Error");
            return BAD_VALUE;
        }
    } else {
        LOGE("m_parent is NULL, Error");
        return BAD_VALUE;
    }
    bool needJpegExifRotation = false;

    recvd_frame = jpeg_job_data->src_frame;
    metadata = jpeg_job_data->metadata;
    jpeg_settings = jpeg_job_data->jpeg_settings;

    LOGD("encoding bufIndex: %u",
        jpeg_job_data->src_frame->bufs[0]->buf_idx);

    QCamera3Channel *pChannel = NULL;
    // first check picture channel
    if (m_parent->getMyHandle() == recvd_frame->ch_id) {
        pChannel = m_parent;
    }
    // check reprocess channel if not found
    if (pChannel == NULL) {
        for (int8_t i = 0; i < m_ppChannelCnt; i++) {
            if (m_pReprocChannel[i] != NULL &&
                m_pReprocChannel[i]->getMyHandle() == recvd_frame->ch_id) {
                pChannel = m_pReprocChannel[i];
                break;
            }
        }
    }

    srcChannel = pChannel;

    if (srcChannel == NULL) {
        LOGE("No corresponding channel (ch_id = %d) exist, return here",
               recvd_frame->ch_id);
        return BAD_VALUE;
    }

    // find snapshot frame and thumnail frame
    //Note: In this version we will receive only snapshot frame.
    for (uint32_t i = 0; i < recvd_frame->num_bufs; i++) {
        QCamera3Stream *srcStream = NULL;
        srcStream = srcChannel->getStreamByHandle(recvd_frame->bufs[i]->stream_id);
        if (srcStream != NULL) {
            switch (srcStream->getMyType()) {
            case CAM_STREAM_TYPE_SNAPSHOT:
            case CAM_STREAM_TYPE_OFFLINE_PROC:
                main_stream = srcStream;
                main_frame = recvd_frame->bufs[i];
                break;
            default:
                break;
            }
        }
    }

    if(NULL == main_frame){
       LOGE("Main frame is NULL");
       return BAD_VALUE;
    }

    if (!jpeg_job_data->halPPAllocatedBuf) {
        QCamera3StreamMem *memObj = (QCamera3StreamMem *)main_frame->mem_info;
        if (NULL == memObj) {
            LOGE("Memeory Obj of main frame is NULL");
            return NO_MEMORY;
        }

        // clean and invalidate cache ops through mem obj of the frame
        memObj->cleanInvalidateCache(main_frame->buf_idx);
    }

    if (mJpegClientHandle <= 0) {
        LOGE("Error: bug here, mJpegClientHandle is 0");
        return UNKNOWN_ERROR;
    }
    cam_dimension_t src_dim;
    memset(&src_dim, 0, sizeof(cam_dimension_t));
    main_stream->getFrameDimension(src_dim);

    cam_dimension_t dst_dim;
    memset(&dst_dim, 0, sizeof(cam_dimension_t));

    //For depth images calculate src and des dim from getDepthMapSize.
    if(jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_DEPTH)
    {
#ifdef ENABLE_QC_BOKEH
        qrcp::getDepthMapSize(src_dim.width, src_dim.height,
                            src_dim.width, src_dim.height);
#endif //ENABLE_QC_BOKEH
        dst_dim = src_dim;
    } else if(jpeg_settings->is_dim_valid){
        dst_dim = jpeg_settings->output_dim;
    } else {
        if (NO_ERROR != m_parent->getStreamSize(dst_dim)) {
            LOGE("Failed to get size of the JPEG stream");
            return UNKNOWN_ERROR;
        }
    }

    needJpegExifRotation = hal_obj->needJpegExifRotation();
    IF_META_AVAILABLE(cam_rotation_info_t, rotation_info, CAM_INTF_PARM_ROTATION, metadata) {
        if (jpeg_settings->jpeg_orientation != 0 && rotation_info->rotation == ROTATE_0) {
            needJpegExifRotation = TRUE;
            LOGH("Need EXIF JPEG ROTATION");
        }
    }

    // Although in HAL3, legacy flip mode is not advertised
    // default value of CAM_INTF_PARM_FLIP is still added here
    // for jpge metadata
    int32_t flipMode = 0; // no flip
    ADD_SET_PARAM_ENTRY_TO_BATCH(metadata, CAM_INTF_PARM_FLIP, flipMode);

    LOGH("Need new session?:%d jpeg_orientation %d needJpegExifRotation %d useExifRotation %d",
            needNewSess, jpeg_settings->jpeg_orientation, needJpegExifRotation,
            hal_obj->useExifRotation());
    if (needNewSess) {
        //creating a new session, so we must destroy the old one
        if ( 0 < mJpegSessionId ) {
            ret = mJpegHandle.destroy_session(mJpegSessionId);
            if (ret != NO_ERROR) {
                LOGE("Error destroying an old jpeg encoding session, id = %d",
                       mJpegSessionId);
                return ret;
            }
            mJpegSessionId = 0;
        }
        // create jpeg encoding session
        mm_jpeg_encode_params_t encodeParam;
        memset(&encodeParam, 0, sizeof(mm_jpeg_encode_params_t));
        if(jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_DEPTH ||
             jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_BOKEH)
        {
            getJpegEncodeConfig(encodeParam, main_stream, jpeg_settings, recvd_frame->bufs[0]);
        } else {
            getJpegEncodeConfig(encodeParam, main_stream, jpeg_settings);
        }
        LOGH("#src bufs:%d # tmb bufs:%d #dst_bufs:%d",
                     encodeParam.num_src_bufs,encodeParam.num_tmb_bufs,encodeParam.num_dst_bufs);
        if (!needJpegExifRotation &&
            (jpeg_settings->jpeg_orientation == 90 ||
            jpeg_settings->jpeg_orientation == 270)) {
           //swap src width and height, stride and scanline due to rotation
           encodeParam.main_dim.src_dim.width = src_dim.height;
           encodeParam.main_dim.src_dim.height = src_dim.width;
           encodeParam.thumb_dim.src_dim.width = src_dim.height;
           encodeParam.thumb_dim.src_dim.height = src_dim.width;

           int32_t temp = encodeParam.src_main_buf[0].offset.mp[0].stride;
           encodeParam.src_main_buf[0].offset.mp[0].stride =
              encodeParam.src_main_buf[0].offset.mp[0].scanline;
           encodeParam.src_main_buf[0].offset.mp[0].scanline = temp;

           temp = encodeParam.src_thumb_buf[0].offset.mp[0].stride;
           encodeParam.src_thumb_buf[0].offset.mp[0].stride =
              encodeParam.src_thumb_buf[0].offset.mp[0].scanline;
           encodeParam.src_thumb_buf[0].offset.mp[0].scanline = temp;
        } else {
           encodeParam.main_dim.src_dim  = src_dim;
           encodeParam.thumb_dim.src_dim = src_dim;
        }
        encodeParam.main_dim.dst_dim = dst_dim;
        encodeParam.thumb_dim.dst_dim = jpeg_settings->thumbnail_size;

        if (!hal_obj->useExifRotation() && needJpegExifRotation) {
            encodeParam.rotation = jpeg_settings->jpeg_orientation;
            encodeParam.thumb_rotation = jpeg_settings->jpeg_orientation;
        }

        LOGI("Src Buffer cnt = %d, res = %dX%d len = %d rot = %d "
            "src_dim = %dX%d dst_dim = %dX%d",
            encodeParam.num_src_bufs,
            encodeParam.src_main_buf[0].offset.mp[0].stride,
            encodeParam.src_main_buf[0].offset.mp[0].scanline,
            encodeParam.src_main_buf[0].offset.frame_len,
            encodeParam.rotation,
            src_dim.width, src_dim.height,
            dst_dim.width, dst_dim.height);
        LOGI("Src THUMB buf_cnt = %d, res = %dX%d len = %d rot = %d "
            "src_dim = %dX%d, dst_dim = %dX%d",
            encodeParam.num_tmb_bufs,
            encodeParam.src_thumb_buf[0].offset.mp[0].stride,
            encodeParam.src_thumb_buf[0].offset.mp[0].scanline,
            encodeParam.src_thumb_buf[0].offset.frame_len,
            encodeParam.thumb_rotation,
            encodeParam.thumb_dim.src_dim.width,
            encodeParam.thumb_dim.src_dim.height,
            encodeParam.thumb_dim.dst_dim.width,
            encodeParam.thumb_dim.dst_dim.height);
        ret = mJpegHandle.create_session(mJpegClientHandle, &encodeParam, &mJpegSessionId);
        if (ret != NO_ERROR) {
            LOGE("Error creating a new jpeg encoding session, ret = %d", ret);
            return ret;
        }
        needNewSess = FALSE;
    }

    // Fill in new job
    memset(&jpg_job, 0, sizeof(mm_jpeg_job_t));
    jpg_job.job_type = JPEG_JOB_TYPE_ENCODE;
    jpg_job.encode_job.session_id = mJpegSessionId;
    jpg_job.encode_job.src_index = (int32_t)main_frame->buf_idx;
    jpg_job.encode_job.dst_index = 0;

    cam_rect_t crop;
    memset(&crop, 0, sizeof(cam_rect_t));
    //TBD_later - Zoom event removed in stream
    //main_stream->getCropInfo(crop);
    if(jpeg_settings->is_crop_valid)
    {
        crop = jpeg_settings->crop;
    }else {
        crop.left = 0;
        crop.top = 0;
        crop.height = src_dim.height;
        crop.width = src_dim.width;
    }

    if (jpeg_settings->hdr_snapshot) {
       memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
       param.type = CAM_STREAM_PARAM_TYPE_GET_OUTPUT_CROP;
       ret = main_stream->getParameter(param);
       if (ret != NO_ERROR) {
          LOGE("%s: stream getParameter for reprocess failed", __func__);
       } else {
           for (int i = 0; i < param.outputCrop.num_of_streams; i++) {
              if (param.outputCrop.crop_info[i].stream_id
                  == main_stream->getMyServerID()) {
                     crop = param.outputCrop.crop_info[i].crop;
                     main_stream->setCropInfo(crop);
              }
           }
         }
    }
    // Set main dim job parameters and handle rotation
    if (!needJpegExifRotation && (jpeg_settings->jpeg_orientation == 90 ||
            jpeg_settings->jpeg_orientation == 270)) {

        jpg_job.encode_job.main_dim.src_dim.width = src_dim.height;
        jpg_job.encode_job.main_dim.src_dim.height = src_dim.width;

        jpg_job.encode_job.main_dim.dst_dim.width = dst_dim.height;
        jpg_job.encode_job.main_dim.dst_dim.height = dst_dim.width;

        jpg_job.encode_job.main_dim.crop.width = crop.height;
        jpg_job.encode_job.main_dim.crop.height = crop.width;
        jpg_job.encode_job.main_dim.crop.left = crop.top;
        jpg_job.encode_job.main_dim.crop.top = crop.left;
    } else {
        jpg_job.encode_job.main_dim.src_dim = src_dim;
        jpg_job.encode_job.main_dim.dst_dim = dst_dim;
        jpg_job.encode_job.main_dim.crop = crop;
    }

    // get 3a sw version info
    cam_q3a_version_t sw_version;
    memset(&sw_version, 0, sizeof(sw_version));

    if (hal_obj)
        hal_obj->get3AVersion(sw_version);

    // get exif data
    QCamera3Exif *pJpegExifObj = getExifData(metadata, jpeg_settings,
            (needJpegExifRotation && hal_obj->useExifRotation()));
    jpeg_job_data->pJpegExifObj = pJpegExifObj;
    if (pJpegExifObj != NULL) {
        jpg_job.encode_job.exif_info.exif_data = pJpegExifObj->getEntries();
        jpg_job.encode_job.exif_info.numOfEntries =
            pJpegExifObj->getNumOfEntries();
        jpg_job.encode_job.exif_info.debug_data.sw_3a_version[0] =
            sw_version.major_version;
        jpg_job.encode_job.exif_info.debug_data.sw_3a_version[1] =
            sw_version.minor_version;
        jpg_job.encode_job.exif_info.debug_data.sw_3a_version[2] =
            sw_version.patch_version;
        jpg_job.encode_job.exif_info.debug_data.sw_3a_version[3] =
            sw_version.new_feature_des;
    }

    if (!hal_obj->useExifRotation() && needJpegExifRotation) {
        jpg_job.encode_job.rotation= jpeg_settings->jpeg_orientation;
    }

    // thumbnail dim
    LOGH("Thumbnail needed:%d", m_bThumbnailNeeded);
    if (m_bThumbnailNeeded == TRUE) {
        jpg_job.encode_job.thumb_dim.dst_dim =
                jpeg_settings->thumbnail_size;

      if (!needJpegExifRotation &&
          (jpeg_settings->jpeg_orientation  == 90 ||
           jpeg_settings->jpeg_orientation == 270)) {
            //swap the thumbnail destination width and height if it has
            //already been rotated
            int temp = jpg_job.encode_job.thumb_dim.dst_dim.width;
            jpg_job.encode_job.thumb_dim.dst_dim.width =
                    jpg_job.encode_job.thumb_dim.dst_dim.height;
            jpg_job.encode_job.thumb_dim.dst_dim.height = temp;

            jpg_job.encode_job.thumb_dim.src_dim.width = src_dim.height;
            jpg_job.encode_job.thumb_dim.src_dim.height = src_dim.width;

            jpg_job.encode_job.thumb_dim.crop.width = crop.height;
            jpg_job.encode_job.thumb_dim.crop.height = crop.width;
            jpg_job.encode_job.thumb_dim.crop.left = crop.top;
            jpg_job.encode_job.thumb_dim.crop.top = crop.left;
        } else {
           jpg_job.encode_job.thumb_dim.src_dim = src_dim;
           jpg_job.encode_job.thumb_dim.crop = crop;
        }
        jpg_job.encode_job.thumb_index = main_frame->buf_idx;
        LOGI("Thumbnail idx = %d src w/h (%dx%d), dst w/h (%dx%d)",
                jpg_job.encode_job.thumb_index,
                jpg_job.encode_job.thumb_dim.src_dim.width,
                jpg_job.encode_job.thumb_dim.src_dim.height,
                jpg_job.encode_job.thumb_dim.dst_dim.width,
                jpg_job.encode_job.thumb_dim.dst_dim.height);
    }
    LOGI("Main image idx = %d src w/h (%dx%d), dst w/h (%dx%d) rot = %d"
            "crop t/lt (%dx%d) wxh (%dx%d)",
            jpg_job.encode_job.src_index,
            jpg_job.encode_job.main_dim.src_dim.width,
            jpg_job.encode_job.main_dim.src_dim.height,
            jpg_job.encode_job.main_dim.dst_dim.width,
            jpg_job.encode_job.main_dim.dst_dim.height,
            jpg_job.encode_job.rotation,
            jpg_job.encode_job.main_dim.crop.top,
            jpg_job.encode_job.main_dim.crop.left,
            jpg_job.encode_job.main_dim.crop.width,
            jpg_job.encode_job.main_dim.crop.height);

    jpg_job.encode_job.cam_exif_params = hal_obj->get3AExifParams();
    exif_debug_params = jpg_job.encode_job.cam_exif_params.debug_params;

    // Allocate for a local copy of debug parameters
    jpg_job.encode_job.cam_exif_params.debug_params =
            (mm_jpeg_debug_exif_params_t *) malloc (sizeof(mm_jpeg_debug_exif_params_t));
    if (!jpg_job.encode_job.cam_exif_params.debug_params) {
        LOGE("Out of Memory. Allocation failed for 3A debug exif params");
        return NO_MEMORY;
    }

    jpg_job.encode_job.mobicat_mask = hal_obj->getMobicatMask();

    if (metadata != NULL) {
       //Fill in the metadata passed as parameter
       jpg_job.encode_job.p_metadata = metadata;

       jpg_job.encode_job.p_metadata->is_mobicat_aec_params_valid =
                jpg_job.encode_job.cam_exif_params.cam_3a_params_valid;

       if (jpg_job.encode_job.cam_exif_params.cam_3a_params_valid) {
            jpg_job.encode_job.p_metadata->mobicat_aec_params =
                jpg_job.encode_job.cam_exif_params.cam_3a_params;
       }

       if (exif_debug_params) {
            // Copy debug parameters locally.
           memcpy(jpg_job.encode_job.cam_exif_params.debug_params,
                   exif_debug_params, (sizeof(mm_jpeg_debug_exif_params_t)));
           /* Save a copy of 3A debug params */
            jpg_job.encode_job.p_metadata->is_statsdebug_ae_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->ae_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_awb_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->awb_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_af_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->af_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_asd_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->asd_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_stats_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->stats_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_bestats_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->bestats_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_bhist_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->bhist_debug_params_valid;
            jpg_job.encode_job.p_metadata->is_statsdebug_3a_tuning_params_valid =
                    jpg_job.encode_job.cam_exif_params.debug_params->q3a_tuning_debug_params_valid;

            if (jpg_job.encode_job.cam_exif_params.debug_params->ae_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_ae_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->ae_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->awb_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_awb_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->awb_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->af_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_af_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->af_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->asd_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_asd_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->asd_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->stats_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_stats_buffer_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->stats_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->bestats_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_bestats_buffer_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->bestats_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->bhist_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_bhist_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->bhist_debug_params;
            }
            if (jpg_job.encode_job.cam_exif_params.debug_params->q3a_tuning_debug_params_valid) {
                jpg_job.encode_job.p_metadata->statsdebug_3a_tuning_data =
                        jpg_job.encode_job.cam_exif_params.debug_params->q3a_tuning_debug_params;
            }
        }
    } else {
       LOGW("Metadata is null");
    }

    // Multi image info
    if ((hal_obj->isDeviceLinked() == TRUE) || (jpeg_settings->encode_type == MM_JPEG_TYPE_MPO )) {
        jpg_job.encode_job.multi_image_info.type = jpeg_settings->encode_type;
        jpg_job.encode_job.multi_image_info.num_of_images = 
                                        (jpeg_settings->encode_type == MM_JPEG_TYPE_MPO)? 3 : 1;
        jpg_job.encode_job.multi_image_info.enable_metadata = 0;
        if (hal_obj->isMainCamera() == TRUE && (jpeg_settings->encode_type == MM_JPEG_TYPE_MPO ?
                    jpeg_settings->image_type == CAM_HAL3_JPEG_TYPE_BOKEH : false)) {
            jpg_job.encode_job.multi_image_info.is_primary = 1;
        } else {
            jpg_job.encode_job.multi_image_info.is_primary = 0;
        }
    }

    jpg_job.encode_job.hal_version = CAM_HAL_V3;

    //Start jpeg encoding
    ret = mJpegHandle.start_job(&jpg_job, &jobId);
    if (jpg_job.encode_job.cam_exif_params.debug_params) {
        free(jpg_job.encode_job.cam_exif_params.debug_params);
    }
    if (ret == NO_ERROR) {
        // remember job info
        jpeg_job_data->jobId = jobId;
    }

    LOGD("X");
    return ret;
}

/*===========================================================================
 * FUNCTION   : doNextJob
 *
 * DESCRIPTION: send DO_NEXT_JOB command to dataProc thread.
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
 void QCamera3PostProcessor::doNextJob()
{
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);
}

/*===========================================================================
 * FUNCTION   : composeMpo
 *
 * DESCRIPTION: compress jpeg images to mpo format. Use jpeg interface handle
 *              to compose mpo image. Sends callback to pic channel (mpoevthandle)
 *              with composed mpo.
 *
 * PARAMETERS :
 *   @mpo_info : contain list of input images and output buffer.
 *   @userdata : need to pass with mpo callback.
 *
 * RETURN     : NO_ERROR on success else -1.
 *==========================================================================*/
int32_t QCamera3PostProcessor::composeMpo(qcamera_hal3_mpo_compose_info_t &mpo_info,
                                                                         void *userdata)
{
    LOGH("E");
    if((mpo_info.num_of_aux_image + 1) > MM_JPEG_MAX_MPO_IMAGES)
    {
        LOGE("Error: cannot composeMpo of %d images",mpo_info.num_of_aux_image + 1);
        return -1;
    }

    ssize_t output_size = 0;
    mm_jpeg_mpo_info_t mpo_compose_info;
    mpo_compose_info.primary_image = mpo_info.main_image;   //first images will be the primary image
    output_size += mpo_info.main_image.buf_filled_len;

    for(uint32_t i = 0; i < mpo_info.num_of_aux_image; i++)
    {
        mpo_compose_info.aux_images[i] = mpo_info.aux_images[i];
        output_size += mpo_info.aux_images[i].buf_filled_len;
    }
    mpo_compose_info.num_of_images = mpo_info.num_of_aux_image + 1 /*num of main image*/;
    mpo_compose_info.output_buff = mpo_info.output;
    mpo_compose_info.output_buff.buf_filled_len = 0;
    mpo_compose_info.output_buff_size = output_size;

    int rc = mMpoHandle.compose_mpo(&mpo_compose_info);
    if(rc != 0)
    {
        LOGE("Error: failed to compose mpo image");
        return -1;
    }

    mMpoCB( JPEG_JOB_STATUS_DONE, &mpo_compose_info.output_buff, userdata);

    LOGH("X");
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : dataProcessRoutine
 *
 * DESCRIPTION: data process routine that handles input data either from input
 *              Jpeg Queue to do jpeg encoding, or from input PP Queue to do
 *              reprocess.
 *
 * PARAMETERS :
 *   @data    : user data ptr (QCamera3PostProcessor)
 *
 * RETURN     : None
 *==========================================================================*/
void *QCamera3PostProcessor::dataProcessRoutine(void *data)
{
    int running = 1;
    int ret;
    uint8_t is_active = FALSE;
    uint8_t needNewSess = TRUE;
    mm_camera_super_buf_t *meta_buffer = NULL;
    LOGD("E");
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)data;
    QCameraCmdThread *cmdThread = &pme->m_dataProcTh;
    cmdThread->setName("cam_data_proc");

    do {
        do {
            ret = cam_sem_wait(&cmdThread->cmd_sem);
            if (ret != 0 && errno != EINVAL) {
                LOGE("cam_sem_wait error (%s)",
                            strerror(errno));
                return NULL;
            }
        } while (ret != 0);

        // we got notified about new cmd avail in cmd queue
        camera_cmd_type_t cmd = cmdThread->getCmd();
        switch (cmd) {
        case CAMERA_CMD_TYPE_START_DATA_PROC:
            LOGH("start data proc");
            is_active = TRUE;
            needNewSess = TRUE;

            pme->m_ongoingPPQ.init();
            pme->m_inputJpegQ.init();
            pme->m_inputPPQ.init();
            pme->m_inputFWKPPQ.init();
            pme->m_inputMultiReprocQ.init();
            pme->m_inputMetaQ.init();
            pme->m_jpegSettingsQ.init();
            cam_sem_post(&cmdThread->sync_sem);

            break;
        case CAMERA_CMD_TYPE_STOP_DATA_PROC:
            {
                LOGH("stop data proc");
                is_active = FALSE;
                // cancel all ongoing jpeg jobs
                qcamera_hal3_jpeg_data_t *jpeg_job =
                    (qcamera_hal3_jpeg_data_t *)pme->m_ongoingJpegQ.dequeue();
                while (jpeg_job != NULL) {
                    pme->mJpegHandle.abort_job(jpeg_job->jobId);

                    pme->releaseJpegJobData(jpeg_job);
                    free(jpeg_job);

                    jpeg_job = (qcamera_hal3_jpeg_data_t *)pme->m_ongoingJpegQ.dequeue();
                }

                // destroy jpeg encoding session
                if ( 0 < pme->mJpegSessionId ) {
                    pme->mJpegHandle.destroy_session(pme->mJpegSessionId);
                    pme->mJpegSessionId = 0;
                }

                needNewSess = TRUE;

                // flush ongoing postproc Queue
                pme->m_ongoingPPQ.flush();

                // flush input jpeg Queue
                pme->m_inputJpegQ.flush();

                // flush input Postproc Queue
                pme->m_inputPPQ.flush();

                // flush framework input Postproc Queue
                pme->m_inputFWKPPQ.flush();

                pme->m_inputMultiReprocQ.flush();

                pme->m_inputMetaQ.flush();
                pme->m_jpegSettingsQ.flush();

                // signal cmd is completed
                cam_sem_post(&cmdThread->sync_sem);
                pthread_mutex_lock(&pme->mHDRJobLock);
                pme->mChannelStop = true;
                pthread_cond_signal(&pme->mProcChStopCond);
                pthread_mutex_unlock(&pme->mHDRJobLock);
            }
            break;
        case CAMERA_CMD_TYPE_DO_NEXT_JOB:
            {
                LOGH("Do next job, active is %d", is_active);
                /* needNewSess is set to TRUE as postproc is not re-STARTed
                 * anymore for every captureRequest */
                needNewSess = TRUE;
                if (is_active == TRUE) {
                    // check if there is any ongoing jpeg jobs
                    if (pme->m_ongoingJpegQ.isEmpty()) {
                        LOGD("ongoing jpeg queue is empty so doing the jpeg job");
                        // no ongoing jpeg job, we are fine to send jpeg encoding job
                        qcamera_hal3_jpeg_data_t *jpeg_job =
                            (qcamera_hal3_jpeg_data_t *)pme->m_inputJpegQ.dequeue();

                        if (NULL != jpeg_job) {
                            // add into ongoing jpeg job Q
                            pme->m_ongoingJpegQ.enqueue((void *)jpeg_job);

                            if (jpeg_job->fwk_frame) {
                                ret = pme->encodeFWKData(jpeg_job, needNewSess);
                            } else {
                                ret = pme->encodeData(jpeg_job, needNewSess);
                            }
                            if (NO_ERROR != ret) {
                                // dequeue the last one
                                pme->m_ongoingJpegQ.dequeue(false);

                                pme->releaseJpegJobData(jpeg_job);
                                free(jpeg_job);
                            }
                        }
                    }

                    if (!pme->m_inputMultiReprocQ.isEmpty()) {
                        QCamera3HardwareInterface* hal_obj =
                            (QCamera3HardwareInterface *)pme->m_parent->mUserData;
                        qcamera_hal3_pp_data_t *pp_job =
                            (qcamera_hal3_pp_data_t *)pme->m_inputMultiReprocQ.dequeue();
                        if (pp_job != NULL) {
                            LOGH("multi reproc Q is not empty, pp channel idx:%d, total pp cnt:%d",
                                pp_job->pp_ch_idx, pme->m_ppChannelCnt);
                            if (pp_job->pp_ch_idx < pme->m_ppChannelCnt &&
                                pme->m_pReprocChannel[pp_job->pp_ch_idx] != NULL) {
                                LOGH("do reproc on %dth reprocess channel", pp_job->pp_ch_idx + 1);

                                qcamera_fwk_input_pp_data_t fwk_frame;
                                memset(&fwk_frame, 0, sizeof(qcamera_fwk_input_pp_data_t));
                                if (pp_job->fwk_src_frame != NULL) {
                                    LOGD("reprocess for fwk input frame");
                                    fwk_frame = *(pp_job->fwk_src_frame);
                                    fwk_frame.input_buffer
                                        = *(pp_job->reprocessed_src_frame->bufs[0]);
                                } else {
                                    fwk_frame.frameNumber = pp_job->frameNumber;
                                    fwk_frame.input_buffer
                                        = *(pp_job->reprocessed_src_frame->bufs[0]);
                                    fwk_frame.metadata_buffer = *(pp_job->src_metadata->bufs[0]);

                                    uint32_t stream_id = hal_obj->mQCFARawChannel->getStreamSvrId();
                                    LOGD("src stream server id:%d", stream_id);
                                    ret = pme->m_pReprocChannel[pp_job->pp_ch_idx]->overrideMetadata(
                                            (metadata_buffer_t *)fwk_frame.metadata_buffer.buffer,
                                            pp_job->jpeg_settings, stream_id);
                                    if (ret != NO_ERROR) {
                                        LOGE("fail to override metadata.");
                                    }
                                }
                                LOGD("frame number: %d", fwk_frame.frameNumber);
                                pme->m_ongoingPPQ.enqueue((void *)pp_job);
                                ret = pme->m_pReprocChannel[pp_job->pp_ch_idx]->doReprocessOffline(
                                    &fwk_frame, true);
                                if (ret != NO_ERROR) {
                                    pme->m_ongoingPPQ.dequeue(false);
                                    LOGE("fail to do offline reprocess");
                                }
                            }
                        } else {
                            LOGE("fail to dequeue pp job!");
                        }
                    }

                    // check if there are any framework pp jobs
                    if (!pme->m_inputFWKPPQ.isEmpty()) {
                        qcamera_fwk_input_pp_data_t *fwk_frame =
                                (qcamera_fwk_input_pp_data_t *) pme->m_inputFWKPPQ.dequeue();
                        if (NULL != fwk_frame) {
                            qcamera_hal3_pp_data_t *pp_job =
                                (qcamera_hal3_pp_data_t *)malloc(sizeof(qcamera_hal3_pp_data_t));
                            jpeg_settings_t *jpeg_settings =
                                (jpeg_settings_t *)pme->m_jpegSettingsQ.dequeue();
                            if (pp_job != NULL) {
                                memset(pp_job, 0, sizeof(qcamera_hal3_pp_data_t));
                                pp_job->jpeg_settings = jpeg_settings;
                                if (pme->m_pReprocChannel[0] != NULL) {
                                    if (NO_ERROR !=
                                        pme->m_pReprocChannel[0]->overrideFwkMetadata(fwk_frame)) {
                                        LOGE("Failed to extract output crop");
                                    }
                                    // add into ongoing PP job Q
                                    pp_job->fwk_src_frame = fwk_frame;
                                    pp_job->pp_ch_idx = 0;
                                    pp_job->frameNumber = fwk_frame->frameNumber;
                                    pme->m_ongoingPPQ.enqueue((void *)pp_job);
                                    ret = pme->m_pReprocChannel[0]->doReprocessOffline(fwk_frame);
                                    if (NO_ERROR != ret) {
                                        // remove from ongoing PP job Q
                                        pme->m_ongoingPPQ.dequeue(false);
                                    }
                                } else {
                                    LOGE("Reprocess channel is NULL");
                                    ret = -1;
                                }
                            } else {
                                LOGE("no mem for qcamera_hal3_pp_data_t");
                                ret = -1;
                            }

                            if (0 != ret) {
                                // free pp_job
                                if (pp_job != NULL) {
                                    free(pp_job);
                                }
                                // free frame
                                if (fwk_frame != NULL) {
                                    free(fwk_frame);
                                }
                            }
                        }
                    }

                    LOGH("dequeuing pp frame");
                    pthread_mutex_lock(&pme->mReprocJobLock);
                    if(pme->mReprocessNode.size()) {
                        List<ReprocessBuffer>::iterator reprocData;
                        reprocData = pme->mReprocessNode.begin();
                        qcamera_hal3_pp_buffer_t *pp_buffer = reprocData->reprocBuf;
                        qcamera_hal3_meta_pp_buffer_t *meta_pp_buffer = reprocData->metaBuffer;
                        pme->mReprocessNode.erase(pme->mReprocessNode.begin());
                        LOGD(" Reprocess Buffer Frame Number :%d  and %d",
                                pp_buffer->frameNumber, meta_pp_buffer->metaFrameNumber);
                        meta_buffer =
                            (mm_camera_super_buf_t *)meta_pp_buffer->metabuf;
                        jpeg_settings_t *jpeg_settings =
                           (jpeg_settings_t *)pme->m_jpegSettingsQ.dequeue();
                        jpeg_settings_t *ppOutput_jpeg_settings = NULL;

                        //In bokeh case, there will be no AUX image jpeg settings.
                        //DEPTH image jpeg_settings need to assign to ppOutPut_jpeg_settings.
                        //BOKEH image jpeg_settings need to assign to ppOutPut_jpeg_settings.
                        QCamera3HardwareInterface* hal_obj =
                                (QCamera3HardwareInterface*)pme->m_parent->mUserData;
                        if(hal_obj->isDualCamera() && jpeg_settings != NULL)
                        {
                            if((jpeg_settings->image_type != CAM_HAL3_JPEG_TYPE_MAIN))
                            {
                                ppOutput_jpeg_settings = jpeg_settings;
                                jpeg_settings = (jpeg_settings_t *)pme->m_jpegSettingsQ.dequeue();
                            } else {
                                ppOutput_jpeg_settings = (jpeg_settings_t *)
                                                                pme->m_jpegSettingsQ.dequeue();
                            }
                        }

                        pthread_mutex_unlock(&pme->mReprocJobLock);
                        qcamera_hal3_pp_data_t *pp_job =
                            (qcamera_hal3_pp_data_t *)malloc(sizeof(qcamera_hal3_pp_data_t));
                        if (pp_job == NULL) {
                            LOGE("no mem for qcamera_hal3_pp_data_t");
                            ret = -1;
                        } else if (meta_buffer == NULL) {
                            LOGE("failed to dequeue from m_inputMetaQ");
                            ret = -1;
                        } else if (pp_buffer == NULL) {
                            LOGE("failed to dequeue from m_inputPPQ");
                            ret = -1;
                        } else if (pp_buffer != NULL){
                            memset(pp_job, 0, sizeof(qcamera_hal3_pp_data_t));
                            pp_job->src_frame = pp_buffer->input;
                            pp_job->src_metadata = meta_buffer;
                            if (meta_buffer->bufs[0] != NULL) {
                                pp_job->metadata = (metadata_buffer_t *)
                                        meta_buffer->bufs[0]->buffer;
                            }
                            pp_job->jpeg_settings = jpeg_settings;
                            pp_job->ppOutput_jpeg_settings = ppOutput_jpeg_settings;

                            pp_job->pp_ch_idx = 0;
                            pp_job->frameNumber = pp_buffer->frameNumber;
                            pme->m_ongoingPPQ.enqueue((void *)pp_job);
                            if (pme->m_pReprocChannel[0] != NULL) {
                                mm_camera_buf_def_t *meta_buffer_arg = NULL;
                                meta_buffer_arg = meta_buffer->bufs[0];
                                qcamera_fwk_input_pp_data_t fwk_frame;
                                memset(&fwk_frame, 0, sizeof(qcamera_fwk_input_pp_data_t));
                                fwk_frame.frameNumber = pp_buffer->frameNumber;
                                if (pme->m_ppChannelCnt > 1) {
                                    LOGD("multi pass reprocess, no need override meta here.");
                                    fwk_frame.input_buffer = *(pp_buffer->input->bufs[0]);
                                    fwk_frame.metadata_buffer = *(meta_buffer->bufs[0]);
                                    fwk_frame.output_buffer = pp_buffer->output;
                                } else {
                                    if(pp_job->jpeg_settings != NULL)
                                    {
                                        ret = pme->m_pReprocChannel[0]->overrideMetadata(
                                                pp_buffer, meta_buffer_arg,
                                                pp_job->jpeg_settings,
                                                fwk_frame);
                                    } else {
                                        ret = pme->m_pReprocChannel[0]->overrideMetadata(
                                                pp_buffer, meta_buffer_arg,
                                                pp_job->ppOutput_jpeg_settings,
                                                fwk_frame);
                                    }
                                }
                                if (NO_ERROR == ret) {
                                    // add into ongoing PP job Q
                                    pme->mPerfLockMgr.acquirePerfLock(PERF_LOCK_OFFLINE_REPROC);
                                    ret = pme->m_pReprocChannel[0]->doReprocessOffline(
                                            &fwk_frame, true);
                                    pme->mPerfLockMgr.releasePerfLock(PERF_LOCK_OFFLINE_REPROC);
                                    if (NO_ERROR != ret) {
                                        // remove from ongoing PP job Q
                                        pme->m_ongoingPPQ.dequeue(false);
                                    }
                                }
                            } else {
                                LOGE("No reprocess. Calling processPPData directly");
                                ret = pme->processPPData(pp_buffer->input);
                            }
                        }

                        if (0 != ret) {
                            // free pp_job
                            if (pp_job != NULL) {
                                free(pp_job);
                            }
                            // free frame
                            if (pp_buffer != NULL) {
                                if (pp_buffer->input) {
                                    pme->releaseSuperBuf(pp_buffer->input);
                                    free(pp_buffer->input);
                                }
                                free(pp_buffer);
                            }
                            //free metadata
                            if (NULL != meta_buffer) {
                                pme->m_parent->metadataBufDone(meta_buffer);
                                free(meta_buffer);
                            }
                        } else {
                            if (pp_buffer != NULL) {
                                free(pp_buffer);
                            }
                        }
                    } else {
                        pthread_mutex_unlock(&pme->mReprocJobLock);
                    }
                } else {
                    // not active, simply return buf and do no op
                    qcamera_hal3_jpeg_data_t *jpeg_job =
                        (qcamera_hal3_jpeg_data_t *)pme->m_inputJpegQ.dequeue();
                    if (NULL != jpeg_job) {
                        free(jpeg_job);
                    }

                    qcamera_hal3_pp_buffer_t* pp_buf =
                            (qcamera_hal3_pp_buffer_t *)pme->m_inputPPQ.dequeue();
                    if (NULL != pp_buf) {
                        if (pp_buf->input) {
                            pme->releaseSuperBuf(pp_buf->input);
                            free(pp_buf->input);
                            pp_buf->input = NULL;
                        }
                        free(pp_buf);
                    }
                    mm_camera_super_buf_t *metadata = (mm_camera_super_buf_t *)
                                                            pme->m_inputMetaQ.dequeue();
                    if (metadata != NULL) {
                        pme->m_parent->metadataBufDone(metadata);
                        free(metadata);
                    }
                    qcamera_fwk_input_pp_data_t *fwk_frame =
                            (qcamera_fwk_input_pp_data_t *) pme->m_inputFWKPPQ.dequeue();
                    if (NULL != fwk_frame) {
                        free(fwk_frame);
                    }
                }
            }
            break;
        case CAMERA_CMD_TYPE_EXIT:
            running = 0;
            break;
        default:
            break;
        }
    } while (running);
    LOGD("X");
    return NULL;
}

/*===========================================================================
 * FUNCTION   : processHalPPDataCB
 *
 * DESCRIPTION: callback function to process frame after HAL PP block
 *
 * PARAMETERS :
 *   @pOutput     : output after HAL PP processed
 *   @pUserData   : user data ptr (QCameraReprocessor)
 *
 * RETURN     : None
 *==========================================================================*/
void QCamera3PostProcessor::processHalPPDataCB(qcamera_hal_pp_data_t *pOutput, void* pUserData)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)pUserData;
    pme->processHalPPData(pOutput);
}

/*===========================================================================
 * FUNCTION   : processHalPPData
 *
 * DESCRIPTION: process received frame after HAL PP block.
 *
 * PARAMETERS :
 *   @pData   : received qcamera_hal_pp_data_t data from HAL PP callback.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *
 * NOTE       : The frame after HAL PP need to send to jpeg encoding.
 *==========================================================================*/
int32_t QCamera3PostProcessor::processHalPPData(qcamera_hal_pp_data_t *pData)
{
    int32_t rc = NO_ERROR;
    LOGH("E");

    if (pData == NULL) {
        LOGE("HAL PP processed data is NULL");
        return BAD_VALUE;
    }
    mm_camera_super_buf_t *frame = pData->frame;
    if (frame == NULL) {
        LOGE("HAL PP processed frame is NULL");
        return BAD_VALUE;
    }
    // send to JPEG encoding
    qcamera_hal3_jpeg_data_t *jpeg_job =
                (qcamera_hal3_jpeg_data_t *)malloc(sizeof(qcamera_hal3_jpeg_data_t));
    if (jpeg_job == NULL) {
        LOGE("No memory for jpeg job");
        return NO_MEMORY;
    }

    memset(jpeg_job, 0, sizeof(qcamera_hal3_jpeg_data_t));
    jpeg_job->src_frame = frame;
    jpeg_job->src_reproc_frame = pData->src_reproc_frame;
    //for bokeh and Depth bufs[1] will be NULL.
    if(pData->metadata != NULL)
    {
        jpeg_job->metadata = pData->metadata;
    }
    else {
        jpeg_job->metadata = (metadata_buffer_t*) pData->bufs[1].buffer;
    }
    jpeg_job->src_metadata = pData->src_metadata;
    jpeg_job->jpeg_settings = pData->jpeg_settings;

    jpeg_job->halPPAllocatedBuf = pData->halPPAllocatedBuf;
    jpeg_job->hal_pp_bufs = pData->bufs;
    jpeg_job->snapshot_heap = pData->snapshot_heap;
    jpeg_job->metadata_heap = pData->metadata_heap;

    LOGD("halPPAllocatedBuf = %d needEncode %d", pData->halPPAllocatedBuf, pData->needEncode);

    if (!pData->halPPAllocatedBuf && !pData->needEncode) {
        // check if to encode hal pp input buffer
        char prop[PROPERTY_VALUE_MAX];
        memset(prop, 0, sizeof(prop));
        property_get("persist.vendor.camera.dualfov.jpegnum", prop, "1");
        int dualfov_snap_num = atoi(prop);
        if (dualfov_snap_num == 1) {
            LOGH("No need to encode input buffer, just release it.");
            releaseJpegJobData(jpeg_job);
            free(jpeg_job);
            jpeg_job = NULL;
            free(pData);
            return NO_ERROR;
        }
    }

    if (pData->is_dim_valid) {
        jpeg_job->jpeg_settings->is_dim_valid = true;
        jpeg_job->jpeg_settings->output_dim = pData->outputDim;
    }

    if (pData->is_offset_valid) {
        jpeg_job->jpeg_settings->is_offset_valid = true;
        jpeg_job->jpeg_settings->offset = pData->snap_offset;
    }

    if (pData->is_format_valid) {
        jpeg_job->jpeg_settings->is_format_valid = true;
        jpeg_job->jpeg_settings->format = pData->outputFormat;
    }

    if (pData->is_crop_valid) {
        jpeg_job->jpeg_settings->is_crop_valid = true;
        jpeg_job->jpeg_settings->crop = pData->outputCrop;
    }


    // Enqueue frame to jpeg input queue
    if (false == m_inputJpegQ.enqueue((void *)jpeg_job)) {
        LOGW("Input Jpeg Q is not active!!!");
        releaseJpegJobData(jpeg_job);
        free(jpeg_job);
        jpeg_job = NULL;
    }

    // wake up data proc thread
    LOGH("Send frame for jpeg encoding");
    m_dataProcTh.sendCmd(CAMERA_CMD_TYPE_DO_NEXT_JOB, FALSE, FALSE);

    free(pData);
    LOGH("X");
    return rc;
}

void QCamera3PostProcessor::createHalPPManager()
{
    LOGH("E");
    if (m_pHalPPManager == NULL) {
        m_pHalPPManager = QCameraHALPPManager::getInstance();
        LOGH("Created HAL PP manager");
    }
    LOGH("X");
    return;
}


/*===========================================================================
 * FUNCTION   : initHalPPManager
 *
 * DESCRIPTION: function to create and init HALPP manager
 * RETURN     : None
 *==========================================================================*/
int32_t QCamera3PostProcessor::initHalPPManager()
{
    int32_t rc = NO_ERROR;

    if (m_pHalPPManager == NULL) {
        LOGE("failed as PP manager is NULL");
        return BAD_VALUE;
    }

    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;

    void *staticParam = hal_obj->getCamHalCapabilities();;
    cam_hal_pp_type_t halPPType = hal_obj->getHalPPType();
    LOGH("E halPPType:%d mPProcType: %d", halPPType, m_pHalPPManager->getPprocType());
    if (m_pHalPPManager->getPprocType() != halPPType) {
        //HAL PP block might change, deinit and re init
        rc = m_pHalPPManager->deinit();
        if (rc != NO_ERROR) {
            LOGE("HAL PP type %d init failed, rc = %d", halPPType, rc);
            return rc;
        }
        rc = m_pHalPPManager->init(halPPType, QCamera3PostProcessor::processHalPPDataCB,
                                  QCamera3PostProcessor::releaseSuperBufCb, staticParam);
        if (rc != NO_ERROR) {
            LOGE("HAL PP type %d init failed, rc = %d", halPPType, rc);
        }
    }
    return rc;
}

void QCamera3PostProcessor::releaseSuperBufCb(mm_camera_super_buf_t *super_buf, void* pUserData)
{
    QCamera3PostProcessor *pme = (QCamera3PostProcessor *)pUserData;
    pme->releaseSuperBuf(super_buf);
}

/*===========================================================================
 * FUNCTION   : getChannelByHandle
 *
 * DESCRIPTION: function to get channel by handle
 * PARAMETERS :
 *   @channelHandle  : channel handle
 * RETURN     : QCameraChannel
 *==========================================================================*/
QCamera3Channel *QCamera3PostProcessor::getChannelByHandle(uint32_t channelHandle)
{
    QCamera3Channel *pChannel = NULL;

    if (m_parent->getMyHandle() == channelHandle) {
        pChannel = m_parent;
    }
    // check reprocess channel if not found
    if (pChannel == NULL) {
        for (int8_t i = 0; i < m_ppChannelCnt; i++) {
            if (m_pReprocChannel[i] != NULL &&
                m_pReprocChannel[i]->getMyHandle() == channelHandle) {
                pChannel = m_pReprocChannel[i];
                break;
            }
        }
    }
    return pChannel;
}

/* EXIF related helper methods */

/*===========================================================================
 * FUNCTION   : getRational
 *
 * DESCRIPTION: compose rational struct
 *
 * PARAMETERS :
 *   @rat     : ptr to struct to store rational info
 *   @num     :num of the rational
 *   @denom   : denom of the rational
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getRational(rat_t *rat, int num, int denom)
{
    if ((0 > num) || (0 >= denom)) {
        LOGE("Negative values");
        return BAD_VALUE;
    }
    if (NULL == rat) {
        LOGE("NULL rat input");
        return BAD_VALUE;
    }
    rat->num = (uint32_t)num;
    rat->denom = (uint32_t)denom;
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : parseGPSCoordinate
 *
 * DESCRIPTION: parse GPS coordinate string
 *
 * PARAMETERS :
 *   @coord_str : [input] coordinate string
 *   @coord     : [output]  ptr to struct to store coordinate
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int parseGPSCoordinate(const char *coord_str, rat_t* coord)
{
    if(coord == NULL) {
        LOGE("error, invalid argument coord == NULL");
        return BAD_VALUE;
    }
    double degF = atof(coord_str);
    if (degF < 0) {
        degF = -degF;
    }
    double minF = (degF - (int) degF) * 60;
    double secF = (minF - (int) minF) * 60;

    getRational(&coord[0], (int)degF, 1);
    getRational(&coord[1], (int)minF, 1);
    getRational(&coord[2], (int)(secF * 10000), 10000);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getExifDateTime
 *
 * DESCRIPTION: query exif date time
 *
 * PARAMETERS :
 *   @dateTime   : string to store exif date time
 *   @subsecTime : string to store exif subsec time
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifDateTime(String8 &dateTime, String8 &subsecTime)
{
    int32_t ret = NO_ERROR;

    //get time and date from system
    struct timeval tv;
    struct tm timeinfo_data;

    int res = gettimeofday(&tv, NULL);
    if (0 == res) {
        struct tm *timeinfo = localtime_r(&tv.tv_sec, &timeinfo_data);
        if (NULL != timeinfo) {
            //Write datetime according to EXIF Spec
            //"YYYY:MM:DD HH:MM:SS" (20 chars including \0)
            dateTime = String8::format("%04d:%02d:%02d %02d:%02d:%02d",
                    timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
                    timeinfo->tm_mday, timeinfo->tm_hour,
                    timeinfo->tm_min, timeinfo->tm_sec);
            //Write subsec according to EXIF Sepc
            subsecTime = String8::format("%06ld", tv.tv_usec);
        } else {
            LOGE("localtime_r() error");
            ret = UNKNOWN_ERROR;
        }
    } else if (-1 == res) {
        LOGE("gettimeofday() error: %s", strerror(errno));
        ret = UNKNOWN_ERROR;
    } else {
        LOGE("gettimeofday() unexpected return code: %d", res);
        ret = UNKNOWN_ERROR;
    }

    return ret;
}

/*===========================================================================
 * FUNCTION   : getExifFocalLength
 *
 * DESCRIPTION: get exif focal length
 *
 * PARAMETERS :
 *   @focalLength : ptr to rational struct to store focal length
 *   @value       : focal length value
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifFocalLength(rat_t *focalLength, float value)
{
    int focalLengthValue =
        (int)(value * FOCAL_LENGTH_DECIMAL_PRECISION);
    return getRational(focalLength, focalLengthValue, FOCAL_LENGTH_DECIMAL_PRECISION);
}

/*===========================================================================
  * FUNCTION   : getExifExpTimeInfo
  *
  * DESCRIPTION: get exif exposure time information
  *
  * PARAMETERS :
  *   @expoTimeInfo     : rational exposure time value
  *   @value            : exposure time value
  * RETURN     : nt32_t type of status
  *              NO_ERROR  -- success
  *              none-zero failure code
  *==========================================================================*/
int32_t getExifExpTimeInfo(rat_t *expoTimeInfo, int64_t value)
{

    int64_t cal_exposureTime;
    if (value != 0)
        cal_exposureTime = value;
    else
        cal_exposureTime = 60;

    return getRational(expoTimeInfo, 1, (int)cal_exposureTime);
}

/*===========================================================================
 * FUNCTION   : getExifGpsProcessingMethod
 *
 * DESCRIPTION: get GPS processing method
 *
 * PARAMETERS :
 *   @gpsProcessingMethod : string to store GPS process method
 *   @count               : length of the string
 *   @value               : the value of the processing method
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifGpsProcessingMethod(char *gpsProcessingMethod,
        uint32_t &count, char* value)
{
    if(value != NULL) {
        memcpy(gpsProcessingMethod, ExifAsciiPrefix, EXIF_ASCII_PREFIX_SIZE);
        count = EXIF_ASCII_PREFIX_SIZE;
        strlcpy(gpsProcessingMethod + EXIF_ASCII_PREFIX_SIZE,
                value,
                GPS_PROCESSING_METHOD_SIZE);
        count += (uint32_t)strlen(value);
        gpsProcessingMethod[count++] = '\0'; // increase 1 for the last NULL char
        return NO_ERROR;
    } else {
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifLatitude
 *
 * DESCRIPTION: get exif latitude
 *
 * PARAMETERS :
 *   @latitude : ptr to rational struct to store latitude info
 *   @latRef   : character to indicate latitude reference
 *   @value    : value of the latitude
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifLatitude(rat_t *latitude, char *latRef, double value)
{
    char str[30];
    snprintf(str, sizeof(str), "%f", value);
    if(str[0] != '\0') {
        parseGPSCoordinate(str, latitude);

        //set Latitude Ref
        float latitudeValue = strtof(str, 0);
        if(latitudeValue < 0.0f) {
            latRef[0] = 'S';
        } else {
            latRef[0] = 'N';
        }
        latRef[1] = '\0';
        return NO_ERROR;
    }else{
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifLongitude
 *
 * DESCRIPTION: get exif longitude
 *
 * PARAMETERS :
 *   @longitude : ptr to rational struct to store longitude info
 *   @lonRef    : character to indicate longitude reference
 *   @value     : value of the longitude
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifLongitude(rat_t *longitude, char *lonRef, double value)
{
    char str[30];
    snprintf(str, sizeof(str), "%f", value);
    if(str[0] != '\0') {
        parseGPSCoordinate(str, longitude);

        //set Longitude Ref
        float longitudeValue = strtof(str, 0);
        if(longitudeValue < 0.0f) {
            lonRef[0] = 'W';
        } else {
            lonRef[0] = 'E';
        }
        lonRef[1] = '\0';
        return NO_ERROR;
    }else{
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifAltitude
 *
 * DESCRIPTION: get exif altitude
 *
 * PARAMETERS :
 *   @altitude : ptr to rational struct to store altitude info
 *   @altRef   : character to indicate altitude reference
 *   @argValue : altitude value
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifAltitude(rat_t *altitude, char *altRef, double argValue)
{
    char str[30];
    snprintf(str, sizeof(str), "%f", argValue);
    if (str[0] != '\0') {
        double value = atof(str);
        *altRef = 0;
        if(value < 0){
            *altRef = 1;
            value = -value;
        }
        return getRational(altitude, (int)(value * 1000), 1000);
    } else {
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifGpsDateTimeStamp
 *
 * DESCRIPTION: get exif GPS date time stamp
 *
 * PARAMETERS :
 *   @gpsDateStamp : GPS date time stamp string
 *   @bufLen       : length of the string
 *   @gpsTimeStamp : ptr to rational struct to store time stamp info
 *   @value        : timestamp value
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifGpsDateTimeStamp(char *gpsDateStamp, uint32_t bufLen,
        rat_t *gpsTimeStamp, int64_t value)
{
    char str[30];
    snprintf(str, sizeof(str), "%lld", (long long int)value);
    if(str[0] != '\0') {
        time_t unixTime = (time_t)atol(str);
        struct tm *UTCTimestamp = gmtime(&unixTime);
        if (UTCTimestamp != NULL && gpsDateStamp != NULL
                && gpsTimeStamp != NULL) {
            strftime(gpsDateStamp, bufLen, "%Y:%m:%d", UTCTimestamp);

            getRational(&gpsTimeStamp[0], UTCTimestamp->tm_hour, 1);
            getRational(&gpsTimeStamp[1], UTCTimestamp->tm_min, 1);
            getRational(&gpsTimeStamp[2], UTCTimestamp->tm_sec, 1);
            return NO_ERROR;
        } else {
            LOGE("Could not get the timestamp");
            return BAD_VALUE;
        }
    } else {
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getExifExposureValue
 *
 * DESCRIPTION: get exif GPS date time stamp
 *
 * PARAMETERS :
 *   @exposure_val        : rational exposure value
 *   @exposure_comp       : exposure compensation
 *   @step                : exposure step
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t getExifExposureValue(srat_t* exposure_val, int32_t exposure_comp,
        cam_rational_type_t step)
{
    exposure_val->num = exposure_comp * step.numerator;
    exposure_val->denom = step.denominator;
    return 0;
}

/*===========================================================================
 * FUNCTION   : getExifData
 *
 * DESCRIPTION: get exif data to be passed into jpeg encoding
 *
 * PARAMETERS :
 * @metadata      : metadata of the encoding request
 * @jpeg_settings : jpeg_settings for encoding
 * @needJpegExifRotation: check if rotation need to added in EXIF
 *
 * RETURN     : exif data from user setting and GPS
 *==========================================================================*/
QCamera3Exif *QCamera3PostProcessor::getExifData(metadata_buffer_t *metadata,
        jpeg_settings_t *jpeg_settings, bool needJpegExifRotation)
{
    QCamera3Exif *exif = new QCamera3Exif();
    if (exif == NULL) {
        LOGE("No memory for QCamera3Exif");
        return NULL;
    }
    QCamera3HardwareInterface* hal_obj = NULL;
    if (m_parent != NULL) {
        hal_obj = (QCamera3HardwareInterface*)m_parent->mUserData;
    } else {
        LOGE("m_parent is NULL, Error");
        delete exif;
        return NULL;
    }

    int32_t rc = NO_ERROR;
    uint32_t count = 0;

    // add exif entries
    String8 dateTime;
    String8 subsecTime;
    rc = getExifDateTime(dateTime, subsecTime);
    if (rc == NO_ERROR) {
        exif->addEntry(EXIFTAGID_DATE_TIME, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_EXIF_DATE_TIME_ORIGINAL, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_EXIF_DATE_TIME_DIGITIZED, EXIF_ASCII,
                (uint32_t)(dateTime.length() + 1), (void *)dateTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME, EXIF_ASCII,
                (uint32_t)(subsecTime.length() + 1), (void *)subsecTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME_ORIGINAL, EXIF_ASCII,
                (uint32_t)(subsecTime.length() + 1), (void *)subsecTime.string());
        exif->addEntry(EXIFTAGID_SUBSEC_TIME_DIGITIZED, EXIF_ASCII,
                (uint32_t)(subsecTime.length() + 1), (void *)subsecTime.string());
    } else {
        LOGW("getExifDateTime failed");
    }


    if (metadata != NULL) {
        IF_META_AVAILABLE(float, focal_length, CAM_INTF_META_LENS_FOCAL_LENGTH, metadata) {
            rat_t focalLength;
            rc = getExifFocalLength(&focalLength, *focal_length);
            if (rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_FOCAL_LENGTH,
                        EXIF_RATIONAL,
                        1,
                        (void *)&(focalLength));
            } else {
                LOGW("getExifFocalLength failed");
            }
        }

        char* jpeg_gps_processing_method = jpeg_settings->gps_processing_method;
        if (strlen(jpeg_gps_processing_method) > 0) {
            char gpsProcessingMethod[EXIF_ASCII_PREFIX_SIZE +
                    GPS_PROCESSING_METHOD_SIZE];
            count = 0;
            rc = getExifGpsProcessingMethod(gpsProcessingMethod,
                    count,
                    jpeg_gps_processing_method);
            if(rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_GPS_PROCESSINGMETHOD,
                        EXIFTAGTYPE_GPS_PROCESSINGMETHOD,
                        count,
                        (void *)gpsProcessingMethod);
            } else {
                LOGW("getExifGpsProcessingMethod failed");
            }
        }

        if (jpeg_settings->gps_coordinates_valid) {

            //latitude
            rat_t latitude[3];
            char latRef[2];
            rc = getExifLatitude(latitude, latRef,
                    jpeg_settings->gps_coordinates[0]);
            if(rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_GPS_LATITUDE,
                        EXIF_RATIONAL,
                        3,
                        (void *)latitude);
                exif->addEntry(EXIFTAGID_GPS_LATITUDE_REF,
                        EXIF_ASCII,
                        2,
                        (void *)latRef);
            } else {
                LOGW("getExifLatitude failed");
            }

            //longitude
            rat_t longitude[3];
            char lonRef[2];
            rc = getExifLongitude(longitude, lonRef,
                    jpeg_settings->gps_coordinates[1]);
            if(rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_GPS_LONGITUDE,
                        EXIF_RATIONAL,
                        3,
                        (void *)longitude);

                exif->addEntry(EXIFTAGID_GPS_LONGITUDE_REF,
                        EXIF_ASCII,
                        2,
                        (void *)lonRef);
            } else {
                LOGW("getExifLongitude failed");
            }

            //altitude
            rat_t altitude;
            char altRef;
            rc = getExifAltitude(&altitude, &altRef,
                    jpeg_settings->gps_coordinates[2]);
            if(rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_GPS_ALTITUDE,
                        EXIF_RATIONAL,
                        1,
                        (void *)&(altitude));

                exif->addEntry(EXIFTAGID_GPS_ALTITUDE_REF,
                        EXIF_BYTE,
                        1,
                        (void *)&altRef);
            } else {
                LOGW("getExifAltitude failed");
            }
        }

        if (jpeg_settings->gps_timestamp_valid) {

            char gpsDateStamp[20];
            rat_t gpsTimeStamp[3];
            rc = getExifGpsDateTimeStamp(gpsDateStamp, 20, gpsTimeStamp,
                    jpeg_settings->gps_timestamp);
            if(rc == NO_ERROR) {
                exif->addEntry(EXIFTAGID_GPS_DATESTAMP, EXIF_ASCII,
                        (uint32_t)(strlen(gpsDateStamp) + 1),
                        (void *)gpsDateStamp);

                exif->addEntry(EXIFTAGID_GPS_TIMESTAMP,
                        EXIF_RATIONAL,
                        3,
                        (void *)gpsTimeStamp);
            } else {
                LOGW("getExifGpsDataTimeStamp failed");
            }
        }

        IF_META_AVAILABLE(int32_t, exposure_comp, CAM_INTF_PARM_EXPOSURE_COMPENSATION, metadata) {
            IF_META_AVAILABLE(cam_rational_type_t, comp_step, CAM_INTF_PARM_EV_STEP, metadata) {
                srat_t exposure_val;
                rc = getExifExposureValue(&exposure_val, *exposure_comp, *comp_step);
                if(rc == NO_ERROR) {
                    exif->addEntry(EXIFTAGID_EXPOSURE_BIAS_VALUE,
                            EXIF_SRATIONAL,
                            1,
                            (void *)(&exposure_val));
                } else {
                    LOGW("getExifExposureValue failed ");
                }
            }
        }
    } else {
        LOGW("no metadata provided ");
    }

#ifdef ENABLE_MODEL_INFO_EXIF

    char value[PROPERTY_VALUE_MAX];
    if (property_get("ro.product.manufacturer", value, "QCOM-AA") > 0) {
        exif->addEntry(EXIFTAGID_MAKE, EXIF_ASCII,
                (uint32_t)(strlen(value) + 1), (void *)value);
    } else {
        LOGW("getExifMaker failed");
    }

    if (property_get("ro.product.model", value, "QCAM-AA") > 0) {
        exif->addEntry(EXIFTAGID_MODEL, EXIF_ASCII,
                (uint32_t)(strlen(value) + 1), (void *)value);
    } else {
        LOGW("getExifModel failed");
    }

    if (property_get("ro.build.description", value, "QCAM-AA") > 0) {
        exif->addEntry(EXIFTAGID_SOFTWARE, EXIF_ASCII,
                (uint32_t)(strlen(value) + 1), (void *)value);
    } else {
        LOGW("getExifSoftware failed");
    }

#endif

    if (jpeg_settings->image_desc_valid) {
        if (exif->addEntry(EXIFTAGID_IMAGE_DESCRIPTION, EXIF_ASCII,
                strlen(jpeg_settings->image_desc)+1,
                (void *)jpeg_settings->image_desc)) {
            LOGW("Adding IMAGE_DESCRIPTION tag failed");
        }
    }

    LOGD("needJpegExifRotation %d jpeg_settings->jpeg_orientation %d",
            needJpegExifRotation, jpeg_settings->jpeg_orientation);

    if (needJpegExifRotation) {
        int16_t orientation;
        switch (jpeg_settings->jpeg_orientation) {
            case 0:
                orientation = 1;
                break;
            case 90:
                orientation = 6;
                break;
            case 180:
                orientation = 3;
                break;
            case 270:
                orientation = 8;
                break;
            default:
                orientation = 1;
                break;
        }
        exif->addEntry(EXIFTAGID_ORIENTATION,
                       EXIF_SHORT,
                       1,
                       (void *)&orientation);
        exif->addEntry(EXIFTAGID_TN_ORIENTATION,
                       EXIF_SHORT,
                       1,
                       (void *)&orientation);

    }

    return exif;
}

/*===========================================================================
 * FUNCTION   : QCamera3Exif
 *
 * DESCRIPTION: constructor of QCamera3Exif
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3Exif::QCamera3Exif()
    : m_nNumEntries(0)
{
    memset(m_Entries, 0, sizeof(m_Entries));
}

/*===========================================================================
 * FUNCTION   : ~QCamera3Exif
 *
 * DESCRIPTION: deconstructor of QCamera3Exif. Will release internal memory ptr.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCamera3Exif::~QCamera3Exif()
{
    for (uint32_t i = 0; i < m_nNumEntries; i++) {
        switch (m_Entries[i].tag_entry.type) {
            case EXIF_BYTE:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._bytes != NULL) {
                        free(m_Entries[i].tag_entry.data._bytes);
                        m_Entries[i].tag_entry.data._bytes = NULL;
                    }
                }
                break;
            case EXIF_ASCII:
                {
                    if (m_Entries[i].tag_entry.data._ascii != NULL) {
                        free(m_Entries[i].tag_entry.data._ascii);
                        m_Entries[i].tag_entry.data._ascii = NULL;
                    }
                }
                break;
            case EXIF_SHORT:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._shorts != NULL) {
                        free(m_Entries[i].tag_entry.data._shorts);
                        m_Entries[i].tag_entry.data._shorts = NULL;
                    }
                }
                break;
            case EXIF_LONG:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._longs != NULL) {
                        free(m_Entries[i].tag_entry.data._longs);
                        m_Entries[i].tag_entry.data._longs = NULL;
                    }
                }
                break;
            case EXIF_RATIONAL:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._rats != NULL) {
                        free(m_Entries[i].tag_entry.data._rats);
                        m_Entries[i].tag_entry.data._rats = NULL;
                    }
                }
                break;
            case EXIF_UNDEFINED:
                {
                    if (m_Entries[i].tag_entry.data._undefined != NULL) {
                        free(m_Entries[i].tag_entry.data._undefined);
                        m_Entries[i].tag_entry.data._undefined = NULL;
                    }
                }
                break;
            case EXIF_SLONG:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._slongs != NULL) {
                        free(m_Entries[i].tag_entry.data._slongs);
                        m_Entries[i].tag_entry.data._slongs = NULL;
                    }
                }
                break;
            case EXIF_SRATIONAL:
                {
                    if (m_Entries[i].tag_entry.count > 1 &&
                            m_Entries[i].tag_entry.data._srats != NULL) {
                        free(m_Entries[i].tag_entry.data._srats);
                        m_Entries[i].tag_entry.data._srats = NULL;
                    }
                }
                break;
            default:
                LOGW("Error, Unknown type");
                break;
        }
    }
}

/*===========================================================================
 * FUNCTION   : addEntry
 *
 * DESCRIPTION: function to add an entry to exif data
 *
 * PARAMETERS :
 *   @tagid   : exif tag ID
 *   @type    : data type
 *   @count   : number of data in uint of its type
 *   @data    : input data ptr
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Exif::addEntry(exif_tag_id_t tagid,
                              exif_tag_type_t type,
                              uint32_t count,
                              void *data)
{
    int32_t rc = NO_ERROR;
    if(m_nNumEntries >= MAX_HAL3_EXIF_TABLE_ENTRIES) {
        LOGE("Number of entries exceeded limit");
        return NO_MEMORY;
    }

    m_Entries[m_nNumEntries].tag_id = tagid;
    m_Entries[m_nNumEntries].tag_entry.type = type;
    m_Entries[m_nNumEntries].tag_entry.count = count;
    m_Entries[m_nNumEntries].tag_entry.copy = 1;
    switch (type) {
        case EXIF_BYTE:
            {
                if (count > 1) {
                    uint8_t *values = (uint8_t *)malloc(count);
                    if (values == NULL) {
                        LOGE("No memory for byte array");
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, data, count);
                        m_Entries[m_nNumEntries].tag_entry.data._bytes = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._byte =
                        *(uint8_t *)data;
                }
            }
            break;
        case EXIF_ASCII:
            {
                char *str = NULL;
                str = (char *)malloc(count + 1);
                if (str == NULL) {
                    LOGE("No memory for ascii string");
                    rc = NO_MEMORY;
                } else {
                    memset(str, 0, count + 1);
                    memcpy(str, data, count);
                    m_Entries[m_nNumEntries].tag_entry.data._ascii = str;
                }
            }
            break;
        case EXIF_SHORT:
            {
                uint16_t *exif_data = (uint16_t *)data;
                if (count > 1) {
                    uint16_t *values =
                        (uint16_t *)malloc(count * sizeof(uint16_t));
                    if (values == NULL) {
                        LOGE("No memory for short array");
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, exif_data, count * sizeof(uint16_t));
                        m_Entries[m_nNumEntries].tag_entry.data._shorts = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._short =
                        *(uint16_t *)data;
                }
            }
            break;
        case EXIF_LONG:
            {
                uint32_t *exif_data = (uint32_t *)data;
                if (count > 1) {
                    uint32_t *values =
                        (uint32_t *)malloc(count * sizeof(uint32_t));
                    if (values == NULL) {
                        LOGE("No memory for long array");
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, exif_data, count * sizeof(uint32_t));
                        m_Entries[m_nNumEntries].tag_entry.data._longs = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._long =
                        *(uint32_t *)data;
                }
            }
            break;
        case EXIF_RATIONAL:
            {
                rat_t *exif_data = (rat_t *)data;
                if (count > 1) {
                    rat_t *values = (rat_t *)malloc(count * sizeof(rat_t));
                    if (values == NULL) {
                        LOGE("No memory for rational array");
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, exif_data, count * sizeof(rat_t));
                        m_Entries[m_nNumEntries].tag_entry.data._rats = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._rat =
                        *(rat_t *)data;
                }
            }
            break;
        case EXIF_UNDEFINED:
            {
                uint8_t *values = (uint8_t *)malloc(count);
                if (values == NULL) {
                    LOGE("No memory for undefined array");
                    rc = NO_MEMORY;
                } else {
                    memcpy(values, data, count);
                    m_Entries[m_nNumEntries].tag_entry.data._undefined = values;
                }
            }
            break;
        case EXIF_SLONG:
            {
                int32_t *exif_data = (int32_t *)data;
                if (count > 1) {
                    int32_t *values =
                        (int32_t *)malloc(count * sizeof(int32_t));
                    if (values == NULL) {
                        LOGE("No memory for signed long array");
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, exif_data, count * sizeof(int32_t));
                        m_Entries[m_nNumEntries].tag_entry.data._slongs =values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._slong =
                        *(int32_t *)data;
                }
            }
            break;
        case EXIF_SRATIONAL:
            {
                srat_t *exif_data = (srat_t *)data;
                if (count > 1) {
                    srat_t *values = (srat_t *)malloc(count * sizeof(srat_t));
                    if (values == NULL) {
                        LOGE("No memory for sign rational array");
                        rc = NO_MEMORY;
                    } else {
                        memcpy(values, exif_data, count * sizeof(srat_t));
                        m_Entries[m_nNumEntries].tag_entry.data._srats = values;
                    }
                } else {
                    m_Entries[m_nNumEntries].tag_entry.data._srat =
                        *(srat_t *)data;
                }
            }
            break;
        default:
            LOGE("Error, Unknown type");
            break;
    }

    // Increase number of entries
    m_nNumEntries++;
    return rc;
}

}; // namespace qcamera
