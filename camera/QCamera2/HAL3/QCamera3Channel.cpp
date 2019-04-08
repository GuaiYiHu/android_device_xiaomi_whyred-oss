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


#define LOG_TAG "QCamera3Channel"

// To remove
#include <cutils/properties.h>

// System dependencies
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include "hardware/gralloc.h"
#include <utils/Timers.h>

// Camera dependencies
#include "QCamera3Channel.h"
#include "QCamera3HWI.h"
#include "QCameraTrace.h"
#include "QCameraFormat.h"
extern "C" {
#include "mm_camera_dbg.h"
}

#ifdef ENABLE_QC_BOKEH
#include "dualcameraddm_wrapper.h"
#endif //ENABLE_QC_BOKEH

#define PAD_TO_SIZE(size, padding) \
        ((size + (typeof(size))(padding - 1)) & \
        (typeof(size))(~(padding - 1)))

using namespace android;

namespace qcamera {
#define IS_BUFFER_ERROR(x) (((x) & V4L2_BUF_FLAG_ERROR) == V4L2_BUF_FLAG_ERROR)

/*===========================================================================
 * FUNCTION   : QCamera3Channel
 *
 * DESCRIPTION: constrcutor of QCamera3Channel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3Channel::QCamera3Channel(uint32_t cam_handle,
                               uint32_t channel_handle,
                               mm_camera_ops_t *cam_ops,
                               channel_cb_routine cb_routine,
                               channel_cb_buffer_err cb_buffer_err,
                               cam_padding_info_t *paddingInfo,
                               cam_feature_mask_t postprocess_mask,
                               void *userData, uint32_t numBuffers)
                               : mMasterCam(CAM_TYPE_MAIN)
{
    m_camHandle = cam_handle;
    m_handle = channel_handle;
    m_camOps = cam_ops;
    m_bIsActive = false;
    m_bUBWCenable = true;

    m_numStreams = 0;
    memset(mStreams, 0, sizeof(mStreams));
    mUserData = userData;

    mStreamInfoBuf = NULL;
    mChannelCB = cb_routine;
    mChannelCbBufErr = cb_buffer_err;
    mPaddingInfo = *paddingInfo;
    mPaddingInfo.offset_info.offset_x = 0;
    mPaddingInfo.offset_info.offset_y = 0;

    mPostProcMask = postprocess_mask;

    mIsType = IS_TYPE_NONE;
    mNumBuffers = numBuffers;
    mPerFrameMapUnmapEnable = true;
    mDumpFrmCnt = 0;
    m_bDualChannel = false;
}

/*===========================================================================
 * FUNCTION   : ~QCamera3Channel
 *
 * DESCRIPTION: destructor of QCamera3Channel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3Channel::~QCamera3Channel()
{
}

/*===========================================================================
 * FUNCTION   : destroy
 *
 * DESCRIPTION: internal destructor of QCamera3Channel called by the subclasses
 *              this destructor will call pure virtual functions.  stop will eventuall call
 *              QCamera3Stream::putBufs.  The putBufs function will
 *              call QCamera3Channel::putStreamBufs which is pure virtual
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3Channel::destroy()
{
    if (m_bIsActive)
        stop();

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL) {
            delete mStreams[i];
            mStreams[i] = 0;
        }
    }
    m_numStreams = 0;
}

/*===========================================================================
 * FUNCTION   : addStream
 *
 * DESCRIPTION: add a stream into channel
 *
 * PARAMETERS :
 *   @streamType     : stream type
 *   @streamFormat   : stream format
 *   @streamDim      : stream dimension
 *   @streamRotation : rotation of the stream
 *   @minStreamBufNum : minimal buffer count for particular stream type
 *   @postprocessMask : post-proccess feature mask
 *   @isType         : type of image stabilization required on the stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::addStream(cam_stream_type_t streamType,
                                  cam_format_t streamFormat,
                                  cam_dimension_t streamDim,
                                  cam_rotation_t streamRotation,
                                  uint8_t minStreamBufNum,
                                  cam_feature_mask_t postprocessMask,
                                  cam_is_type_t isType,
                                  uint32_t batchSize)
{
    int32_t rc = NO_ERROR;

    if (m_numStreams >= 1) {
        LOGE("Only one stream per channel supported in v3 Hal");
        return BAD_VALUE;
    }

    if (m_numStreams >= MAX_STREAM_NUM_IN_BUNDLE) {
        LOGE("stream number (%d) exceeds max limit (%d)",
               m_numStreams, MAX_STREAM_NUM_IN_BUNDLE);
        return BAD_VALUE;
    }
    QCamera3Stream *pStream = new QCamera3Stream(m_camHandle,
                                               m_handle,
                                               m_camOps,
                                               &mPaddingInfo,
                                               this);
    if (pStream == NULL) {
        LOGE("No mem for Stream");
        return NO_MEMORY;
    }
    LOGD("batch size is %d", batchSize);

    rc = pStream->init(streamType, streamFormat, streamDim, streamRotation,
            NULL, minStreamBufNum, postprocessMask, isType, batchSize,
            streamCbRoutine, this);
    if (rc == 0) {
        mStreams[m_numStreams] = pStream;
        m_numStreams++;
    } else {
        delete pStream;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: start channel, which will start all streams belong to this channel
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::start()
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_CH_START);
    int32_t rc = NO_ERROR;

    if (m_numStreams > 1) {
        LOGW("bundle not supported");
    } else if (m_numStreams == 0) {
        return NO_INIT;
    }

    if(m_bIsActive) {
        LOGW("Attempt to start active channel");
        return rc;
    }

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL) {
            mStreams[i]->start();
        }
    }

    m_bIsActive = true;

    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop a channel, which will stop all streams belong to this channel
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::stop()
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_CH_STOP);
    int32_t rc = NO_ERROR;
    if(!m_bIsActive) {
        LOGE("Attempt to stop inactive channel");
        return rc;
    }

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL) {
            mStreams[i]->stop();
        }
    }

    m_bIsActive = false;
    return rc;
}

/*===========================================================================
 * FUNCTION   : setBatchSize
 *
 * DESCRIPTION: Set batch size for the channel. This is a dummy implementation
 *              for the base class
 *
 * PARAMETERS :
 *   @batchSize  : Number of image buffers in a batch
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success always
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::setBatchSize(uint32_t batchSize)
{
    LOGD("Dummy method. batchSize: %d unused ", batchSize);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : queueBatchBuf
 *
 * DESCRIPTION: This is a dummy implementation for the base class
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success always
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::queueBatchBuf()
{
    LOGD("Dummy method. Unused ");
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : setPerFrameMapUnmap
 *
 * DESCRIPTION: Sets internal enable flag
 *
 * PARAMETERS :
 *  @enable : Bool value for the enable flag
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success always
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::setPerFrameMapUnmap(bool enable)
{
    mPerFrameMapUnmapEnable = enable;
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : flush
 *
 * DESCRIPTION: flush a channel
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::flush()
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_CH_FLUSH);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : bufDone
 *
 * DESCRIPTION: return a stream buf back to kernel
 *
 * PARAMETERS :
 *   @recvd_frame  : stream buf frame to be returned
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::bufDone(mm_camera_super_buf_t *recvd_frame)
{
    int32_t rc = NO_ERROR;
    for (uint32_t i = 0; i < recvd_frame->num_bufs; i++) {
         if (recvd_frame->bufs[i] != NULL) {
             for (uint32_t j = 0; j < m_numStreams; j++) {
                 if (mStreams[j] != NULL &&
                     validate_handle(mStreams[j]->getMyHandle(), recvd_frame->bufs[i]->stream_id)) {
                     rc = mStreams[j]->bufDone(recvd_frame->bufs[i]->buf_idx);
                     break; // break loop j
                 }
             }
         }
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : setBundleInfo
 *
 * DESCRIPTION: setting bundle information in stream params
 *
 * PARAMETERS :
 *   @bundleInfo  : stream bundle information.
 *   @cam_type    : MAIN or AUX.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3Channel::setBundleInfo(const cam_bundle_config_t &bundleInfo, uint32_t cam_type)
{
    int32_t rc = NO_ERROR;
    cam_stream_parm_buffer_t param;
    memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
    param.type = CAM_STREAM_PARAM_TYPE_SET_BUNDLE_INFO;
    param.bundleInfo = bundleInfo;
    if (m_numStreams > 0 && mStreams[0]) {
        rc = mStreams[0]->setParameter(param, cam_type);
        if (rc != NO_ERROR) {
            LOGE("stream setParameter for set bundle failed");
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : getStreamTypeMask
 *
 * DESCRIPTION: Get bit mask of all stream types in this channel
 *
 * PARAMETERS : None
 *
 * RETURN     : Bit mask of all stream types in this channel
 *==========================================================================*/
uint32_t QCamera3Channel::getStreamTypeMask()
{
    uint32_t mask = 0;
    for (uint32_t i = 0; i < m_numStreams; i++) {
       mask |= (1U << mStreams[i]->getMyType());
    }
    return mask;
}

/*===========================================================================
 * FUNCTION   : getStreamID
 *
 * DESCRIPTION: Get StreamID of requested stream type
 *
 * PARAMETERS : streamMask
 *
 * RETURN     : Stream ID
 *==========================================================================*/
uint32_t QCamera3Channel::getStreamID(uint32_t streamMask)
{
    uint32_t streamID = 0;
    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (streamMask == (uint32_t )(0x1 << mStreams[i]->getMyType())) {
            streamID = mStreams[i]->getMyServerID();
            break;
        }
    }
    return streamID;
}

/*===========================================================================
 * FUNCTION   : getStreamByHandle
 *
 * DESCRIPTION: return stream object by stream handle
 *
 * PARAMETERS :
 *   @streamHandle : stream handle
 *
 * RETURN     : stream object. NULL if not found
 *==========================================================================*/
QCamera3Stream *QCamera3Channel::getStreamByHandle(uint32_t streamHandle)
{
    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL && validate_handle(mStreams[i]->getMyHandle(), streamHandle)) {
            return mStreams[i];
        }
    }
    return NULL;
}

/*===========================================================================
 * FUNCTION   : getStreamByIndex
 *
 * DESCRIPTION: return stream object by index
 *
 * PARAMETERS :
 *   @streamHandle : stream handle
 *
 * RETURN     : stream object. NULL if not found
 *==========================================================================*/
QCamera3Stream *QCamera3Channel::getStreamByIndex(uint32_t index)
{
    if (index < m_numStreams) {
        return mStreams[index];
    }
    return NULL;
}

/*===========================================================================
 * FUNCTION   : getPaddingInfo
 *
 * DESCRIPTION: return stream padding info
 *
 * RETURN     : ptr to padding info structure
 *==========================================================================*/
 cam_padding_info_t* QCamera3Channel::getPaddingInfo() {
    return &mPaddingInfo;
}

/*===========================================================================
 * FUNCTION   : streamCbRoutine
 *
 * DESCRIPTION: callback routine for stream
 *
 * PARAMETERS :
 *   @streamHandle : stream handle
 *
 * RETURN     : stream object. NULL if not found
 *==========================================================================*/
void QCamera3Channel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                QCamera3Stream *stream, void *userdata)
{
    QCamera3Channel *channel = (QCamera3Channel *)userdata;
    if (channel == NULL) {
        LOGE("invalid channel pointer");
        return;
    }
    channel->streamCbRoutine(super_frame, stream);
}

/*===========================================================================
 * FUNCTION   : dumpYUV
 *
 * DESCRIPTION: function to dump the YUV data from ISP/pproc
 *
 * PARAMETERS :
 *   @frame   : frame to be dumped
 *   @dim     : dimension of the stream
 *   @offset  : offset of the data
 *   @name    : 1 if it is ISP output/pproc input, 2 if it is pproc output
 *
 * RETURN  :
 *==========================================================================*/
void QCamera3Channel::dumpYUV(mm_camera_buf_def_t *frame, cam_dimension_t dim,
        cam_frame_len_offset_t offset, uint8_t dump_type)
{
    char buf[FILENAME_MAX];
    memset(buf, 0, sizeof(buf));
    static int counter = 0;
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.vendor.camera.dumpimg", prop, "0");
    mYUVDump = (uint32_t)atoi(prop);
    if (mYUVDump & dump_type) {
        mFrmNum = ((mYUVDump & 0xffff0000) >> 16);
        if (mFrmNum == 0) {
            mFrmNum = 10;
        }
        if (mFrmNum > 256) {
            mFrmNum = 256;
        }
        mSkipMode = ((mYUVDump & 0x0000ff00) >> 8);
        if (mSkipMode == 0) {
            mSkipMode = 1;
        }
        if (mDumpSkipCnt == 0) {
            mDumpSkipCnt = 1;
        }
        if (mDumpSkipCnt % mSkipMode == 0) {
            if (mDumpFrmCnt < mFrmNum) {
                /* Note that the image dimension will be the unrotated stream dimension.
                * If you feel that the image would have been rotated during reprocess
                * then swap the dimensions while opening the file
                * */
                switch (dump_type) {
                    case QCAMERA_DUMP_FRM_PREVIEW:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"p_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    case QCAMERA_DUMP_FRM_VIDEO:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"v_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    case QCAMERA_DUMP_FRM_INPUT_JPEG:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"s_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    case QCAMERA_DUMP_FRM_INPUT_REPROCESS:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"ir_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    case QCAMERA_DUMP_FRM_CALLBACK:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"c_%d_%d_%dx%d.yuv",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    case QCAMERA_DUMP_FRM_OUTPUT_JPEG:
                        snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"j_%d_%d_%dx%d.jpg",
                            counter, frame->frame_idx, dim.width, dim.height);
                    break;
                    default :
                        LOGE("dumping not enabled for stream type %d",dump_type);
                    break;
                }
                counter++;
                int file_fd = open(buf, O_RDWR | O_CREAT, 0777);
                ssize_t written_len = 0;
                if (file_fd >= 0) {
                    void *data = NULL;
                    fchmod(file_fd, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
                    if( dump_type == QCAMERA_DUMP_FRM_OUTPUT_JPEG ) {
                        written_len = write(file_fd, frame->buffer, frame->frame_len);
                    }
                    else {
                        for (uint32_t i = 0; i < offset.num_planes; i++) {
                            uint32_t index = offset.mp[i].offset;
                            if (i > 0) {
                                index += offset.mp[i-1].len;
                            }
                            for (int j = 0; j < offset.mp[i].height; j++) {
                                data = (void *)((uint8_t *)frame->buffer + index);
                                written_len += write(file_fd, data,
                                        (size_t)offset.mp[i].width);
                                index += (uint32_t)offset.mp[i].stride;
                            }
                        }
                    }
                    LOGH("written number of bytes %ld\n", written_len);
                    mDumpFrmCnt++;
                    frame->cache_flags |= CPU_HAS_READ;
                    close(file_fd);
                } else {
                    LOGE("failed to open file to dump image");
                }
            }
        } else {
            mDumpSkipCnt++;
        }
    }
}

/*===========================================================================
 * FUNCTION   : isUBWCEnabled
 *
 * DESCRIPTION: Function to get UBWC hardware support.
 *
 * PARAMETERS : None
 *
 * RETURN     : TRUE -- UBWC format supported
 *              FALSE -- UBWC is not supported.
 *==========================================================================*/
bool QCamera3Channel::isUBWCEnabled()
{
#ifdef UBWC_PRESENT
    char value[PROPERTY_VALUE_MAX];
    int prop_value = 0;
    memset(value, 0, sizeof(value));
    property_get("debug.gralloc.gfx_ubwc_disable", value, "0");
    prop_value = atoi(value);
    if (prop_value) {
        return FALSE;
    }

    //Disable UBWC if Eztune is enabled
    //EzTune process CPP output frame and cannot understand UBWC.
    memset(value, 0, sizeof(value));
    property_get("persist.vendor.camera.eztune.enable", value, "0");
    prop_value = atoi(value);
    if (prop_value) {
        return FALSE;
    }
    return TRUE;
#else
    return FALSE;
#endif
}

/*===========================================================================
 * FUNCTION   : setUBWCEnabled
 *
 * DESCRIPTION: set UBWC enable
 *
 * PARAMETERS : UBWC enable value
 *
 * RETURN     : none
 *
 *==========================================================================*/
void QCamera3Channel::setUBWCEnabled(bool val)
{
    m_bUBWCenable = val;
}

/*===========================================================================
 * FUNCTION   : getStreamDefaultFormat
 *
 * DESCRIPTION: return default buffer format for the stream
 *
 * PARAMETERS : type : Stream type
 *
 ** RETURN    : format for stream type
 *
 *==========================================================================*/
cam_format_t QCamera3Channel::getStreamDefaultFormat(cam_stream_type_t type,
        uint32_t width, uint32_t height)
{
    cam_format_t streamFormat;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;

    switch (type) {
    case CAM_STREAM_TYPE_PREVIEW:
        if (isUBWCEnabled()) {
            char prop[PROPERTY_VALUE_MAX];
            int pFormat;
            memset(prop, 0, sizeof(prop));
            property_get("persist.vendor.camera.preview.ubwc", prop, "1");
            pFormat = atoi(prop);
            if (pFormat == 1 && (m_bUBWCenable)) {
                streamFormat = CAM_FORMAT_YUV_420_NV12_UBWC;
            } else {
                /* Changed to macro to ensure format sent to gralloc for preview
                is also changed if the preview format is changed at camera HAL */
                streamFormat = PREVIEW_STREAM_FORMAT;
            }
        } else {
            /* Changed to macro to ensure format sent to gralloc for preview
            is also changed if the preview format is changed at camera HAL */
            streamFormat = PREVIEW_STREAM_FORMAT;
        }
        break;
    case CAM_STREAM_TYPE_VIDEO:
    {
        /* Disable UBWC for smaller video resolutions due to CPP downscale
            limits. Refer cpp_hw_params.h::CPP_DOWNSCALE_LIMIT_UBWC */
        if (isUBWCEnabled() && (width >= 640) && (height >= 480)) {
            QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
            if ((hal_obj->mCommon.isVideoUBWCEnabled())) {
                streamFormat = CAM_FORMAT_YUV_420_NV12_UBWC;
            } else {
                streamFormat = CAM_FORMAT_YUV_420_NV12_VENUS;
            }
        } else {
#if VENUS_PRESENT
        streamFormat = CAM_FORMAT_YUV_420_NV12_VENUS;
#else
        streamFormat = CAM_FORMAT_YUV_420_NV12;
#endif
        }
        break;
    }
    case CAM_STREAM_TYPE_SNAPSHOT:
        streamFormat = CAM_FORMAT_YUV_420_NV21;
        break;
    case CAM_STREAM_TYPE_CALLBACK:
        /* Changed to macro to ensure format sent to gralloc for callback
        is also changed if the preview format is changed at camera HAL */
        streamFormat = CALLBACK_STREAM_FORMAT;
        break;
    case CAM_STREAM_TYPE_RAW:
        streamFormat = hal_obj->mRdiModeFmt;
        break;
    case CAM_STREAM_TYPE_OFFLINE_PROC:
        if (hal_obj->isQuadCfaSensor() && hal_obj->getReprocChannelCnt() > 1) {
            if (hal_obj->mRdiModeFmt == CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG) {
                streamFormat = CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_GBRG;
            } else if (hal_obj->mRdiModeFmt == CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GRBG) {
                streamFormat = CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_GRBG;
            } else if (hal_obj->mRdiModeFmt == CAM_FORMAT_BAYER_MIPI_RAW_10BPP_RGGB) {
                streamFormat = CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_RGGB;
            } else if (hal_obj->mRdiModeFmt == CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR) {
                streamFormat = CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_BGGR;
            } else {
                LOGW("unkown rdi format, use default one for offline proc stream");
                streamFormat = CAM_FORMAT_BAYER_IDEAL_RAW_PLAIN16_10BPP_BGGR;
            }
        } else {
            streamFormat = CAM_FORMAT_YUV_420_NV21;
        }
        break;
    default:
        streamFormat = CAM_FORMAT_YUV_420_NV21;
        break;
    }
    return streamFormat;
}

/*===========================================================================
 * FUNCTION   : switchMaster
 *
 * DESCRIPTION: switch master camera in all the channels
 *
 * PARAMETERS : master camera
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3Channel::switchMaster(uint32_t masterCam)
{
    mMasterCam = masterCam;
    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mStreams[i] != NULL) {
            mStreams[i]->switchMaster(masterCam);
        }
    }
}

void QCamera3Channel::overridePPConfig(cam_feature_mask_t pp_mask)
{
    LOGD("overriding ppmask to %" PRIx64, pp_mask);
    mPostProcMask = pp_mask;
}

/* QCamera3ProcessingChannel methods */

/*===========================================================================
 * FUNCTION   : QCamera3ProcessingChannel
 *
 * DESCRIPTION: constructor of QCamera3ProcessingChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @cb_routine : callback routine to frame aggregator
 *   @paddingInfo: stream padding info
 *   @userData   : HWI handle
 *   @stream     : camera3_stream_t structure
 *   @stream_type: Channel stream type
 *   @postprocess_mask: the postprocess mask for streams of this channel
 *   @metadataChannel: handle to the metadataChannel
 *   @numBuffers : number of max dequeued buffers
 * RETURN     : none
 *==========================================================================*/
QCamera3ProcessingChannel::QCamera3ProcessingChannel(uint32_t cam_handle,
        uint32_t channel_handle,
        mm_camera_ops_t *cam_ops,
        channel_cb_routine cb_routine,
        channel_cb_buffer_err cb_buffer_err,
        cam_padding_info_t *paddingInfo,
        void *userData,
        camera3_stream_t *stream,
        cam_stream_type_t stream_type,
        cam_feature_mask_t postprocess_mask,
        QCamera3Channel *metadataChannel,
        uint32_t numBuffers) :
            QCamera3Channel(cam_handle, channel_handle, cam_ops, cb_routine,
                    cb_buffer_err, paddingInfo, postprocess_mask, userData, numBuffers),
            m_postprocessor(this),
            mFrameCount(0),
            mLastFrameCount(0),
            mLastFpsTime(0),
            mMemory(numBuffers),
            mNumBufs(CAM_MAX_NUM_BUFS_PER_STREAM),
            mStreamType(stream_type),
            mPostProcStarted(false),
            mInputBufferConfig(false),
            m_pMetaChannel(metadataChannel),
            mMetaFrame(NULL),
            mOfflineMemory(0),
            mOfflineMetaMemory(numBuffers + (MAX_REPROCESS_PIPELINE_STAGES - 1),
                    false),
            mJpegMemory(numBuffers),
            mCamera3Stream(stream)
{
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.vendor.debug.sf.showfps", prop, "0");
    mDebugFPS = (uint8_t) atoi(prop);

    int32_t rc = m_postprocessor.init(&mMemory);
    if (rc != 0) {
        LOGE("Init Postprocessor failed");
    }
}

/*===========================================================================
 * FUNCTION   : ~QCamera3ProcessingChannel
 *
 * DESCRIPTION: destructor of QCamera3ProcessingChannel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3ProcessingChannel::~QCamera3ProcessingChannel()
{
    destroy();

    int32_t rc = m_postprocessor.deinit();
    if (rc != 0) {
        LOGE("De-init Postprocessor failed");
    }

    if (0 < mOfflineMetaMemory.getCnt()) {
        mOfflineMetaMemory.deallocate();
    }
    if (0 < mOfflineMemory.getCnt()) {
        mOfflineMemory.unregisterBuffers();
    }

}

/*===========================================================================
 * FUNCTION   : streamCbRoutine
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 * @super_frame : the super frame with filled buffer
 * @stream      : stream on which the buffer was requested and filled
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3ProcessingChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
        QCamera3Stream *stream)
{
    if (mStreamType == CAM_STREAM_TYPE_PREVIEW) {
        KPI_ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PREVIEW_STRM_CB);
    } else {
        ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PROC_CH_STRM_CB);
    }
    //FIXME Q Buf back in case of error?
    uint8_t frameIndex;
    buffer_handle_t *resultBuffer;
    int32_t resultFrameNumber;
    camera3_stream_buffer_t result;
    cam_dimension_t dim;
    cam_frame_len_offset_t offset;

    memset(&dim, 0, sizeof(dim));
    memset(&offset, 0, sizeof(cam_frame_len_offset_t));
    if (checkStreamCbErrors(super_frame, stream) != NO_ERROR) {
        LOGE("Error with the stream callback");
        return;
    }

    frameIndex = (uint8_t)super_frame->bufs[0]->buf_idx;
    if(frameIndex >= mNumBufs) {
         LOGE("Error, Invalid index for buffer");
         stream->bufDone(frameIndex);
         return;
    }

    if (mDebugFPS) {
        showDebugFPS(stream->getMyType());
    }
    stream->getFrameDimension(dim);
    stream->getFrameOffset(offset);
    if (stream->getMyType() == CAM_STREAM_TYPE_PREVIEW) {
        dumpYUV(super_frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_PREVIEW);
    } else if (stream->getMyType() == CAM_STREAM_TYPE_VIDEO) {
        dumpYUV(super_frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_VIDEO);
    } else if (stream->getMyType() == CAM_STREAM_TYPE_CALLBACK) {
        dumpYUV(super_frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_CALLBACK);
    }

    do {

       //Use below data to issue framework callback
       resultBuffer = (buffer_handle_t *)mMemory.getBufferHandle(frameIndex);
       resultFrameNumber = mMemory.getFrameNumber(frameIndex);
       uint32_t oldestBufIndex;
       int32_t lowestFrameNumber = mMemory.getOldestFrameNumber(oldestBufIndex);
       QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
       if ((lowestFrameNumber != -1 ) && (lowestFrameNumber < resultFrameNumber) &&
            hal_obj->mOpMode != CAMERA3_STREAM_CONFIGURATION_CONSTRAINED_HIGH_SPEED_MODE) {
           LOGE("Error buffer dropped for framenumber:%d with bufidx:%d",
                   lowestFrameNumber, oldestBufIndex);
           if (mOutOfSequenceBuffers.empty()) {
              stream->cancelBuffer(oldestBufIndex);
           }
           mOutOfSequenceBuffers.push_back(super_frame);
           return;
       }

       if(hal_obj->mStreamConfig == true) {
          switch (stream->getMyType()) {
              case CAM_STREAM_TYPE_PREVIEW:
                  LOGI("[KPI Perf] : PROFILE_FIRST_PREVIEW_FRAME camera id %d",
                        hal_obj->getCameraID());
                  break;
              case CAM_STREAM_TYPE_VIDEO:
                  LOGI("[KPI Perf] : PROFILE_FIRST_VIDEO_FRAME camera id %d ",
                        hal_obj->getCameraID());
                  break;
              default:
                  break;
          }
          hal_obj->mStreamConfig = false;
       }

       result.stream = mCamera3Stream;
       result.buffer = resultBuffer;
       if (IS_BUFFER_ERROR(super_frame->bufs[0]->flags)) {
           result.status = CAMERA3_BUFFER_STATUS_ERROR;
           LOGW("CAMERA3_BUFFER_STATUS_ERROR for stream_type: %d",
                   mStreams[0]->getMyType());
           mChannelCbBufErr(this, resultFrameNumber, CAMERA3_BUFFER_STATUS_ERROR, mUserData);
       } else {
           result.status = CAMERA3_BUFFER_STATUS_OK;
       }
       result.acquire_fence = -1;
       result.release_fence = -1;
       if(mPerFrameMapUnmapEnable) {
           int32_t rc = stream->bufRelease(frameIndex);
           if (NO_ERROR != rc) {
               LOGE("Error %d releasing stream buffer %d",
                        rc, frameIndex);
           }

           rc = mMemory.unregisterBuffer(frameIndex);
           if (NO_ERROR != rc) {
               LOGE("Error %d unregistering stream buffer %d",
                        rc, frameIndex);
           }
       }

       if (0 <= resultFrameNumber) {
           if (mChannelCB) {
               mChannelCB(NULL, &result, (uint32_t)resultFrameNumber, false, mUserData);
           }
       } else {
           LOGE("Bad frame number");
       }
       free(super_frame);
       super_frame = NULL;
       if (mOutOfSequenceBuffers.empty()) {
          break;
       } else {
            auto itr = mOutOfSequenceBuffers.begin();
            super_frame = *itr;
            frameIndex = super_frame->bufs[0]->buf_idx;
            resultFrameNumber = mMemory.getFrameNumber(frameIndex);
            lowestFrameNumber = mMemory.getOldestFrameNumber(oldestBufIndex);
            LOGE("Attempting to recover next frame: result Frame#: %d, resultIdx: %d, "
                    "Lowest Frame#: %d, oldestBufIndex: %d",
                    resultFrameNumber, frameIndex, lowestFrameNumber, oldestBufIndex);
            if ((lowestFrameNumber != -1) && (lowestFrameNumber < resultFrameNumber)) {
                int32_t ret = NO_ERROR;
                LOGE("Multiple frame dropped requesting cancel for frame %d, idx:%d",
                        lowestFrameNumber, oldestBufIndex);
                ret = stream->cancelBuffer(oldestBufIndex);
                if (NO_ERROR == ret) {
                    return;
                } else {
                    //check if the buffer already exists in the out of sequence list
                    auto it = mOutOfSequenceBuffers.begin();
                    for (;it != mOutOfSequenceBuffers.end(); it++) {
                        mm_camera_super_buf_t *frame = *it;
                        if (frame->bufs[0]->buf_idx == oldestBufIndex) {
                            LOGE("buffer already in mOutOfSequenceBuffers list");
                            frameIndex = oldestBufIndex;
                            mOutOfSequenceBuffers.erase(it);
                            break;
                        }
                    }
                    if (it == mOutOfSequenceBuffers.end()) {
                        LOGE("SHOULD NOT BE HERE, BUFFER LOST !!!");
                        return;
                    }
                }
             } else if (lowestFrameNumber == resultFrameNumber) {
                LOGE("Time to flush out head of list continue loop with this new super frame");
                itr = mOutOfSequenceBuffers.erase(itr);
             } else {
                LOGE("Unexpected condition head of list is not the lowest frame number");
                itr = mOutOfSequenceBuffers.erase(itr);
             }
          }
    } while (1);
    return;
}

/*===========================================================================
 * FUNCTION   : putStreamBufs
 *
 * DESCRIPTION: release the buffers allocated to the stream
 *
 * PARAMETERS : NONE
 *
 * RETURN     : NONE
 *==========================================================================*/
void QCamera3YUVChannel::putStreamBufs()
{
    QCamera3ProcessingChannel::putStreamBufs();

    // Free allocated heap buffer.
    mMemory.deallocate();
    // Clear free heap buffer list.
    mFreeHeapBufferList.clear();
    // Clear offlinePpInfoList
    mOfflinePpInfoList.clear();
    // Clear offlineMemory
    mOfflineMemory.clear();
}

/*===========================================================================
 * FUNCTION   : timeoutFrame
 *
 * DESCRIPTION: Method to indicate to channel that a given frame has take too
 *              long to be generated
 *
 * PARAMETERS : framenumber indicating the framenumber of the buffer timingout
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::timeoutFrame(uint32_t frameNumber)
{
    int32_t bufIdx;

    bufIdx = mMemory.getBufferIndex(frameNumber);

    if (bufIdx < 0) {
        LOGE("Buffer not found for frame:%d", frameNumber);
        return -1;
    }

    mStreams[0]->timeoutFrame(bufIdx);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : request
 *
 * DESCRIPTION: handle the request - either with an input buffer or a direct
 *              output request
 *
 * PARAMETERS :
 * @buffer          : pointer to the output buffer
 * @frameNumber     : frame number of the request
 * @pInputBuffer    : pointer to input buffer if an input request
 * @metadata        : parameters associated with the request
 * @internalreq      : boolean to indicate if this is purely internal request
 *                    needing internal buffer allocation
 * @meteringonly    : boolean indicating metering only frame subset of internal
 *                    not consumed by postprocessor
 *
 * RETURN     : 0 on a success start of capture
 *              -EINVAL on invalid input
 *              -ENODEV on serious error
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::request(buffer_handle_t *buffer,
        uint32_t frameNumber,
        camera3_stream_buffer_t* pInputBuffer,
        metadata_buffer_t* metadata,
        int &indexUsed,
        __unused bool internalRequest = false,
        __unused bool meteringOnly = false)
{
    int32_t rc = NO_ERROR;
    int index;

    if (NULL == buffer || NULL == metadata) {
        LOGE("Invalid buffer/metadata in channel request");
        return BAD_VALUE;
    }

    if (pInputBuffer) {
        //need to send to reprocessing
        LOGD("Got a request with input buffer, output streamType = %d", mStreamType);
        reprocess_config_t reproc_cfg;
        cam_dimension_t dim;
        memset(&reproc_cfg, 0, sizeof(reprocess_config_t));
        memset(&dim, 0, sizeof(dim));
        setReprocConfig(reproc_cfg, pInputBuffer, metadata, mStreamFormat, dim);
        startPostProc(reproc_cfg);

        qcamera_fwk_input_pp_data_t *src_frame = NULL;
        src_frame = (qcamera_fwk_input_pp_data_t *)calloc(1,
                sizeof(qcamera_fwk_input_pp_data_t));
        if (src_frame == NULL) {
            LOGE("No memory for src frame");
            return NO_MEMORY;
        }
        rc = setFwkInputPPData(src_frame, pInputBuffer, &reproc_cfg, metadata, buffer, frameNumber);
        if (NO_ERROR != rc) {
            LOGE("Error %d while setting framework input PP data", rc);
            free(src_frame);
            return rc;
        }
        LOGH("Post-process started");
        m_postprocessor.processData(src_frame);
    } else {
        //need to fill output buffer with new data and return
        if(!m_bIsActive) {
            rc = registerBuffer(buffer, mIsType);
            if (NO_ERROR != rc) {
                LOGE("On-the-fly buffer registration failed %d",
                         rc);
                return rc;
            }

            rc = start();
            if (NO_ERROR != rc)
                return rc;
        } else {
            LOGD("Request on an existing stream");
        }

        index = mMemory.getMatchBufIndex((void*)buffer);
        if(index < 0) {
            rc = registerBuffer(buffer, mIsType);
            if (NO_ERROR != rc) {
                LOGE("On-the-fly buffer registration failed %d",
                         rc);
                return rc;
            }

            index = mMemory.getMatchBufIndex((void*)buffer);
            if (index < 0) {
                LOGE("Could not find object among registered buffers");
                return DEAD_OBJECT;
            }
        }
        rc = mMemory.markFrameNumber(index, frameNumber);
        if(rc != NO_ERROR) {
            LOGE("Error marking frame number:%d for index %d", frameNumber,
                index);
            return rc;
        }
        rc = mStreams[0]->bufDone(index);
        if(rc != NO_ERROR) {
            LOGE("Failed to Q new buffer to stream");
            mMemory.markFrameNumber(index, -1);
            return rc;
        }
        indexUsed = index;
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : initialize
 *
 * DESCRIPTION:
 *
 * PARAMETERS : isType : type of image stabilization on the buffer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::initialize(__unused cam_is_type_t isType)
{
    int32_t rc = NO_ERROR;
    rc = mOfflineMetaMemory.allocateAll(sizeof(metadata_buffer_t));
    if (rc == NO_ERROR) {
        Mutex::Autolock lock(mFreeOfflineMetaBuffersLock);
        mFreeOfflineMetaBuffersList.clear();
        for (uint32_t i = 0; i < mNumBuffers + (MAX_REPROCESS_PIPELINE_STAGES - 1);
                i++) {
            mFreeOfflineMetaBuffersList.push_back(i);
        }
    } else {
        LOGE("Could not allocate offline meta buffers for input reprocess");
    }
    mOutOfSequenceBuffers.clear();
    mReprocDropBufferList.clear();
    return rc;
}

/*===========================================================================
 * FUNCTION   : registerBuffer
 *
 * DESCRIPTION: register streaming buffer to the channel object
 *
 * PARAMETERS :
 *   @buffer     : buffer to be registered
 *   @isType     : image stabilization type on the stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::registerBuffer(buffer_handle_t *buffer,
        cam_is_type_t isType)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PROC_CH_REG_BUF);
    int rc = 0;
    mIsType = isType;
    cam_stream_type_t streamType;

    if ((uint32_t)mMemory.getCnt() > (mNumBufs - 1)) {
        LOGE("Trying to register more buffers than initially requested");
        return BAD_VALUE;
    }

    if (0 == m_numStreams) {
        rc = initialize(mIsType);
        if (rc != NO_ERROR) {
            LOGE("Couldn't initialize camera stream %d", rc);
            return rc;
        }
    }

    streamType = mStreams[0]->getMyType();
    rc = mMemory.registerBuffer(buffer, streamType);
    if (ALREADY_EXISTS == rc) {
        return NO_ERROR;
    } else if (NO_ERROR != rc) {
        LOGE("Buffer %p couldn't be registered %d", buffer, rc);
        return rc;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : setFwkInputPPData
 *
 * DESCRIPTION: fill out the framework src frame information for reprocessing
 *
 * PARAMETERS :
 *   @src_frame         : input pp data to be filled out
 *   @pInputBuffer      : input buffer for reprocessing
 *   @reproc_cfg        : pointer to the reprocess config
 *   @metadata          : pointer to the metadata buffer
 *   @output_buffer     : output buffer for reprocessing; could be NULL if not
 *                        framework allocated
 *   @frameNumber       : frame number of the request
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::setFwkInputPPData(qcamera_fwk_input_pp_data_t *src_frame,
        camera3_stream_buffer_t *pInputBuffer, reprocess_config_t *reproc_cfg,
        metadata_buffer_t *metadata, buffer_handle_t *output_buffer,
        uint32_t frameNumber)
{
    int32_t rc = NO_ERROR;
    int input_index = mOfflineMemory.getMatchBufIndex((void*)pInputBuffer->buffer);
    if(input_index < 0) {
        rc = mOfflineMemory.registerBuffer(pInputBuffer->buffer, mStreamType);
        if (NO_ERROR != rc) {
            LOGE("On-the-fly input buffer registration failed %d",
                     rc);
            return rc;
        }
        input_index = mOfflineMemory.getMatchBufIndex((void*)pInputBuffer->buffer);
        if (input_index < 0) {
            LOGE("Could not find object among registered buffers");
            return DEAD_OBJECT;
        }
    }
    mOfflineMemory.markFrameNumber(input_index, frameNumber);

    src_frame->src_frame = *pInputBuffer;
    rc = mOfflineMemory.getBufDef(reproc_cfg->input_stream_plane_info.plane_info,
            src_frame->input_buffer, input_index);
    if (rc != 0) {
        return rc;
    }
    dumpYUV(&src_frame->input_buffer, reproc_cfg->input_stream_dim,
            reproc_cfg->input_stream_plane_info.plane_info, QCAMERA_DUMP_FRM_INPUT_REPROCESS);
    cam_dimension_t dim = {sizeof(metadata_buffer_t), 1};
    cam_stream_buf_plane_info_t meta_planes;
    rc = mm_stream_calc_offset_metadata(&dim, &mPaddingInfo, &meta_planes);
    if (rc != 0) {
        LOGE("Metadata stream plane info calculation failed!");
        return rc;
    }
    uint32_t metaBufIdx;
    {
        Mutex::Autolock lock(mFreeOfflineMetaBuffersLock);
        if (mFreeOfflineMetaBuffersList.empty()) {
            LOGE("mFreeOfflineMetaBuffersList is null. Fatal");
            return BAD_VALUE;
        }

        metaBufIdx = *(mFreeOfflineMetaBuffersList.begin());
        mFreeOfflineMetaBuffersList.erase(mFreeOfflineMetaBuffersList.begin());
        LOGD("erasing %d, mFreeOfflineMetaBuffersList.size %d", metaBufIdx,
                mFreeOfflineMetaBuffersList.size());
    }

    mOfflineMetaMemory.markFrameNumber(metaBufIdx, frameNumber);

    mm_camera_buf_def_t meta_buf;
    cam_frame_len_offset_t offset = meta_planes.plane_info;
    rc = mOfflineMetaMemory.getBufDef(offset, meta_buf, metaBufIdx);
    if (NO_ERROR != rc) {
        return rc;
    }
    memcpy(meta_buf.buffer, metadata, sizeof(metadata_buffer_t));
    src_frame->metadata_buffer = meta_buf;
    src_frame->reproc_config = *reproc_cfg;
    src_frame->output_buffer = output_buffer;
    src_frame->frameNumber = frameNumber;
    return rc;
}

/*===========================================================================
 * FUNCTION   : checkStreamCbErrors
 *
 * DESCRIPTION: check the stream callback for errors
 *
 * PARAMETERS :
 *   @super_frame : the super frame with filled buffer
 *   @stream      : stream on which the buffer was requested and filled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::checkStreamCbErrors(mm_camera_super_buf_t *super_frame,
        QCamera3Stream *stream)
{
    if (NULL == stream) {
        LOGE("Invalid stream");
        return BAD_VALUE;
    }

    if(NULL == super_frame) {
         LOGE("Invalid Super buffer");
         return BAD_VALUE;
    }

    if(super_frame->num_bufs != 1) {
         LOGE("Multiple streams are not supported");
         return BAD_VALUE;
    }
    if(NULL == super_frame->bufs[0]) {
         LOGE("Error, Super buffer frame does not contain valid buffer");
         return BAD_VALUE;
    }
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getStreamSize
 *
 * DESCRIPTION: get the size from the camera3_stream_t for the channel
 *
 * PARAMETERS :
 *   @dim     : Return the size of the stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::getStreamSize(cam_dimension_t &dim)
{
    if (mCamera3Stream) {
        dim.width = mCamera3Stream->width;
        dim.height = mCamera3Stream->height;
        return NO_ERROR;
    } else {
        return BAD_VALUE;
    }
}

/*===========================================================================
 * FUNCTION   : getStreamBufs
 *
 * DESCRIPTION: get the buffers allocated to the stream
 *
 * PARAMETERS :
 * @len       : buffer length
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
QCamera3StreamMem* QCamera3ProcessingChannel::getStreamBufs(uint32_t /*len*/)
{
    KPI_ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_GETSTREAMBUFS);
    return &mMemory;
}

/*===========================================================================
 * FUNCTION   : putStreamBufs
 *
 * DESCRIPTION: release the buffers allocated to the stream
 *
 * PARAMETERS : NONE
 *
 * RETURN     : NONE
 *==========================================================================*/
void QCamera3ProcessingChannel::putStreamBufs()
{
    mMemory.unregisterBuffers();

    /* Reclaim all the offline metabuffers and push them to free list */
    {
        Mutex::Autolock lock(mFreeOfflineMetaBuffersLock);
        mFreeOfflineMetaBuffersList.clear();
        for (uint32_t i = 0; i < mOfflineMetaMemory.getCnt(); i++) {
            mFreeOfflineMetaBuffersList.push_back(i);
        }
    }
}


/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop processing channel, which will stop all streams within,
 *              including the reprocessing channel in postprocessor.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::stop()
{
    if (mStreamType == CAM_STREAM_TYPE_PREVIEW) {
        KPI_ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_STOP_PREVIEW);
    }
    int32_t rc = NO_ERROR;
    if(!m_bIsActive) {
        LOGE("Attempt to stop inactive channel");
        return rc;
    }

    m_postprocessor.stop();
    mPostProcStarted = false;
    rc |= QCamera3Channel::stop();
    return rc;
}

/*===========================================================================
 * FUNCTION   : startPostProc
 *
 * DESCRIPTION: figure out if the postprocessor needs to be restarted and if yes
 *              start it
 *
 * PARAMETERS :
 * @inputBufExists : whether there is an input buffer for post processing
 * @config         : reprocessing configuration
 * @metadata       : metadata associated with the reprocessing request
 *
 * RETURN     : NONE
 *==========================================================================*/
void QCamera3ProcessingChannel::startPostProc(const reprocess_config_t &config)
{
    if(!mPostProcStarted) {
        m_postprocessor.start(config);
        mPostProcStarted = true;
    }
}

/*===========================================================================
 * FUNCTION   : queueReprocMetadata
 *
 * DESCRIPTION: queue the reprocess metadata to the postprocessor
 *
 * PARAMETERS : metadata : the metadata corresponding to the pp frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::queueReprocMetadata(mm_camera_super_buf_t *metadata,
                                    uint32_t framenum, bool dropframe)
{
    Mutex::Autolock lock(mDropReprocBuffersLock);
    List<uint32_t>::iterator dropInfo;
    if(mReprocDropBufferList.size()) {
        for (dropInfo = mReprocDropBufferList.begin();
                dropInfo != mReprocDropBufferList.end(); dropInfo++) {
            if (*dropInfo == framenum) {
                mReprocDropBufferList.erase(dropInfo);
                return metadataBufDone(metadata);
            } else if (*dropInfo < framenum) {
                if(m_postprocessor.releaseReprocMetaBuffer(*dropInfo)) {
                    mReprocDropBufferList.erase(dropInfo);
                }
            }
        }
    }
    return m_postprocessor.processPPMetadata(metadata, framenum, dropframe);
}

/*===========================================================================
 * FUNCTION : metadataBufDone
 *
 * DESCRIPTION: Buffer done method for a metadata buffer
 *
 * PARAMETERS :
 * @recvd_frame : received metadata frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::metadataBufDone(mm_camera_super_buf_t *recvd_frame)
{
    int32_t rc = NO_ERROR;;
    if ((NULL == m_pMetaChannel) || (NULL == recvd_frame)) {
        LOGE("Metadata channel or metadata buffer invalid");
        return BAD_VALUE;
    }

    rc = ((QCamera3MetadataChannel*)m_pMetaChannel)->bufDone(recvd_frame);

    return rc;
}

/*===========================================================================
 * FUNCTION : translateStreamTypeAndFormat
 *
 * DESCRIPTION: translates the framework stream format into HAL stream type
 *              and format
 *
 * PARAMETERS :
 * @streamType   : translated stream type
 * @streamFormat : translated stream format
 * @stream       : fwk stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::translateStreamTypeAndFormat(camera3_stream_t *stream,
        cam_stream_type_t &streamType, cam_format_t &streamFormat)
{
    switch (stream->format) {
        case HAL_PIXEL_FORMAT_YCbCr_420_888:
            if(stream->stream_type == CAMERA3_STREAM_INPUT){
                streamType = CAM_STREAM_TYPE_SNAPSHOT;
                streamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_SNAPSHOT,
                        stream->width, stream->height);
            } else {
                streamType = CAM_STREAM_TYPE_CALLBACK;
                streamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_CALLBACK,
                        stream->width, stream->height);
            }
            break;
        case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
            if (stream->usage & GRALLOC_USAGE_HW_VIDEO_ENCODER) {
                streamType = CAM_STREAM_TYPE_VIDEO;
                streamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_VIDEO,
                        stream->width, stream->height);
            } else if(stream->stream_type == CAMERA3_STREAM_INPUT ||
                    stream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL ||
                    IS_USAGE_ZSL(stream->usage)){
                streamType = CAM_STREAM_TYPE_SNAPSHOT;
                streamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_SNAPSHOT,
                        stream->width, stream->height);
            } else {
                streamType = CAM_STREAM_TYPE_PREVIEW;
                streamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_PREVIEW,
                        stream->width, stream->height);
            }
            break;
        case HAL_PIXEL_FORMAT_RAW_OPAQUE:
        case HAL_PIXEL_FORMAT_RAW16:
        case HAL_PIXEL_FORMAT_RAW10:
            streamType = CAM_STREAM_TYPE_RAW;
            streamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_RAW,
                    stream->width, stream->height);
            break;
        case HAL_PIXEL_FORMAT_RAW8:
            streamType = CAM_STREAM_TYPE_RAW;
            streamFormat = CAM_FORMAT_BAYER_MIPI_RAW_8BPP_GBRG;
            break;
        case HAL_PIXEL_FORMAT_BLOB:
            if (stream->data_space == HAL_DATASPACE_DEPTH) {
                streamType = CAM_STREAM_TYPE_DEPTH;
                streamFormat = CAM_FORMAT_DEPTH_POINT_CLOUD;
            }
            break;
        case HAL_PIXEL_FORMAT_Y16:
            if (stream->data_space == HAL_DATASPACE_DEPTH) {
                streamType = CAM_STREAM_TYPE_DEPTH;
                streamFormat = CAM_FORMAT_DEPTH16;
            }
            break;
        case HAL_PIXEL_FORMAT_Y8:
            if (stream->data_space == HAL_DATASPACE_DEPTH) {
                streamType = CAM_STREAM_TYPE_DEPTH;
                streamFormat = CAM_FORMAT_DEPTH8;
            }
            break;
        default:
            return -EINVAL;
    }
    LOGD("fwk_format = %d, streamType = %d, streamFormat = %d",
            stream->format, streamType, streamFormat);
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION : setReprocConfig
 *
 * DESCRIPTION: sets the reprocessing parameters for the input buffer
 *
 * PARAMETERS :
 * @reproc_cfg : the configuration to be set
 * @pInputBuffer : pointer to the input buffer
 * @metadata : pointer to the reprocessing metadata buffer
 * @streamFormat : format of the input stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::setReprocConfig(reprocess_config_t &reproc_cfg,
        camera3_stream_buffer_t *pInputBuffer,
        __unused metadata_buffer_t *metadata,
        cam_format_t streamFormat, cam_dimension_t dim, bool bNeedUpScale)
{
    int32_t rc = 0;
    reproc_cfg.padding = &mPaddingInfo;
    //to ensure a big enough buffer size set the height and width
    //padding to max(height padding, width padding)
    if (reproc_cfg.padding->height_padding > reproc_cfg.padding->width_padding) {
       reproc_cfg.padding->width_padding = reproc_cfg.padding->height_padding;
    } else {
       reproc_cfg.padding->height_padding = reproc_cfg.padding->width_padding;
    }

    if (NULL != pInputBuffer && !bNeedUpScale) {
        reproc_cfg.input_stream_dim.width = (int32_t)pInputBuffer->stream->width;
        reproc_cfg.input_stream_dim.height = (int32_t)pInputBuffer->stream->height;
    } else {
        reproc_cfg.input_stream_dim.width = (int32_t)dim.width;
        reproc_cfg.input_stream_dim.height = (int32_t)dim.height;
    }
    reproc_cfg.src_channel = this;

    QCamera3HardwareInterface *hal_obj = (QCamera3HardwareInterface *)mUserData;
    //In bokeh mode: upscaling will be done by jpeg in asymetric mode.
    if((hal_obj->getHalPPType() == CAM_HAL_PP_TYPE_BOKEH ) && bNeedUpScale)
    {
        reproc_cfg.output_stream_dim.width = (int32_t)dim.width;
        reproc_cfg.output_stream_dim.height = (int32_t)dim.height;
    }else {
        reproc_cfg.output_stream_dim.width = mCamera3Stream->width;
        reproc_cfg.output_stream_dim.height = mCamera3Stream->height;
    }
    reproc_cfg.reprocess_type = getReprocessType();
    reproc_cfg.output_stream_format = streamFormat;
    //offset calculation
    if (NULL != pInputBuffer) {
        rc = translateStreamTypeAndFormat(pInputBuffer->stream,
                reproc_cfg.stream_type, reproc_cfg.stream_format);
        if (rc != NO_ERROR) {
            LOGE("Stream format %d is not supported",
                    pInputBuffer->stream->format);
            return rc;
        }
    } else {
        reproc_cfg.stream_type = mStreamType;
        reproc_cfg.stream_format = streamFormat;
    }

    if (getStreamByIndex(0) == NULL) {
        LOGE("Could not find stream");
        rc = -1;
        return rc;
    }

    switch (reproc_cfg.stream_type) {
        case CAM_STREAM_TYPE_PREVIEW:
            rc = mm_stream_calc_offset_preview(
                    getStreamByIndex(0)->getStreamInfo(),
                    &reproc_cfg.input_stream_dim,
                    reproc_cfg.padding,
                    &reproc_cfg.input_stream_plane_info);
            break;
        case CAM_STREAM_TYPE_VIDEO:
            rc = mm_stream_calc_offset_video(getStreamByIndex(0)->getStreamInfo(),
                    reproc_cfg.padding,
                    &reproc_cfg.input_stream_plane_info);
            break;
        case CAM_STREAM_TYPE_RAW:
            rc = mm_stream_calc_offset_raw(reproc_cfg.stream_format,
                    &reproc_cfg.input_stream_dim,
                    reproc_cfg.padding, &reproc_cfg.input_stream_plane_info);
            break;
        case CAM_STREAM_TYPE_SNAPSHOT:
        case CAM_STREAM_TYPE_CALLBACK:
        default:
            rc = mm_stream_calc_offset_snapshot(streamFormat, &reproc_cfg.input_stream_dim,
                    reproc_cfg.padding, &reproc_cfg.input_stream_plane_info);
            break;
    }
    if (rc != 0) {
        LOGE("Stream %d plane info calculation failed!", mStreamType);
        return rc;
    }
    IF_META_AVAILABLE(cam_hdr_param_t, hdr_info, CAM_INTF_PARM_HAL_BRACKETING_HDR, metadata) {
        reproc_cfg.hdr_param = *hdr_info;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : reprocessCbRoutine
 *
 * DESCRIPTION: callback function for the reprocessed frame. This frame now
 *              should be returned to the framework
 *
 * PARAMETERS :
 * @resultBuffer      : buffer containing the reprocessed data
 * @resultFrameNumber : frame number on which the buffer was requested
 *
 * RETURN     : NONE
 *
 *==========================================================================*/
void QCamera3ProcessingChannel::reprocessCbRoutine(buffer_handle_t *resultBuffer,
        uint32_t resultFrameNumber)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PROC_CH_REPROC_CB);
    int rc = NO_ERROR;

    LOGD("E resultFrameNumber: %d", resultFrameNumber);
    rc = releaseOfflineMemory(resultFrameNumber);
    if (NO_ERROR != rc) {
        LOGE("Error releasing offline memory %d", rc);
    }
    /* Since reprocessing is done, send the callback to release the input buffer */
    if (mChannelCB) {
        mChannelCB(NULL, NULL, resultFrameNumber, true, mUserData);
    }
    issueChannelCb(resultBuffer, resultFrameNumber);

    return;
}

/*===========================================================================
 * FUNCTION   : issueChannelCb
 *
 * DESCRIPTION: function to set the result and issue channel callback
 *
 * PARAMETERS :
 * @resultBuffer      : buffer containing the data
 * @resultFrameNumber : frame number on which the buffer was requested
 *
 * RETURN     : NONE
 *
 *
 *==========================================================================*/
void QCamera3ProcessingChannel::issueChannelCb(buffer_handle_t *resultBuffer,
        uint32_t resultFrameNumber)
{
    camera3_stream_buffer_t result;
    //Use below data to issue framework callback
    result.stream = mCamera3Stream;
    result.buffer = resultBuffer;
    result.status = CAMERA3_BUFFER_STATUS_OK;
    result.acquire_fence = -1;
    result.release_fence = -1;

    if (mChannelCB) {
        mChannelCB(NULL, &result, resultFrameNumber, false, mUserData);
    }
}

/*===========================================================================
 * FUNCTION   : showDebugFPS
 *
 * DESCRIPTION: Function to log the fps for preview, video, callback and raw
 *              streams
 *
 * PARAMETERS : Stream type
 *
 * RETURN  : None
 *==========================================================================*/
void QCamera3ProcessingChannel::showDebugFPS(int32_t streamType)
{
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    int cameraId = -1;
    if (hal_obj != NULL) {
        cameraId = hal_obj->getCameraId();
    } else {
        LOGE("Failed to get hal obj for cameraId");
    }

    double fps = 0;
    mFrameCount++;
    nsecs_t now = systemTime();
    nsecs_t diff = now - mLastFpsTime;
    if (diff > ms2ns(250)) {
        fps = (((double)(mFrameCount - mLastFrameCount)) *
                (double)(s2ns(1))) / (double)diff;
        switch(streamType) {
            case CAM_STREAM_TYPE_PREVIEW:
                LOGH("PROFILE_PREVIEW_FRAMES_PER_SECOND CAMERA %d: %.4f: mFrameCount=%d",
                         cameraId, fps, mFrameCount);
                break;
            case CAM_STREAM_TYPE_VIDEO:
                LOGH("PROFILE_VIDEO_FRAMES_PER_SECOND CAMERA %d: %.4f",
                         cameraId, fps);
                break;
            case CAM_STREAM_TYPE_CALLBACK:
                LOGH("PROFILE_CALLBACK_FRAMES_PER_SECOND CAMERA %d: %.4f",
                         cameraId, fps);
                break;
            case CAM_STREAM_TYPE_RAW:
                LOGH("PROFILE_RAW_FRAMES_PER_SECOND CAMERA %d: %.4f",
                         cameraId, fps);
                break;
            default:
                LOGH("logging not supported for the stream");
                break;
        }
        mLastFpsTime = now;
        mLastFrameCount = mFrameCount;
    }
}

/*===========================================================================
 * FUNCTION   : releaseOfflineMemory
 *
 * DESCRIPTION: function to clean up the offline memory used for input reprocess
 *
 * PARAMETERS :
 * @resultFrameNumber : frame number on which the buffer was requested
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              non-zero failure code
 *
 *
 *==========================================================================*/
int32_t QCamera3ProcessingChannel::releaseOfflineMemory(uint32_t resultFrameNumber)
{
    int32_t rc = NO_ERROR;
    int32_t inputBufIndex =
            mOfflineMemory.getGrallocBufferIndex(resultFrameNumber);
    if (0 <= inputBufIndex) {
        rc = mOfflineMemory.unregisterBuffer(inputBufIndex);
    } else {
        LOGW("Could not find offline input buffer, resultFrameNumber %d",
                 resultFrameNumber);
    }
    if (rc != NO_ERROR) {
        LOGE("Failed to unregister offline input buffer");
    }

    int32_t metaBufIndex =
            mOfflineMetaMemory.getHeapBufferIndex(resultFrameNumber);
    if (0 <= metaBufIndex) {
        Mutex::Autolock lock(mFreeOfflineMetaBuffersLock);
        mFreeOfflineMetaBuffersList.push_back((uint32_t)metaBufIndex);
    } else {
        LOGW("Could not find offline meta buffer, resultFrameNumber %d",
                resultFrameNumber);
    }

    return rc;
}

int32_t QCamera3ProcessingChannel::releaseInputBuffer(uint32_t resultFrameNumber)
{
    int32_t rc = NO_ERROR;
    int32_t inputBufIndex =
            mOfflineMemory.getGrallocBufferIndex(resultFrameNumber);
    if (0 <= inputBufIndex) {
        rc = mOfflineMemory.unregisterBuffer(inputBufIndex);
    } else {
        LOGW("Could not find offline input buffer, resultFrameNumber %d",
                 resultFrameNumber);
    }
    if (rc != NO_ERROR) {
        LOGE("Failed to unregister offline input buffer");
    }

    return rc;
}


bool QCamera3ProcessingChannel::isFwkInputBuffer(uint32_t resultFrameNumber)
{
    int32_t inputBufIndex =
                mOfflineMemory.getGrallocBufferIndex(resultFrameNumber);
    if (0 <= inputBufIndex)
        return true;
    else
        return false;
}


/* Regular Channel methods */
/*===========================================================================
 * FUNCTION   : QCamera3RegularChannel
 *
 * DESCRIPTION: constructor of QCamera3RegularChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @cb_routine : callback routine to frame aggregator
 *   @stream     : camera3_stream_t structure
 *   @stream_type: Channel stream type
 *   @postprocess_mask: feature mask for postprocessing
 *   @metadataChannel : metadata channel for the session
 *   @numBuffers : number of max dequeued buffers
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3RegularChannel::QCamera3RegularChannel(uint32_t cam_handle,
        uint32_t channel_handle,
        mm_camera_ops_t *cam_ops,
        channel_cb_routine cb_routine,
        channel_cb_buffer_err cb_buffer_err,
        cam_padding_info_t *paddingInfo,
        void *userData,
        camera3_stream_t *stream,
        cam_stream_type_t stream_type,
        cam_feature_mask_t postprocess_mask,
        QCamera3Channel *metadataChannel,
        uint32_t numBuffers) :
            QCamera3ProcessingChannel(cam_handle, channel_handle, cam_ops,
                    cb_routine, cb_buffer_err, paddingInfo, userData, stream, stream_type,
                    postprocess_mask, metadataChannel, numBuffers),
            mBatchSize(0),
            mRotation(ROTATE_0),
            mAuxChannel(NULL)
{
    cam_dimension_t dim = {0,0};
    dim.width = stream->width;
    dim.height = stream->height;

    QCamera3HardwareInterface *hal_obj = (QCamera3HardwareInterface *)mUserData;
    if(is_dual_camera_by_handle(cam_handle)
        && (stream_type == CAM_STREAM_TYPE_SNAPSHOT)
        && hal_obj->isAsymetricDim(dim))
    {
        m_camHandle = get_main_camera_handle(cam_handle);
        m_handle = get_main_camera_handle(channel_handle);
        mAuxChannel = new QCamera3RegularChannel(get_aux_camera_handle(cam_handle),
                      get_aux_camera_handle(channel_handle),cam_ops, cb_routine,
                      cb_buffer_err, paddingInfo, userData, stream, stream_type,
                      postprocess_mask, metadataChannel, numBuffers);
        setDualChannelMode(true);
    }

}

/*===========================================================================
 * FUNCTION   : ~QCamera3RegularChannel
 *
 * DESCRIPTION: destructor of QCamera3RegularChannel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3RegularChannel::~QCamera3RegularChannel()
{
    destroy();
    if(NULL != mAuxChannel)
    {
        delete mAuxChannel;
        mAuxChannel = NULL;
    }
}

void QCamera3RegularChannel::setDualChannelMode(bool bMode)
{
    QCamera3Channel::setDualChannelMode(bMode);
    if(NULL != mAuxChannel)
    {
        mAuxChannel->setDualChannelMode(bMode);
    }
}

/*===========================================================================
 * FUNCTION   : initialize
 *
 * DESCRIPTION: Initialize and add camera channel & stream
 *
 * PARAMETERS :
 *    @isType : type of image stabilization required on this stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/

int32_t QCamera3RegularChannel::initialize(cam_is_type_t isType)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_REG_CH_INIT);
    int32_t rc = NO_ERROR;

    cam_dimension_t streamDim;

    if (NULL == mCamera3Stream) {
        LOGE("Camera stream uninitialized");
        return NO_INIT;
    }

    if (1 <= m_numStreams) {
        // Only one stream per channel supported in v3 Hal
        return NO_ERROR;
    }

    mIsType  = isType;

    rc = translateStreamTypeAndFormat(mCamera3Stream, mStreamType,
            mStreamFormat);
    if (rc != NO_ERROR) {
        return -EINVAL;
    }


    if ((mStreamType == CAM_STREAM_TYPE_VIDEO) ||
            (mStreamType == CAM_STREAM_TYPE_PREVIEW)) {
        if ((mCamera3Stream->rotation != CAMERA3_STREAM_ROTATION_0) &&
                ((mPostProcMask & CAM_QCOM_FEATURE_ROTATION) == 0)) {
            LOGE("attempting rotation %d when rotation is disabled",
                    mCamera3Stream->rotation);
            return -EINVAL;
        }

        switch (mCamera3Stream->rotation) {
            case CAMERA3_STREAM_ROTATION_0:
                mRotation = ROTATE_0;
                break;
            case CAMERA3_STREAM_ROTATION_90: {
                mRotation = ROTATE_90;
                break;
            }
            case CAMERA3_STREAM_ROTATION_180:
                mRotation = ROTATE_180;
                break;
            case CAMERA3_STREAM_ROTATION_270: {
                mRotation = ROTATE_270;
                break;
            }
            default:
                LOGE("Unknown rotation: %d",
                         mCamera3Stream->rotation);
            return -EINVAL;
        }

        // Camera3/HAL3 spec expecting counter clockwise rotation but CPP HW is
        // doing Clockwise rotation and so swap it.
        if (mRotation == ROTATE_90) {
            mRotation = ROTATE_270;
        } else if (mRotation == ROTATE_270) {
            mRotation = ROTATE_90;
        }

    } else if (mCamera3Stream->rotation != CAMERA3_STREAM_ROTATION_0) {
        LOGE("Rotation %d is not supported by stream type %d",
                mCamera3Stream->rotation,
                mStreamType);
        return -EINVAL;
    }

    streamDim.width = mCamera3Stream->width;
    streamDim.height = mCamera3Stream->height;

    //In DualCamera usecase for Asymetric mode
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if(m_bDualChannel)
    {
         //if 2 channels are configure with different dimensions
        //for low configuration don't change the stream dimension.
        bool needUpScale;
        hal_obj->rectifyStreamDimIfNeeded(
                streamDim, (mAuxChannel != NULL) ? CAM_TYPE_MAIN: CAM_TYPE_AUX, needUpScale);
    }

    LOGD("batch size is %d", mBatchSize);
    rc = QCamera3Channel::addStream(mStreamType,
            mStreamFormat,
            streamDim,
            mRotation,
            mNumBufs,
            mPostProcMask,
            mIsType,
            mBatchSize);

    if(NULL != mAuxChannel)
    {
        mAuxChannel->initialize(isType);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : setBatchSize
 *
 * DESCRIPTION: Set batch size for the channel.
 *
 * PARAMETERS :
 *   @batchSize  : Number of image buffers in a batch
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success always
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3RegularChannel::setBatchSize(uint32_t batchSize)
{
    int32_t rc = NO_ERROR;

    mBatchSize = batchSize;
    LOGD("Batch size set: %d", mBatchSize);
    if(NULL != mAuxChannel) {
        mAuxChannel->setBatchSize(batchSize);
    }
    return rc;
}

void QCamera3RegularChannel::overridePPConfig(cam_feature_mask_t pp_mask)
{
    LOGD("overriding ppmask to %" PRIx64, pp_mask);
    mPostProcMask = pp_mask;
    if(NULL != mAuxChannel)
    {
        mAuxChannel->overridePPConfig(pp_mask);
    }
}

/*===========================================================================
 * FUNCTION   : getStreamTypeMask
 *
 * DESCRIPTION: Get bit mask of all stream types in this channel.
 *              If stream is not initialized, then generate mask based on
 *              local streamType
 *
 * PARAMETERS : None
 *
 * RETURN     : Bit mask of all stream types in this channel
 *==========================================================================*/
uint32_t QCamera3RegularChannel::getStreamTypeMask()
{
    if (mStreams[0]) {
        return QCamera3Channel::getStreamTypeMask();
    } else {
        return (1U << mStreamType);
    }
}

/*===========================================================================
 * FUNCTION   : queueBatchBuf
 *
 * DESCRIPTION: queue batch container to downstream
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success always
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3RegularChannel::queueBatchBuf()
{
    int32_t rc = NO_ERROR;

    if (mStreams[0]) {
        rc = mStreams[0]->queueBatchBuf();
    }
    if (rc != NO_ERROR) {
        LOGE("stream->queueBatchContainer failed");
    }

    if(NULL != mAuxChannel)
        rc = mAuxChannel->queueBatchBuf();
    return rc;
}

/*===========================================================================
 * FUNCTION   : request
 *
 * DESCRIPTION: process a request from camera service. Stream on if ncessary.
 *
 * PARAMETERS :
 *   @buffer  : buffer to be filled for this request
 *
 * RETURN     : 0 on a success start of capture
 *              -EINVAL on invalid input
 *              -ENODEV on serious error
 *==========================================================================*/
int32_t QCamera3RegularChannel::request(buffer_handle_t *buffer, uint32_t frameNumber, int &indexUsed)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_REG_CH_REQ);
    //FIX ME: Return buffer back in case of failures below.

    int32_t rc = NO_ERROR;
    int index;

    if (NULL == buffer) {
        LOGE("Invalid buffer in channel request");
        return BAD_VALUE;
    }

    bool bIsMaster = true;
    if(m_bDualChannel)
    {
        bIsMaster = (mAuxChannel != NULL) ? (mMasterCam == CAM_TYPE_MAIN)
                                           : (mMasterCam == CAM_TYPE_AUX);
    }
    if(bIsMaster)
    {
        if(!m_bIsActive) {
            rc = registerBuffer(buffer, mIsType);
            if (NO_ERROR != rc) {
                LOGE("On-the-fly buffer registration failed %d",
                         rc);
                return rc;
            }

            rc = start();
            if (NO_ERROR != rc) {
                return rc;
            }
        } else {
            LOGD("Request on an existing stream");
        }

        index = mMemory.getMatchBufIndex((void*)buffer);
        if(index < 0) {
            rc = registerBuffer(buffer, mIsType);
            if (NO_ERROR != rc) {
                LOGE("On-the-fly buffer registration failed %d",
                         rc);
                return rc;
            }

            index = mMemory.getMatchBufIndex((void*)buffer);
            if (index < 0) {
                LOGE("Could not find object among registered buffers");
                return DEAD_OBJECT;
            }
        }

        rc = mMemory.markFrameNumber((uint32_t)index, frameNumber);
        if(rc != NO_ERROR) {
            LOGE("Failed to mark FrameNumber:%d,idx:%d",frameNumber,index);
            return rc;
        }
        rc = mStreams[0]->bufDone((uint32_t)index);
        if(rc != NO_ERROR) {
            LOGE("Failed to Q new buffer to stream");
            mMemory.markFrameNumber(index, -1);
            return rc;
        }

        indexUsed = index;
    } else if(NULL != mAuxChannel) {
        mAuxChannel->request(buffer, frameNumber, indexUsed);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : getReprocessType
 *
 * DESCRIPTION: get the type of reprocess output supported by this channel
 *
 * PARAMETERS : NONE
 *
 * RETURN     : reprocess_type_t : type of reprocess
 *==========================================================================*/
reprocess_type_t QCamera3RegularChannel::getReprocessType()
{
    return REPROCESS_TYPE_PRIVATE;
}

void QCamera3RegularChannel::switchMaster(uint32_t masterCam)
{
    if(m_bDualChannel)
    {
        mMasterCam = masterCam;
        if(NULL != mAuxChannel)
            mAuxChannel->switchMaster(masterCam);
    } else {
        QCamera3Channel::switchMaster(masterCam);
    }
}

/*===========================================================================
 * FUNCTION   : setBundleInfo
 *
 * DESCRIPTION: setting bundle information in stream params
 *
 * PARAMETERS :
 *   @bundleInfo  : stream bundle information.
 *   @cam_type    : MAIN or AUX.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3RegularChannel::setBundleInfo(
                const cam_bundle_config_t &bundleInfo, uint32_t cam_type)
{
    if(m_bDualChannel)
    {
        if ((cam_type == CAM_TYPE_AUX) && (NULL != mAuxChannel)) {
            return mAuxChannel->setBundleInfo(bundleInfo);
        }
        int32_t rc = NO_ERROR;
        cam_stream_parm_buffer_t param;
        memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
        param.type = CAM_STREAM_PARAM_TYPE_SET_BUNDLE_INFO;
        param.bundleInfo = bundleInfo;
        if (m_numStreams > 0 && mStreams[0]) {
            rc = mStreams[0]->setParameter(param, cam_type);
            if (rc != NO_ERROR) {
                LOGE("stream setParameter for set bundle failed");
            }
        }
        return rc;
    }else {
        return QCamera3Channel::setBundleInfo(bundleInfo, cam_type);
    }

}


QCamera3MetadataChannel::QCamera3MetadataChannel(uint32_t cam_handle,
                    uint32_t channel_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    channel_cb_buffer_err cb_buffer_err,
                    cam_padding_info_t *paddingInfo,
                    cam_feature_mask_t postprocess_mask,
                    void *userData, uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, channel_handle, cam_ops,
                                cb_routine, cb_buffer_err, paddingInfo, postprocess_mask,
                                userData, numBuffers),
                        mMemory(NULL)
{
}

QCamera3MetadataChannel::~QCamera3MetadataChannel()
{
    destroy();

    if (mMemory) {
        mMemory->deallocate();
        delete mMemory;
        mMemory = NULL;
    }
}

int32_t QCamera3MetadataChannel::initialize(cam_is_type_t isType)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_METADATA_CH_INIT);
    int32_t rc;
    cam_dimension_t streamDim;

    if (mMemory || m_numStreams > 0) {
        LOGE("metadata channel already initialized");
        return -EINVAL;
    }

    streamDim.width = (int32_t)sizeof(metadata_buffer_t),
    streamDim.height = 1;

    mIsType = isType;
    rc = QCamera3Channel::addStream(CAM_STREAM_TYPE_METADATA, CAM_FORMAT_MAX,
            streamDim, ROTATE_0, (uint8_t)mNumBuffers, mPostProcMask, mIsType);
    if (rc < 0) {
        LOGE("addStream failed");
    }
    return rc;
}

int32_t QCamera3MetadataChannel::request(buffer_handle_t * /*buffer*/,
                                                uint32_t /*frameNumber*/,
                                                int&  /*indexUsed*/)
{
    if (!m_bIsActive) {
        return start();
    }
    else
        return 0;
}

void QCamera3MetadataChannel::streamCbRoutine(
                        mm_camera_super_buf_t *super_frame,
                        QCamera3Stream * /*stream*/)
{
    ATRACE_NAME("metadata_stream_cb_routine");
    uint32_t requestNumber = 0;
    if (super_frame == NULL || super_frame->num_bufs != 1) {
        LOGE("super_frame is not valid");
        return;
    }
    if (mChannelCB) {
        mChannelCB(super_frame, NULL, requestNumber, false, mUserData);
    }
}

QCamera3StreamMem* QCamera3MetadataChannel::getStreamBufs(uint32_t len)
{
    int rc;
    if (len < sizeof(metadata_buffer_t)) {
        LOGE("Metadata buffer size less than structure %d vs %d",
                len,
                sizeof(metadata_buffer_t));
        return NULL;
    }

    uint32_t numBufs = MIN_STREAMING_BUFFER_NUM;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if (hal_obj && hal_obj->isDualCamera()) {
        //Allocate equal no. of meta buffers for aux session
        numBufs += MIN_STREAMING_BUFFER_NUM;
    }

    mMemory = new QCamera3StreamMem(numBufs);
    if (!mMemory) {
        LOGE("unable to create metadata memory");
        return NULL;
    }
    rc = mMemory->allocateAll(len);
    if (rc < 0) {
        LOGE("unable to allocate metadata memory");
        delete mMemory;
        mMemory = NULL;
        return NULL;
    }
    clear_metadata_buffer((metadata_buffer_t*)mMemory->getPtr(0));
    return mMemory;
}

void QCamera3MetadataChannel::putStreamBufs()
{
    mMemory->deallocate();
    delete mMemory;
    mMemory = NULL;
}

/*************************************************************************************/
// DEPTH Channel related functions
QCamera3DepthChannel::QCamera3DepthChannel(uint32_t cam_handle,
                    uint32_t channel_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    channel_cb_buffer_err cb_buffer_err,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    camera3_stream_t *stream,
                    cam_feature_mask_t postprocess_mask,
                    QCamera3Channel *metadataChannel,
                    int32_t numDepthPoints, uint32_t numBuffers) :
                        QCamera3RegularChannel(cam_handle, channel_handle, cam_ops,
                                cb_routine, cb_buffer_err, paddingInfo, userData, stream,
                                CAM_STREAM_TYPE_DEPTH, postprocess_mask,
                                metadataChannel, numBuffers)
{
    mNumDepthPoints = numDepthPoints;
}

QCamera3DepthChannel::~QCamera3DepthChannel()
{
}

/*===========================================================================
 * FUNCTION   : initialize
 *
 * DESCRIPTION: Initialize and add camera channel & stream
 *
 * PARAMETERS :
 * @isType    : image stabilization type on the stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/

int32_t QCamera3DepthChannel::initialize(cam_is_type_t isType)
{
    return QCamera3RegularChannel::initialize(isType);
}

void QCamera3DepthChannel::streamCbRoutine(
                        mm_camera_super_buf_t *super_frame,
                        QCamera3Stream * stream)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_RAW_CH_STRM_CB);
    //Make sure cache coherence because extra processing is done
    mMemory.cleanInvalidateCache(super_frame->bufs[0]->buf_idx);

    QCamera3RegularChannel::streamCbRoutine(super_frame, stream);
    return;
}

/*===========================================================================
 * FUNCTION   : getReprocessType
 *
 * DESCRIPTION: get the type of reprocess output supported by this channel
 *
 * PARAMETERS : NONE
 *
 * RETURN     : reprocess_type_t : type of reprocess
 *==========================================================================*/
reprocess_type_t QCamera3DepthChannel::getReprocessType()
{
    return REPROCESS_TYPE_NONE;
}

/*************************************************************************************/
// RAW Channel related functions
QCamera3RawChannel::QCamera3RawChannel(uint32_t cam_handle,
                    uint32_t channel_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    channel_cb_buffer_err cb_buffer_err,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    camera3_stream_t *stream,
                    cam_feature_mask_t postprocess_mask,
                    QCamera3Channel *metadataChannel,
                    bool raw_16, uint32_t numBuffers) :
                        QCamera3RegularChannel(cam_handle, channel_handle, cam_ops,
                                cb_routine, cb_buffer_err, paddingInfo, userData, stream,
                                CAM_STREAM_TYPE_RAW, postprocess_mask,
                                metadataChannel, numBuffers),
                        mIsRaw16(raw_16)
{
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.vendor.camera.raw.debug.dump", prop, "0");
    mRawDump = atoi(prop);
}

QCamera3RawChannel::~QCamera3RawChannel()
{
}

/*===========================================================================
 * FUNCTION   : initialize
 *
 * DESCRIPTION: Initialize and add camera channel & stream
 *
 * PARAMETERS :
 * @isType    : image stabilization type on the stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/

int32_t QCamera3RawChannel::initialize(cam_is_type_t isType)
{
    return QCamera3RegularChannel::initialize(isType);
}

void QCamera3RawChannel::streamCbRoutine(
                        mm_camera_super_buf_t *super_frame,
                        QCamera3Stream * stream)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_RAW_CH_STRM_CB);
    /* Move this back down once verified */
    if (mRawDump)
        dumpRawSnapshot(super_frame->bufs[0]);

    if (mIsRaw16) {
        cam_format_t streamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_RAW,
                mCamera3Stream->width, mCamera3Stream->height);
        if (streamFormat >= CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG &&
            streamFormat <= CAM_FORMAT_BAYER_MIPI_RAW_10BPP_BGGR)
            convertMipiToRaw16(super_frame->bufs[0]);
        else
            convertLegacyToRaw16(super_frame->bufs[0]);
    }

    //Make sure cache coherence because extra processing is done
    mMemory.cleanInvalidateCache(super_frame->bufs[0]->buf_idx);

    QCamera3RegularChannel::streamCbRoutine(super_frame, stream);
    return;
}

void QCamera3RawChannel::dumpRawSnapshot(mm_camera_buf_def_t *frame)
{
   QCamera3Stream *stream = getStreamByIndex(0);
   if (stream != NULL) {
       char buf[FILENAME_MAX];
       memset(buf, 0, sizeof(buf));
       cam_dimension_t dim;
       memset(&dim, 0, sizeof(dim));
       stream->getFrameDimension(dim);

       cam_frame_len_offset_t offset;
       memset(&offset, 0, sizeof(cam_frame_len_offset_t));
       stream->getFrameOffset(offset);
       snprintf(buf, sizeof(buf), QCAMERA_DUMP_FRM_LOCATION"r_%d_%dx%d.raw",
                frame->frame_idx, offset.mp[0].stride, offset.mp[0].scanline);

       int file_fd = open(buf, O_RDWR| O_CREAT, 0644);
       if (file_fd >= 0) {
          ssize_t written_len = write(file_fd, frame->buffer, frame->frame_len);
          LOGD("written number of bytes %zd", written_len);
          frame->cache_flags |= CPU_HAS_READ;
          close(file_fd);
       } else {
          LOGE("failed to open file to dump image");
       }
   } else {
       LOGE("Could not find stream");
   }

}

void QCamera3RawChannel::convertLegacyToRaw16(mm_camera_buf_def_t *frame)
{
    // Convert image buffer from Opaque raw format to RAW16 format
    // 10bit Opaque raw is stored in the format of:
    // 0000 - p5 - p4 - p3 - p2 - p1 - p0
    // where p0 to p5 are 6 pixels (each is 10bit)_and most significant
    // 4 bits are 0s. Each 64bit word contains 6 pixels.

  QCamera3Stream *stream = getStreamByIndex(0);
  if (stream != NULL) {
      cam_dimension_t dim;
      memset(&dim, 0, sizeof(dim));
      stream->getFrameDimension(dim);

      cam_frame_len_offset_t offset;
      memset(&offset, 0, sizeof(cam_frame_len_offset_t));
      stream->getFrameOffset(offset);

      uint32_t raw16_stride = ((uint32_t)dim.width + 15U) & ~15U;
      uint16_t* raw16_buffer = (uint16_t *)frame->buffer;

      // In-place format conversion.
      // Raw16 format always occupy more memory than opaque raw10.
      // Convert to Raw16 by iterating through all pixels from bottom-right
      // to top-left of the image.
      // One special notes:
      // 1. Cross-platform raw16's stride is 16 pixels.
      // 2. Opaque raw10's stride is 6 pixels, and aligned to 16 bytes.
      for (int32_t ys = dim.height - 1; ys >= 0; ys--) {
          uint32_t y = (uint32_t)ys;
          uint64_t* row_start = (uint64_t *)frame->buffer +
                  y * (uint32_t)offset.mp[0].stride_in_bytes / 8;
          for (int32_t xs = dim.width - 1; xs >= 0; xs--) {
              uint32_t x = (uint32_t)xs;
              uint16_t raw16_pixel = 0x3FF & (row_start[x/6] >> (10*(x%6)));
              raw16_buffer[y*raw16_stride+x] = raw16_pixel;
          }
      }
  } else {
      LOGE("Could not find stream");
  }

}

void QCamera3RawChannel::convertMipiToRaw16(mm_camera_buf_def_t *frame)
{
    // Convert image buffer from mipi10 raw format to RAW16 format
    // mipi10 opaque raw is stored in the format of:
    // P3(1:0) P2(1:0) P1(1:0) P0(1:0) P3(9:2) P2(9:2) P1(9:2) P0(9:2)
    // 4 pixels occupy 5 bytes, no padding needed

    QCamera3Stream *stream = getStreamByIndex(0);
    if (stream != NULL) {
        cam_dimension_t dim;
        memset(&dim, 0, sizeof(dim));
        stream->getFrameDimension(dim);

        cam_frame_len_offset_t offset;
        memset(&offset, 0, sizeof(cam_frame_len_offset_t));
        stream->getFrameOffset(offset);

        uint32_t raw16_stride = ((uint32_t)dim.width + 15U) & ~15U;
        uint16_t* raw16_buffer = (uint16_t *)frame->buffer;

        // In-place format conversion.
        // Raw16 format always occupy more memory than opaque raw10.
        // Convert to Raw16 by iterating through all pixels from bottom-right
        // to top-left of the image.
        // One special notes:
        // 1. Cross-platform raw16's stride is 16 pixels.
        // 2. mipi raw10's stride is 4 pixels, and aligned to 16 bytes.
        for (int32_t ys = dim.height - 1; ys >= 0; ys--) {
            uint32_t y = (uint32_t)ys;
            uint8_t* row_start = (uint8_t *)frame->buffer +
                    y * (uint32_t)offset.mp[0].stride_in_bytes;
            for (int32_t xs = dim.width - 1; xs >= 0; xs--) {
                uint32_t x = (uint32_t)xs;
                uint8_t upper_8bit = row_start[5*(x/4)+x%4];
                uint8_t lower_2bit = ((row_start[5*(x/4)+4] >> (x%4)) & 0x3);
                uint16_t raw16_pixel =
                        (uint16_t)(((uint16_t)upper_8bit)<<2 |
                        (uint16_t)lower_2bit);
                raw16_buffer[y*raw16_stride+x] = raw16_pixel;
            }
        }
    } else {
        LOGE("Could not find stream");
    }

}

/*===========================================================================
 * FUNCTION   : getReprocessType
 *
 * DESCRIPTION: get the type of reprocess output supported by this channel
 *
 * PARAMETERS : NONE
 *
 * RETURN     : reprocess_type_t : type of reprocess
 *==========================================================================*/
reprocess_type_t QCamera3RawChannel::getReprocessType()
{
    return REPROCESS_TYPE_RAW;
}


/*************************************************************************************/
// RAW Dump Channel related functions

/*===========================================================================
 * FUNCTION   : QCamera3RawDumpChannel
 *
 * DESCRIPTION: Constructor for RawDumpChannel
 *
 * PARAMETERS :
 *   @cam_handle    : Handle for Camera
 *   @cam_ops       : Function pointer table
 *   @rawDumpSize   : Dimensions for the Raw stream
 *   @paddinginfo   : Padding information for stream
 *   @userData      : Cookie for parent
 *   @pp mask       : PP feature mask for this stream
 *   @numBuffers    : number of max dequeued buffers
 *
 * RETURN           : NA
 *==========================================================================*/
QCamera3RawDumpChannel::QCamera3RawDumpChannel(uint32_t cam_handle,
                    uint32_t channel_handle,
                    mm_camera_ops_t *cam_ops,
                    cam_dimension_t rawDumpSize,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    cam_feature_mask_t postprocess_mask, uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, channel_handle, cam_ops, NULL,
                                NULL, paddingInfo, postprocess_mask,
                                userData, numBuffers),
                        mDim(rawDumpSize),
                        mMemory(NULL)
{
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.vendor.camera.raw.dump", prop, "0");
    mRawDump = atoi(prop);
}

/*===========================================================================
 * FUNCTION   : QCamera3RawDumpChannel
 *
 * DESCRIPTION: Destructor for RawDumpChannel
 *
 * PARAMETERS :
 *
 * RETURN           : NA
 *==========================================================================*/

QCamera3RawDumpChannel::~QCamera3RawDumpChannel()
{
    destroy();
}

/*===========================================================================
 * FUNCTION   : dumpRawSnapshot
 *
 * DESCRIPTION: Helper function to dump Raw frames
 *
 * PARAMETERS :
 *  @frame      : stream buf frame to be dumped
 *
 *  RETURN      : NA
 *==========================================================================*/
void QCamera3RawDumpChannel::dumpRawSnapshot(mm_camera_buf_def_t *frame)
{
    QCamera3Stream *stream = getStreamByIndex(0);
    if (stream != NULL) {
        char buf[FILENAME_MAX];
        struct timeval tv;
        struct tm timeinfo_data;
        struct tm *timeinfo;

        cam_dimension_t dim;
        memset(&dim, 0, sizeof(dim));
        stream->getFrameDimension(dim);

        cam_frame_len_offset_t offset;
        memset(&offset, 0, sizeof(cam_frame_len_offset_t));
        stream->getFrameOffset(offset);

        gettimeofday(&tv, NULL);
        timeinfo = localtime_r(&tv.tv_sec, &timeinfo_data);

        if (NULL != timeinfo) {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf),
                    QCAMERA_DUMP_FRM_LOCATION
                    "%04d-%02d-%02d-%02d-%02d-%02d-%06ld_%d_%dx%d.raw",
                    timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
                    timeinfo->tm_mday, timeinfo->tm_hour,
                    timeinfo->tm_min, timeinfo->tm_sec,tv.tv_usec,
                    frame->frame_idx, dim.width, dim.height);

            int file_fd = open(buf, O_RDWR| O_CREAT, 0777);
            if (file_fd >= 0) {
                ssize_t written_len =
                        write(file_fd, frame->buffer, offset.frame_len);
                LOGD("written number of bytes %zd", written_len);
                frame->cache_flags |= CPU_HAS_READ;
                close(file_fd);
            } else {
                LOGE("failed to open file to dump image");
            }
        } else {
            LOGE("localtime_r() error");
        }
    } else {
        LOGE("Could not find stream");
    }

}

/*===========================================================================
 * FUNCTION   : streamCbRoutine
 *
 * DESCRIPTION: Callback routine invoked for each frame generated for
 *              Rawdump channel
 *
 * PARAMETERS :
 *   @super_frame  : stream buf frame generated
 *   @stream       : Underlying Stream object cookie
 *
 * RETURN          : NA
 *==========================================================================*/
void QCamera3RawDumpChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                                                __unused QCamera3Stream *stream)
{
    LOGD("E");
    if (super_frame == NULL || super_frame->num_bufs != 1) {
        LOGE("super_frame is not valid");
        return;
    }

    if (mRawDump)
        dumpRawSnapshot(super_frame->bufs[0]);

    bufDone(super_frame);
    free(super_frame);
}

/*===========================================================================
 * FUNCTION   : getStreamBufs
 *
 * DESCRIPTION: Callback function provided to interface to get buffers.
 *
 * PARAMETERS :
 *   @len       : Length of each buffer to be allocated
 *
 * RETURN     : NULL on buffer allocation failure
 *              QCamera3StreamMem object on sucess
 *==========================================================================*/
QCamera3StreamMem* QCamera3RawDumpChannel::getStreamBufs(uint32_t len)
{
    int rc;
    mMemory = new QCamera3StreamMem(mNumBuffers);

    if (!mMemory) {
        LOGE("unable to create heap memory");
        return NULL;
    }
    rc = mMemory->allocateAll((size_t)len);
    if (rc < 0) {
        LOGE("unable to allocate heap memory");
        delete mMemory;
        mMemory = NULL;
        return NULL;
    }
    return mMemory;
}

/*===========================================================================
 * FUNCTION   : putStreamBufs
 *
 * DESCRIPTION: Callback function provided to interface to return buffers.
 *              Although no handles are actually returned, implicitl assumption
 *              that interface will no longer use buffers and channel can
 *              deallocated if necessary.
 *
 * PARAMETERS : NA
 *
 * RETURN     : NA
 *==========================================================================*/
void QCamera3RawDumpChannel::putStreamBufs()
{
    mMemory->deallocate();
    delete mMemory;
    mMemory = NULL;
}

/*===========================================================================
 * FUNCTION : request
 *
 * DESCRIPTION: Request function used as trigger
 *
 * PARAMETERS :
 * @recvd_frame : buffer- this will be NULL since this is internal channel
 * @frameNumber : Undefined again since this is internal stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3RawDumpChannel::request(buffer_handle_t * /*buffer*/,
                                                uint32_t /*frameNumber*/,
                                                int & /*indexUsed*/)
{
    if (!m_bIsActive) {
        return QCamera3Channel::start();
    }
    else
        return 0;
}

/*===========================================================================
 * FUNCTION : intialize
 *
 * DESCRIPTION: Initializes channel params and creates underlying stream
 *
 * PARAMETERS :
 *    @isType : type of image stabilization required on this stream
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3RawDumpChannel::initialize(cam_is_type_t isType)
{
    int32_t rc;

    mIsType = isType;
    rc = QCamera3Channel::addStream(CAM_STREAM_TYPE_RAW,
        CAM_FORMAT_BAYER_MIPI_RAW_10BPP_GBRG, mDim, ROTATE_0, (uint8_t)mNumBuffers,
        mPostProcMask, mIsType);
    if (rc < 0) {
        LOGE("addStream failed");
    }
    return rc;
}

/*************************************************************************************/
// Quadra CFA Raw Channel related functions

QCamera3QCfaRawChannel::QCamera3QCfaRawChannel(uint32_t cam_handle,
                    uint32_t channel_handle,
                    mm_camera_ops_t *cam_ops,
                    cam_dimension_t rawDumpSize,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    cam_feature_mask_t postprocess_mask, uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, channel_handle, cam_ops, NULL,
                                NULL, paddingInfo, postprocess_mask,
                                userData, numBuffers),
                        mDim(rawDumpSize),
                        mMemory(NULL),
                        m_metaMem(NULL),
                        m_frameNumber(0)
{
    svr_stream_id = 0;

    raw_received = false;
    memset(&raw_frame, 0, sizeof(mm_camera_super_buf_t));

    meta_received = false;
    memset(&meta_frame, 0, sizeof(mm_camera_super_buf_t));

    cam_sem_init(&m_syncSem, 0);
}

QCamera3QCfaRawChannel::~QCamera3QCfaRawChannel()
{
    destroy();
    if (mMemory != NULL) {
        LOGD("deallocate stream buffers");
        mMemory->deallocate();
        delete mMemory;
        mMemory = NULL;
    }
    if (m_metaMem != NULL) {
        LOGD("deallocate local meta buffer");
        m_metaMem->deallocate();
        delete m_metaMem;
        m_metaMem = NULL;
    }

    cam_sem_destroy(&m_syncSem);
}

void QCamera3QCfaRawChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                                                __unused QCamera3Stream *stream)
{
    LOGD("E");
    if (super_frame == NULL || super_frame->num_bufs != 1) {
        LOGE("super_frame is not valid");
        return;
    }

    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if (hal_obj->m_bQuadraCfaRequest) {
        LOGD("store quadra cfa raw frame");
        raw_frame = *super_frame;
        raw_buf_def = *(super_frame->bufs[0]);
        raw_frame.bufs[0] = &raw_buf_def;
        raw_received = true;

        if (raw_received && meta_received) {
            notifyCaptureDone();
        }
    } else {
        bufDone(super_frame);
    }

    free(super_frame);
}

QCamera3StreamMem* QCamera3QCfaRawChannel::getStreamBufs(uint32_t len)
{
    int rc;
    mMemory = new QCamera3StreamMem(mNumBuffers);
    if (!mMemory) {
        LOGE("unable to create heap memory");
        return NULL;
    }
    rc = mMemory->allocateAll((size_t)len);
    if (rc < 0) {
        LOGE("unable to allocate heap memory");
        delete mMemory;
        mMemory = NULL;
    }
    return mMemory;
}

void QCamera3QCfaRawChannel::putStreamBufs()
{
    LOGD("E. don't release the memory.");
}

int32_t QCamera3QCfaRawChannel::request(buffer_handle_t *, uint32_t frameNumber, int &)
{
    int32_t rc = 0;
    if (!m_bIsActive) {
        rc = QCamera3Channel::start();
    }

    LOGD("frame number:%d", frameNumber);
    m_frameNumber = frameNumber;

    return rc;
}

int32_t QCamera3QCfaRawChannel::initialize(cam_is_type_t isType)
{
    int32_t rc;

    mIsType = isType;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    rc = QCamera3Channel::addStream(CAM_STREAM_TYPE_RAW,
        hal_obj->mRdiModeFmt, mDim, ROTATE_0, (uint8_t)mNumBuffers,
        mPostProcMask, mIsType);
    if (rc < 0) {
        LOGE("addStream failed");
    }

    svr_stream_id = mStreams[0]->getMyServerID();
    LOGD("stream server id:%d", svr_stream_id);
    return rc;
}

int32_t QCamera3QCfaRawChannel::queueReprocMetadata(mm_camera_super_buf_t *metadata,
                                        cam_frame_len_offset_t &offset,  bool urgent)
{
    LOGD("E. urgen meta:%d", urgent);
    metadata_buffer_t *p_metadata = (metadata_buffer_t *)(metadata->bufs[0]->buffer);
    uint32_t *p_frame_number = POINTER_OF_META(CAM_INTF_META_FRAME_NUMBER, p_metadata);

    if (urgent) {
        uint32_t *p_urgent_frame_number =
            POINTER_OF_META(CAM_INTF_META_URGENT_FRAME_NUMBER, p_metadata);
        if (p_urgent_frame_number == NULL || *p_urgent_frame_number != m_frameNumber) {
            LOGE("frame number not match!");
            return -1;
        }
        memcpy(&urgent_meta, p_metadata, sizeof(metadata_buffer_t));
    } else {
        if (p_frame_number == NULL || *p_frame_number != m_frameNumber) {
            return -1;
        }

        /* allocate meta buffre here and copy the content of metadata,
         * as the original meta channel will be deleted
         */
        m_metaMem = new QCamera3StreamMem(1);
        if (!m_metaMem) {
            LOGE("unable to create memory for meta");
            return -1;
        }
        m_metaMem->allocateAll((size_t)metadata->bufs[0]->frame_len);
        m_metaMem->getBufDef(offset, meta_buf, 0);
        memcpy(meta_buf.buffer, metadata->bufs[0]->buffer, sizeof(metadata_buffer_t));

        meta_frame = *metadata;
        meta_frame.bufs[0] = &meta_buf;

        meta_received = true;
        if (raw_received && meta_received) {
            notifyCaptureDone();
        }
    }

    LOGD("X");
    return 0;
}

void QCamera3QCfaRawChannel::notifyCaptureDone()
{
    cam_sem_post(&m_syncSem);
}

void QCamera3QCfaRawChannel::waitCaptureDone()
{
    cam_sem_wait(&m_syncSem);
}


/*************************************************************************************/

/* QCamera3YUVChannel methods */

/*===========================================================================
 * FUNCTION   : QCamera3YUVChannel
 *
 * DESCRIPTION: constructor of QCamera3YUVChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @cb_routine : callback routine to frame aggregator
 *   @paddingInfo : padding information for the stream
 *   @stream     : camera3_stream_t structure
 *   @stream_type: Channel stream type
 *   @postprocess_mask: the postprocess mask for streams of this channel
 *   @metadataChannel: handle to the metadataChannel
 * RETURN     : none
 *==========================================================================*/
QCamera3YUVChannel::QCamera3YUVChannel(uint32_t cam_handle,
        uint32_t channel_handle,
        mm_camera_ops_t *cam_ops,
        channel_cb_routine cb_routine,
        channel_cb_buffer_err cb_buf_err,
        cam_padding_info_t *paddingInfo,
        void *userData,
        camera3_stream_t *stream,
        cam_stream_type_t stream_type,
        cam_feature_mask_t postprocess_mask,
        QCamera3Channel *metadataChannel) :
            QCamera3ProcessingChannel(cam_handle, channel_handle, cam_ops,
                    cb_routine, cb_buf_err, paddingInfo, userData, stream, stream_type,
                    postprocess_mask, metadataChannel),
                mAuxYUVChannel(NULL),
                mNeedPPUpscale(false)
{

    mBypass = (postprocess_mask == CAM_QCOM_FEATURE_NONE);
    mFrameLen = 0;
    mEdgeMode.edge_mode = CAM_EDGE_MODE_OFF;
    mEdgeMode.sharpness = 0;
    mNoiseRedMode = CAM_NOISE_REDUCTION_MODE_OFF;
    lastReturnedFrame = 0;
    memset(&mCropRegion, 0, sizeof(mCropRegion));
    cam_dimension_t dim = {0,0};
    dim.width = stream->width;
    dim.height = stream->height;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if (is_dual_camera_by_handle(cam_handle) && hal_obj->isAsymetricDim(dim)) {
        m_camHandle = get_main_camera_handle(cam_handle);
        m_handle = get_main_camera_handle(channel_handle);
        mAuxYUVChannel = new QCamera3YUVChannel(get_aux_camera_handle(cam_handle),
                    get_aux_camera_handle(channel_handle), cam_ops,
                    cb_routine, cb_buf_err,
                    paddingInfo, userData,
                    stream, stream_type,
                    postprocess_mask,
                    metadataChannel);
        setDualChannelMode(true);
    }
}

/*===========================================================================
 * FUNCTION   : ~QCamera3YUVChannel
 *
 * DESCRIPTION: destructor of QCamera3YUVChannel
 *
 * PARAMETERS : none
 *
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3YUVChannel::~QCamera3YUVChannel()
{
   // Deallocation of heap buffers allocated in mMemory is freed
   // automatically by its destructor
   if(NULL != mAuxYUVChannel)
   {
       delete mAuxYUVChannel;
       mAuxYUVChannel = NULL;
   }
}
void QCamera3YUVChannel::setDualChannelMode(bool bMode)
{
    QCamera3Channel::setDualChannelMode(bMode);
    if(NULL != mAuxYUVChannel)
    {
        mAuxYUVChannel->setDualChannelMode(bMode);
    }
}

/*===========================================================================
 * FUNCTION   : initialize
 *
 * DESCRIPTION: Initialize and add camera channel & stream
 *
 * PARAMETERS :
 * @isType    : the image stabilization type
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3YUVChannel::initialize(cam_is_type_t isType)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_YUV_CH_INIT);
    int32_t rc = NO_ERROR;
    cam_dimension_t streamDim;

    if (NULL == mCamera3Stream) {
        LOGE("Camera stream uninitialized");
        return NO_INIT;
    }

    if (1 <= m_numStreams) {
        // Only one stream per channel supported in v3 Hal
        return NO_ERROR;
    }

    mIsType  = isType;
    mStreamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_CALLBACK,
            mCamera3Stream->width, mCamera3Stream->height);

    streamDim.width = mCamera3Stream->width;
    streamDim.height = mCamera3Stream->height;

    //In DualCamera usecase for Asymetric mode
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if(m_bDualChannel)
    {
        //for low configuration don't change the stream dimension.
        hal_obj->rectifyStreamDimIfNeeded(
                streamDim, (mAuxYUVChannel != NULL) ? CAM_TYPE_MAIN: CAM_TYPE_AUX, mNeedPPUpscale);
     }

    rc = QCamera3Channel::addStream(mStreamType,
            mStreamFormat,
            streamDim,
            ROTATE_0,
            mNumBufs,
            mPostProcMask,
            mIsType);
    if (rc < 0) {
        LOGE("addStream failed");
        return rc;
    }

    cam_stream_buf_plane_info_t buf_planes;
    cam_padding_info_t paddingInfo = mPaddingInfo;

    memset(&buf_planes, 0, sizeof(buf_planes));
    //to ensure a big enough buffer size set the height and width
    //padding to max(height padding, width padding)
    paddingInfo.width_padding = MAX(paddingInfo.width_padding, paddingInfo.height_padding);
    paddingInfo.height_padding = paddingInfo.width_padding;

    rc = mm_stream_calc_offset_snapshot(mStreamFormat, &streamDim, &paddingInfo,
            &buf_planes);
    if (rc < 0) {
        LOGE("mm_stream_calc_offset_preview failed");
        return rc;
    }

    mFrameLen = buf_planes.plane_info.frame_len;

    if (NO_ERROR != rc) {
        LOGE("Initialize failed, rc = %d", rc);
        return rc;
    }

    /* initialize offline meta memory for input reprocess */
    rc = QCamera3ProcessingChannel::initialize(isType);
    if (NO_ERROR != rc) {
        LOGE("Processing Channel initialize failed, rc = %d",
                 rc);
    }

    if(NULL != mAuxYUVChannel)
    {
        mAuxYUVChannel->initialize(isType);
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : queueReprocMetadata
 *
 * DESCRIPTION: queue the reprocess metadata to the postprocessor
 *
 * PARAMETERS : metadata : the metadata corresponding to the pp frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3YUVChannel::queueReprocMetadata(mm_camera_super_buf_t *metadata,
                            uint32_t framenum, bool dropFrame)
{
    if((NULL != mAuxYUVChannel) && (metadata->camera_handle != m_camHandle))
    {
        return mAuxYUVChannel->queueReprocMetadata(metadata, framenum, dropFrame);
    } else {
        return QCamera3ProcessingChannel::queueReprocMetadata(metadata, framenum, dropFrame);
    }
    LOGE("Should never be here");
    return -1;
}

/*===========================================================================
 * FUNCTION   : request
 *
 * DESCRIPTION: entry function for a request on a YUV stream. This function
 *              has the logic to service a request based on its type
 *
 * PARAMETERS :
 * @buffer          : pointer to the output buffer
 * @frameNumber     : frame number of the request
 * @pInputBuffer    : pointer to input buffer if an input request
 * @metadata        : parameters associated with the request
 * @internalreq      : boolean to indicate if this is purely internal request
 *                    needing internal buffer allocation
 * @meteringonly    : boolean indicating metering only frame subset of internal
 *                    not consumed by postprocessor
 *
 * RETURN     : 0 on a success start of capture
 *              -EINVAL on invalid input
 *              -ENODEV on serious error
 *==========================================================================*/
int32_t QCamera3YUVChannel::request(buffer_handle_t *buffer,
        uint32_t frameNumber,
        camera3_stream_buffer_t* pInputBuffer,
        metadata_buffer_t* metadata, bool &needMetadata,
        int &indexUsed,
        __unused bool internalRequest = false,
        __unused bool meteringOnly = false)
{
    int32_t rc = NO_ERROR;
    Mutex::Autolock lock(mOfflinePpLock);

    LOGD("pInputBuffer is %p frame number %d", pInputBuffer, frameNumber);
    if (NULL == buffer || NULL == metadata) {
        LOGE("Invalid buffer/metadata in channel request");
        return BAD_VALUE;
    }

    bool bIsMaster = true;
    if(m_bDualChannel)
    {
        bIsMaster = (mAuxYUVChannel != NULL) ? (mMasterCam == CAM_TYPE_MAIN)
                                           : (mMasterCam == CAM_TYPE_AUX);
    }
    if(bIsMaster)
    {
        PpInfo ppInfo;
        memset(&ppInfo, 0, sizeof(ppInfo));
        ppInfo.frameNumber = frameNumber;
        ppInfo.offlinePpFlag = false;
        if (mBypass && !pInputBuffer ) {
            ppInfo.offlinePpFlag = needsFramePostprocessing(metadata);
            ppInfo.output = buffer;
            mOfflinePpInfoList.push_back(ppInfo);
        }

        LOGD("offlinePpFlag %d and mbypass is %d", ppInfo.offlinePpFlag, mBypass);
        needMetadata = ppInfo.offlinePpFlag;
        if (!ppInfo.offlinePpFlag) {
            // regular request
            return QCamera3ProcessingChannel::request(buffer, frameNumber,
                    pInputBuffer, metadata, indexUsed);
        } else {
            if(!m_bIsActive) {
                rc = start();
                if (NO_ERROR != rc)
                    return rc;
            } else {
                LOGD("Request on an existing stream");
            }

            //we need to send this frame through the CPP
            //Allocate heap memory, then buf done on the buffer
            uint32_t bufIdx;
            if (mFreeHeapBufferList.empty()) {
                rc = mMemory.allocateOne(mFrameLen);
                if (rc < 0) {
                    LOGE("Failed allocating heap buffer. Fatal");
                    return BAD_VALUE;
                } else {
                    bufIdx = (uint32_t)rc;
                }
            } else {
                bufIdx = *(mFreeHeapBufferList.begin());
                mFreeHeapBufferList.erase(mFreeHeapBufferList.begin());
            }

            /* Configure and start postproc if necessary */
            reprocess_config_t reproc_cfg;
            cam_dimension_t dim;
            memset(&reproc_cfg, 0, sizeof(reprocess_config_t));
            memset(&dim, 0, sizeof(dim));
            mStreams[0]->getFrameDimension(dim);
            setReprocConfig(reproc_cfg, NULL, metadata, mStreamFormat, dim, mNeedPPUpscale);

            // Start postprocessor without input buffer
            startPostProc(reproc_cfg);

            LOGD("erasing %d", bufIdx);

            mMemory.markFrameNumber(bufIdx, frameNumber);
            indexUsed = bufIdx;
            mStreams[0]->bufDone(bufIdx);

        }
    }else if(NULL != mAuxYUVChannel) {
        mAuxYUVChannel->request(buffer, frameNumber, pInputBuffer, metadata, needMetadata,
                                indexUsed);
    }

    return rc;
}

void QCamera3YUVChannel::switchMaster(uint32_t masterCam)
{
    if(m_bDualChannel)
    {
        mMasterCam = masterCam;
        if(NULL != mAuxYUVChannel)
            mAuxYUVChannel->switchMaster(masterCam);
    } else {
        QCamera3Channel::switchMaster(masterCam);
    }
}

void QCamera3YUVChannel::overridePPConfig(cam_feature_mask_t pp_mask)
{
    LOGD("overriding ppmask to %" PRIx64, pp_mask);
    mPostProcMask = pp_mask;
    if(NULL != mAuxYUVChannel)
    {
        mAuxYUVChannel->overridePPConfig(pp_mask);
    }
}

/*===========================================================================
 * FUNCTION   : setBundleInfo
 *
 * DESCRIPTION: setting bundle information in stream params
 *
 * PARAMETERS :
 *   @bundleInfo  : stream bundle information.
 *   @cam_type    : MAIN or AUX.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3YUVChannel::setBundleInfo(
                const cam_bundle_config_t &bundleInfo, uint32_t cam_type)
{
    if ((cam_type == CAM_TYPE_AUX) && (NULL != mAuxYUVChannel)) {
        return mAuxYUVChannel->setBundleInfo(bundleInfo);
    }
    int32_t rc = NO_ERROR;
    cam_stream_parm_buffer_t param;
    memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
    param.type = CAM_STREAM_PARAM_TYPE_SET_BUNDLE_INFO;
    param.bundleInfo = bundleInfo;
    if (m_numStreams > 0 && mStreams[0]) {
        rc = mStreams[0]->setParameter(param, cam_type);
        if (rc != NO_ERROR) {
            LOGE("stream setParameter for set bundle failed");
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : streamCbRoutine
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 * @super_frame : the super frame with filled buffer
 * @stream      : stream on which the buffer was requested and filled
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3YUVChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
        QCamera3Stream *stream)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_YUV_CH_STRM_CB);
    Mutex::Autolock lock(mYuvCbBufferLock);
    uint8_t frameIndex;
    uint32_t resultFrameNumber;
    bool cancelledBuffer = false, isReturnFrame = false;
    bool reprocNeeded = false;
    buffer_handle_t *output;
    uint32_t frameNum;
    List<PpInfo>::iterator ppInfo;
    if (checkStreamCbErrors(super_frame, stream) != NO_ERROR) {
        LOGE("Error with the stream callback");
        return;
    }

    frameIndex = (uint8_t)super_frame->bufs[0]->buf_idx;
    if(frameIndex >= mNumBufs) {
         LOGE("Error, Invalid index for buffer");
         stream->bufDone(frameIndex);
         return;
    }

    if (mBypass) {
        List<PpInfo>::iterator ppInfoPending;
        List<uint32_t>::iterator ppDropInfo;
        int32_t index;
        resultFrameNumber = mMemory.getFrameNumber(frameIndex);
        if (IS_BUFFER_ERROR(super_frame->bufs[0]->flags)) {
        /* TODO Sending notify to framework - need to send before corresponding full Meta */
            cancelledBuffer = true;
            mChannelCbBufErr(this, resultFrameNumber,
                    CAMERA3_BUFFER_STATUS_ERROR, mUserData);
        }
        Mutex::Autolock lock(mOfflinePpLock);
        LOGD("mBypass result frame num  %d and pplist start frame :%d",
                resultFrameNumber, mOfflinePpInfoList.begin()->frameNumber);
        reprocNeeded = false; isReturnFrame = false;
        for (ppInfo = mOfflinePpInfoList.begin();
                ppInfo != mOfflinePpInfoList.end(); ppInfo++) {
            if (ppInfo->frameNumber == (uint32_t)resultFrameNumber) {
                break;
            }
        }
        LOGD("frame index %d, frame number %d ", frameIndex, resultFrameNumber);
        //check the reprocessing required flag against the frame number
        if (ppInfo == mOfflinePpInfoList.end()) {
            LOGE("Error, request for frame number is a reprocess.");
            stream->bufDone(frameIndex);
            return;
        }
        if(lastReturnedFrame == 0) {
                lastReturnedFrame = resultFrameNumber;
        }
        if (IS_BUFFER_ERROR(super_frame->bufs[0]->flags)) {
            LOGD("Cancelled Buffer ");
            if(ppInfo->offlinePpFlag == true) {
                ppInfo->isReturnBuffer = true;
                int32_t bufferIndex =
                       mMemory.getHeapBufferIndex(resultFrameNumber);
                if(bufferIndex != -1) {
                    mMemory.markFrameNumber(bufferIndex, -1);
                    mFreeHeapBufferList.push_back(bufferIndex);
                    free(super_frame);
                    {
                        Mutex::Autolock lock(mDropReprocBuffersLock);
                        if(m_postprocessor.releaseReprocMetaBuffer(resultFrameNumber)) {
                            LOGD(" Meta has been released");
                        } else {
                            LOGD("Meta hasnt been received for Cancelled buffer");
                            mReprocDropBufferList.push_back(ppInfo->frameNumber);
                        }
                    }
                }
            }
        }
        if(resultFrameNumber > lastReturnedFrame + 1) {
            LOGD(" Detecting out of order %d and %d",
                        lastReturnedFrame, resultFrameNumber);
            for (ppInfoPending = mOfflinePpInfoList.begin();
                    (ppInfoPending != mOfflinePpInfoList.end()); ppInfoPending++) {
                if(ppInfoPending->frameNumber == lastReturnedFrame)
                    break;
            }
            for (ppInfoPending++;
                    (ppInfoPending != mOfflinePpInfoList.end()) &&
                    (ppInfoPending->frameNumber < resultFrameNumber); ppInfoPending++) {
                LOGH(" Detecting out of order in Filed list:%d and %d",
                ppInfoPending->frameNumber,resultFrameNumber);
                index = mMemory.getBufferIndex(ppInfoPending->frameNumber);
                mStreams[0]->cancelBuffer(index);
            }
        }
        if(lastReturnedFrame < resultFrameNumber) {
            lastReturnedFrame = resultFrameNumber;
        }
        if (ppInfo->offlinePpFlag && !cancelledBuffer) {
            mm_camera_super_buf_t *frame =
                    (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
            if (frame == NULL) {
                LOGE("Error allocating memory to save received_frame structure.");
                if(stream) {
                    stream->bufDone(frameIndex);
                }
                return;
            }

            *frame = *super_frame;
            LOGD(" Sending Frame for Reprocessing : %d", resultFrameNumber);
            m_postprocessor.processData(frame, ppInfo->output, resultFrameNumber);
            free(super_frame);
            return;
        } else {
            if (ppInfo != mOfflinePpInfoList.begin()) {
                // There is pending reprocess buffer, cache current buffer
                if (ppInfo->callback_buffer != NULL) {
                    LOGE("Fatal: cached callback_buffer is already present");
                }
                LOGD(" Caching Frame: %d", resultFrameNumber);
                ppInfo->isReturnBuffer = true;
                ppInfo->callback_buffer = super_frame;
                return;
            }
        }
        isReturnFrame = (ppInfo == mOfflinePpInfoList.begin());
        if(isReturnFrame ) {
            if(ppInfo->offlinePpFlag) {
                reprocNeeded = true;
                output = ppInfo->output;
                frameNum = ppInfo->frameNumber;
            }
            mOfflinePpInfoList.erase(ppInfo);
        }
    }
    if((mBypass == true) && (isReturnFrame)) {
        LOGD(" Returning the Buffer");
        if(reprocNeeded) {
            issueChannelCb(output, frameNum);
        } else {
            QCamera3ProcessingChannel::streamCbRoutine(super_frame, stream);
        }
        while(getNextPendingCbBuffer());
    }
    if(mBypass == false) {
        resultFrameNumber = mMemory.getFrameNumber(frameIndex);
        LOGD(" Returning Non bypass Buffer : %d",resultFrameNumber );
        QCamera3ProcessingChannel::streamCbRoutine(super_frame, stream);
    }
    return;
}


/*===========================================================================
 * FUNCTION   : timeoutFrame
 *
 * DESCRIPTION: Method to indicate to channel that a given frame has take too
 *              long to be generated
 *
 * PARAMETERS : framenumber indicating the framenumber of the buffer timingout
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3YUVChannel::timeoutFrame(uint32_t frameNumber)
{
    int32_t yuvbufIdx = 0;
    int32_t rc = NO_ERROR;

    LOGH("E, frameNumber: %d", frameNumber)
    yuvbufIdx = mMemory.getBufferIndex(frameNumber);
    if (yuvbufIdx < 0) {
        LOGE("X: Buffer not found for frame:%d", frameNumber);
        return -1;
    }

    if (mBypass) {
        List<PpInfo>::iterator ppInfo;

        Mutex::Autolock lock(mOfflinePpLock);
        for (ppInfo = mOfflinePpInfoList.begin();
                ppInfo != mOfflinePpInfoList.end(); ppInfo++) {
            if (ppInfo->frameNumber == (uint32_t)frameNumber) {
                break;
            }
        }
        //check the reprocessing required flag against the frame number
        if (ppInfo == mOfflinePpInfoList.end()) {
            LOGD("No reprocess frame for this frame number.");
        } else if (ppInfo->offlinePpFlag) {
            // FixMe : timeout logic update for reprocess buffer
            rc = m_postprocessor.timeoutFrame(frameNumber);
            LOGH("X, calling reprocess timeoutFrame for frame: %d", frameNumber)
            return rc;
        }
    }
    QCamera3ProcessingChannel::timeoutFrame(frameNumber);
    LOGH("X frameNumber: %d", frameNumber)
    return rc;
}

/*===========================================================================
 * FUNCTION   : getNextPendingCbBuffer
 *
 * DESCRIPTION: Returns the callback_buffer from the first entry of
 *              mOfflinePpInfoList
 *
 * PARAMETERS : none
 *
 * RETURN     : callback_buffer
 *==========================================================================*/
bool QCamera3YUVChannel::getNextPendingCbBuffer() {
    bool returned = false;
    buffer_handle_t *output;
    mm_camera_super_buf_t *callback_buffer = NULL;
    uint32_t frameNum;
    {
        Mutex::Autolock lock(mOfflinePpLock);
        if (mOfflinePpInfoList.size()) {
            if(mOfflinePpInfoList.begin()->offlinePpFlag == true &&
                    mOfflinePpInfoList.begin()->isReturnBuffer == true) {
                returned = true;
                output = mOfflinePpInfoList.begin()->output;
                frameNum = mOfflinePpInfoList.begin()->frameNumber;
                mOfflinePpInfoList.erase(mOfflinePpInfoList.begin());
                LOGD(" Sending the reprocess buffer :%d",frameNum);
            } else if (mOfflinePpInfoList.begin()->callback_buffer) {
                returned = true;
                callback_buffer = mOfflinePpInfoList.begin()->callback_buffer;
                mOfflinePpInfoList.erase(mOfflinePpInfoList.begin());
            }
        }
    }
    if(returned == true) {
        if(callback_buffer != NULL) {
            QCamera3ProcessingChannel::streamCbRoutine(
                   callback_buffer, mStreams[0]);
        } else {
            issueChannelCb(output, frameNum);
        }
    }
    return returned;
}

/*===========================================================================
 * FUNCTION   : reprocessCbRoutine
 *
 * DESCRIPTION: callback function for the reprocessed frame. This frame now
 *              should be returned to the framework. This same callback is
 *              used during input reprocessing or offline postprocessing
 *
 * PARAMETERS :
 * @resultBuffer      : buffer containing the reprocessed data
 * @resultFrameNumber : frame number on which the buffer was requested
 *
 * RETURN     : NONE
 *
 *==========================================================================*/
void QCamera3YUVChannel::reprocessCbRoutine(buffer_handle_t *resultBuffer,
        uint32_t resultFrameNumber)
{
    LOGD("E: frame number %d", resultFrameNumber);
    Vector<mm_camera_super_buf_t *> pendingCbs;

    /* release the input buffer and input metadata buffer if used */
    if (0 > mMemory.getHeapBufferIndex(resultFrameNumber)) {
        /* mOfflineMemory and mOfflineMetaMemory used only for input reprocessing */
        int32_t rc = releaseOfflineMemory(resultFrameNumber);
        if (NO_ERROR != rc) {
            LOGE("Error releasing offline memory rc = %d", rc);
        }
        /* Since reprocessing is done, send the callback to release the input buffer */
        if (mChannelCB) {
            mChannelCB(NULL, NULL, resultFrameNumber, true, mUserData);
        }
    }

    if (mBypass) {
        int32_t rc = handleOfflinePpCallback(resultFrameNumber, pendingCbs);
        if (rc != NO_ERROR) {
            LOGI("X: Error!!! rc = %d", rc);
            return;
        }
    }
    Mutex::Autolock lock(mYuvCbBufferLock);
    issueChannelCb(resultBuffer, resultFrameNumber);
    // Call all pending callbacks to return buffers
    while(getNextPendingCbBuffer());
}

/*===========================================================================
 * FUNCTION   : needsFramePostprocessing
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 *
 * RETURN     :
 *  TRUE if frame needs to be postprocessed
 *  FALSE is frame does not need to be postprocessed
 *
 *==========================================================================*/
bool QCamera3YUVChannel::needsFramePostprocessing(metadata_buffer_t *meta)
{
    bool ppNeeded = false;

    //sharpness
    IF_META_AVAILABLE(cam_edge_application_t, edgeMode,
            CAM_INTF_META_EDGE_MODE, meta) {
        mEdgeMode = *edgeMode;
    }

    //wnr
    IF_META_AVAILABLE(uint32_t, noiseRedMode,
            CAM_INTF_META_NOISE_REDUCTION_MODE, meta) {
        mNoiseRedMode = *noiseRedMode;
    }

    //crop region
    IF_META_AVAILABLE(cam_crop_region_t, scalerCropRegion,
            CAM_INTF_META_SCALER_CROP_REGION, meta) {
        mCropRegion = *scalerCropRegion;
    }

    if ((CAM_EDGE_MODE_OFF != mEdgeMode.edge_mode) &&
            (CAM_EDGE_MODE_ZERO_SHUTTER_LAG != mEdgeMode.edge_mode)) {
        ppNeeded = true;
    }
    if ((CAM_NOISE_REDUCTION_MODE_ZERO_SHUTTER_LAG != mNoiseRedMode) &&
            (CAM_NOISE_REDUCTION_MODE_OFF != mNoiseRedMode) &&
            (CAM_NOISE_REDUCTION_MODE_MINIMAL != mNoiseRedMode)) {
        ppNeeded = true;
    }
    if ((mCropRegion.width < (int32_t)mCamera3Stream->width) ||
            (mCropRegion.height < (int32_t)mCamera3Stream->height)) {
        ppNeeded = true;
    }

    return ppNeeded;
}

/*===========================================================================
 * FUNCTION   : handleOfflinePpCallback
 *
 * DESCRIPTION: callback function for the reprocessed frame from offline
 *              postprocessing.
 *
 * PARAMETERS :
 * @resultFrameNumber : frame number on which the buffer was requested
 * @pendingCbs        : pending buffers to be returned first
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3YUVChannel::handleOfflinePpCallback(uint32_t resultFrameNumber,
            Vector<mm_camera_super_buf_t *>& pendingCbs)
{
    Mutex::Autolock lock(mOfflinePpLock);
    List<PpInfo>::iterator ppInfo;
    if(pendingCbs.size() == 0)
        LOGD(" No Pending Call Back ");
    for (ppInfo = mOfflinePpInfoList.begin();
            ppInfo != mOfflinePpInfoList.end(); ppInfo++) {
        if (ppInfo->frameNumber == resultFrameNumber) {
            break;
        }
    }

    if (ppInfo == mOfflinePpInfoList.end()) {
        LOGI("Request of frame number %d is reprocessing",
                resultFrameNumber);
        return NO_ERROR;
    } else if (ppInfo->offlinePpFlag) {
        int32_t bufferIndex =
                mMemory.getHeapBufferIndex(resultFrameNumber);
        if (bufferIndex < 0) {
            LOGE("Fatal %d: no buffer index for frame number %d",
                    bufferIndex, resultFrameNumber);
            return BAD_VALUE;
        }
        LOGD(" Returned Reprocessed Buffer :%d ",resultFrameNumber );
        mMemory.markFrameNumber(bufferIndex, -1);
        mFreeHeapBufferList.push_back(bufferIndex);
    }
    else {
        LOGE("Fatal: request of frame number %d doesn't need"
                " offline postprocessing. However there is"
                " reprocessing callback.",
                resultFrameNumber);
        return BAD_VALUE;
    }

    if (ppInfo != mOfflinePpInfoList.begin()) {
        LOGD("callback for frame number %d should be head of list",
                 resultFrameNumber);
        ppInfo->isReturnBuffer = true;
        return BAD_VALUE;
    } else {
        LOGD(" Erasing Reprocessed Buffer :%d ",resultFrameNumber );
        //Move heap buffer into free pool and invalidate the frame number
        ppInfo = mOfflinePpInfoList.erase(ppInfo);
    }
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : getReprocessType
 *
 * DESCRIPTION: get the type of reprocess output supported by this channel
 *
 * PARAMETERS : NONE
 *
 * RETURN     : reprocess_type_t : type of reprocess
 *==========================================================================*/
reprocess_type_t QCamera3YUVChannel::getReprocessType()
{
    return REPROCESS_TYPE_YUV;
}

/*===========================================================================
 * FUNCTION   : getMpoOutputBuffer
 *
 * DESCRIPTION: get the output mpo buffer of BOKEH snapshot request (received
 *              in PCR).
 *
 * PARAMETERS : (output)pointer to mm_jpeg_output_t structure to get the
 *              buffer details.
 *
 * RETURN     : BAD_VALUE in case of error else NO_ERROR (int32_t).
 *==========================================================================*/
int32_t QCamera3PicChannel::getMpoOutputBuffer(mm_jpeg_output_t *output)
{
    mm_jpeg_output_t buf;
    int32_t bufIdx = getMpoBufferIndex();

    memset(&buf, 0, sizeof(mm_jpeg_output_t));

    if(bufIdx < 0)
    {
        LOGE("Error: could not find output buffer index");
        return BAD_VALUE;
    }

    buf.buf_filled_len = mMemory.getSize(bufIdx);
    buf.buf_vaddr = (uint8_t *)mMemory.getPtr(bufIdx);
    buf.fd = mMemory.getFd(bufIdx);

    *output = buf;

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : releaseSnapshotBuffer
 *
 * DESCRIPTION: Release snapshot buffer for next snapshot request. Adding
 *               snapshot index to mFreeBufferList so that it can be reused
 *               in next request.
 *
 * PARAMETERS :
 *  @src_frame : src_frame need to released.
 *
 * RETURN     : none.
 *==========================================================================*/
void QCamera3PicChannel::releaseSnapshotBuffer (mm_camera_super_buf_t*
                                                                      src_frame)
{
    int32_t snapshotIdx = -1;
    if(src_frame == NULL)
    {
        LOGD("Calling release of NULL frame");
        return;
    }

    if (src_frame) {
        if (mStreams[0]->getMyHandle() ==
                                src_frame->bufs[0]->stream_id) {
            snapshotIdx = (int32_t)src_frame->bufs[0]->buf_idx;
        } else {
            LOGD("Snapshot stream id %d and framework"
                    "source frame streamid %d don't match!",
                     mStreams[0]->getMyHandle(),
                     src_frame->bufs[0]->stream_id);
        }
    }

    if (0 <= snapshotIdx) {
        Mutex::Autolock lock(mFreeBuffersLock);
        mFreeBufferList.push_back((uint32_t)snapshotIdx);
    }
}

/*===========================================================================
 * FUNCTION   : mpoEvtHandle
 *
 * DESCRIPTION: Function registerd to postProcessor to handle MPO events.
 *
 * PARAMETERS :
 *  @status    : status of jpeg job.
 *  @p_output  : ptr to jpeg output result struct.
 *  @userdata  : user data ptr.
 *
 * RETURN     : none.
 *==========================================================================*/
void QCamera3PicChannel::mpoEvtHandle(jpeg_job_status_t status,
                                              mm_jpeg_output_t *p_output,
                                              void *userdata)
{
    LOGH("E");
    buffer_handle_t *resultBuffer = NULL;
    buffer_handle_t *jpegBufferHandle = NULL;
    QCamera3PicChannel *obj = (QCamera3PicChannel *)userdata;
    camera3_jpeg_blob_t jpegHeader;
    camera3_stream_buffer_t result;
    int resultStatus = CAMERA3_BUFFER_STATUS_OK;

    if(status == JPEG_JOB_STATUS_ERROR)
    {
        resultStatus = CAMERA3_BUFFER_STATUS_ERROR;
    }

    if(obj)
    {
        //create jpeg header
        uint32_t bufIdx = obj->getMpoBufferIndex();
        int32_t resultFrameNumber = obj->mMemory.getFrameNumber(bufIdx);
        jpegHeader.jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
        if(status == JPEG_JOB_STATUS_DONE)
        {
            char* jpeg_buf = (char *)p_output->buf_vaddr;
            jpegHeader.jpeg_size = p_output->buf_filled_len;
            {
                //dumping mpo output
                cam_frame_len_offset_t offset;
                memset(&offset, 0, sizeof(cam_frame_len_offset_t));
                mm_camera_buf_def_t *jpeg_dump_buffer = NULL;
                cam_dimension_t dim;
                dim.width = obj->mCamera3Stream->width;
                dim.height = obj->mCamera3Stream->height;
                jpeg_dump_buffer = (mm_camera_buf_def_t *)malloc(sizeof(mm_camera_buf_def_t));
                if(!jpeg_dump_buffer) {
                    LOGE("Could not allocate jpeg dump buffer");
                } else {
                    jpeg_dump_buffer->buffer = jpeg_buf;
                    jpeg_dump_buffer->frame_len = p_output->buf_filled_len;
                    jpeg_dump_buffer->frame_idx = obj->mMemory.getFrameNumber(bufIdx);
                    obj->dumpYUV(jpeg_dump_buffer, dim, offset, QCAMERA_DUMP_FRM_OUTPUT_JPEG);
                    free(jpeg_dump_buffer);
                }
            }

            ssize_t maxJpegSize = -1;

            // Gralloc buffer may have additional padding for 4K page size
            // Follow size guidelines based on spec since framework relies
            // on that to reach end of buffer and with it the header
            //Handle same as resultBuffer, but for readablity
            jpegBufferHandle =
                    (buffer_handle_t *)obj->mMemory.getBufferHandle(bufIdx);
            if (NULL != jpegBufferHandle) {
                maxJpegSize = ((private_handle_t*)(*jpegBufferHandle))->width;
                if (maxJpegSize <= 0 || maxJpegSize > obj->mMemory.getSize(bufIdx)) {
                    maxJpegSize = obj->mMemory.getSize(bufIdx);
                }

                size_t jpeg_eof_offset =
                        (size_t)(maxJpegSize - (ssize_t)sizeof(jpegHeader));
                char *jpeg_eof = &jpeg_buf[jpeg_eof_offset];
                memcpy(jpeg_eof, &jpegHeader, sizeof(jpegHeader));
                obj->mMemory.cleanInvalidateCache(bufIdx);
            } else {
                LOGE("JPEG buffer not found and index: %d",
                        bufIdx);
                resultStatus = CAMERA3_BUFFER_STATUS_ERROR;
            }
        }

        ////Use below data to issue framework callback
        resultBuffer =
                (buffer_handle_t *)obj->mMemory.getBufferHandle(bufIdx);
        int rc = obj->mMemory.unregisterBuffer(bufIdx);
        if (NO_ERROR != rc) {
            LOGE("Error %d unregistering stream buffer %d",
                     rc, bufIdx);
        }

        result.stream = obj->mCamera3Stream;
        result.buffer = resultBuffer;
        result.status = resultStatus;
        result.acquire_fence = -1;
        result.release_fence = -1;
        if (obj->mChannelCB) {
            LOGI("Issue Jpeg Callback frameNumber = %d status = %d",
                                   resultFrameNumber, resultStatus);
            obj->mChannelCB(NULL,
                        &result,
                        (uint32_t)resultFrameNumber,
                        false,
                        obj->mUserData);
        }
        obj->clearMpoBufferIndex();
     } else {
        LOGE("ERROR: NULL MPO USERDATA");
    }
    LOGH("X");
}

/* QCamera3PicChannel methods */

/*===========================================================================
 * FUNCTION   : jpegEvtHandle
 *
 * DESCRIPTION: Function registerd to mm-jpeg-interface to handle jpeg events.
                Construct result payload and call mChannelCb to deliver buffer
                to framework.
 *
 * PARAMETERS :
 *   @status    : status of jpeg job
 *   @client_hdl: jpeg client handle
 *   @jobId     : jpeg job Id
 *   @p_ouput   : ptr to jpeg output result struct
 *   @userdata  : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3PicChannel::jpegEvtHandle(jpeg_job_status_t status,
                                              uint32_t /*client_hdl*/,
                                              uint32_t jobId,
                                              mm_jpeg_output_t *p_output,
                                              void *userdata)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PIC_CH_JPEG_EVT_HANDLE);
    buffer_handle_t *resultBuffer = NULL;
    buffer_handle_t *jpegBufferHandle = NULL;
    int resultStatus = CAMERA3_BUFFER_STATUS_OK;
    bool isHDR = false;
    camera3_stream_buffer_t result;
    camera3_jpeg_blob_t jpegHeader;

    KPI_ATRACE_CAMSCOPE_INT("SNAPSHOT", CAMSCOPE_HAL3_SNAPSHOT, 0);
    QCamera3PicChannel *obj = (QCamera3PicChannel *)userdata;
    if (obj) {
        //Construct payload for process_capture_result. Call mChannelCb

        qcamera_hal3_jpeg_data_t *job = obj->m_postprocessor.findJpegJobByJobId(jobId);

        if ((job == NULL) || (status == JPEG_JOB_STATUS_ERROR)) {
            LOGE("Error in jobId: (%d) with status: %d", jobId, status);
            resultStatus = CAMERA3_BUFFER_STATUS_ERROR;
        }

        if (NULL != job) {
            uint32_t bufIdx = (uint32_t)job->jpeg_settings->out_buf_index;
            LOGD("jpeg out_buf_index: %d and if HDR :%d", bufIdx, job->jpeg_settings->hdr_snapshot);
            isHDR = job->jpeg_settings->hdr_snapshot;
            //Construct jpeg transient header of type camera3_jpeg_blob_t
            //Append at the end of jpeg image of buf_filled_len size

            jpegHeader.jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
            if (JPEG_JOB_STATUS_DONE == status) {
                jpegHeader.jpeg_size = (uint32_t)p_output->buf_filled_len;
                char* jpeg_buf = (char *)p_output->buf_vaddr;
                cam_frame_len_offset_t offset;
                memset(&offset, 0, sizeof(cam_frame_len_offset_t));
                mm_camera_buf_def_t *jpeg_dump_buffer = NULL;
                cam_dimension_t dim;
                dim.width = obj->mCamera3Stream->width;
                dim.height = obj->mCamera3Stream->height;
                jpeg_dump_buffer = (mm_camera_buf_def_t *)malloc(sizeof(mm_camera_buf_def_t));
                if(!jpeg_dump_buffer) {
                    LOGE("Could not allocate jpeg dump buffer");
                } else {
                    jpeg_dump_buffer->buffer = p_output->buf_vaddr;
                    jpeg_dump_buffer->frame_len = p_output->buf_filled_len;
                    jpeg_dump_buffer->frame_idx = obj->mMemory.getFrameNumber(bufIdx);
                    obj->dumpYUV(jpeg_dump_buffer, dim, offset, QCAMERA_DUMP_FRM_OUTPUT_JPEG);
                    free(jpeg_dump_buffer);
                }

                ssize_t maxJpegSize = -1;

                // Gralloc buffer may have additional padding for 4K page size
                // Follow size guidelines based on spec since framework relies
                // on that to reach end of buffer and with it the header

                //Handle same as resultBuffer, but for readablity
                jpegBufferHandle =
                        (buffer_handle_t *)obj->mMemory.getBufferHandle(bufIdx);

                if (NULL != jpegBufferHandle) {
                    maxJpegSize = ((private_handle_t*)(*jpegBufferHandle))->width;
                    if (maxJpegSize <= 0 || maxJpegSize > obj->mMemory.getSize(bufIdx)) {
                        maxJpegSize = obj->mMemory.getSize(bufIdx);
                    }

                    size_t jpeg_eof_offset =
                            (size_t)(maxJpegSize - (ssize_t)sizeof(jpegHeader));
                    char *jpeg_eof = &jpeg_buf[jpeg_eof_offset];
                    memcpy(jpeg_eof, &jpegHeader, sizeof(jpegHeader));
                    obj->mMemory.cleanInvalidateCache(bufIdx);
                } else {
                    LOGE("JPEG buffer not found and index: %d",
                            bufIdx);
                    resultStatus = CAMERA3_BUFFER_STATUS_ERROR;
                }
            }

            ////Use below data to issue framework callback
            resultBuffer =
                    (buffer_handle_t *)obj->mMemory.getBufferHandle(bufIdx);
            int32_t resultFrameNumber = obj->mMemory.getFrameNumber(bufIdx);
            int32_t rc = obj->mMemory.unregisterBuffer(bufIdx);
            if (NO_ERROR != rc) {
                LOGE("Error %d unregistering stream buffer %d",
                     rc, bufIdx);
            }

            result.stream = obj->mCamera3Stream;
            result.buffer = resultBuffer;
            result.status = resultStatus;
            result.acquire_fence = -1;
            result.release_fence = -1;

            // Release any snapshot buffers before calling
            // the user callback. The callback can potentially
            // unblock pending requests to snapshot stream.
            int32_t snapshotIdx = -1;
            mm_camera_super_buf_t* src_frame = NULL;

            if (job->src_reproc_frame)
                src_frame = job->src_reproc_frame;
            else
                src_frame = job->src_frame;

            if (src_frame) {
                if (obj->mStreams[0]->getMyHandle() ==
                        src_frame->bufs[0]->stream_id) {
                    snapshotIdx = (int32_t)src_frame->bufs[0]->buf_idx;
                } else {
                    LOGD("Snapshot stream id %d and framework"
                             "source frame streamid %d don't match!",
                             obj->mStreams[0]->getMyHandle(),
                            src_frame->bufs[0]->stream_id);
                }
            }
            if (0 <= snapshotIdx) {
                Mutex::Autolock lock(obj->mFreeBuffersLock);
                obj->mFreeBufferList.push_back((uint32_t)snapshotIdx);
            }

            if(isHDR) {
               obj->mPostProcStarted = false;
               obj->m_postprocessor.mChannelStop = false;
            } else {
                LOGI("Issue Jpeg Callback frameNumber = %d status = %d and %d",
                        resultFrameNumber, resultStatus, isHDR);
                if (obj->mChannelCB) {
                    obj->mChannelCB(NULL,
                            &result,
                            (uint32_t)resultFrameNumber,
                            false,
                            obj->mUserData);
                }
            }

            // release internal data for jpeg job
            if ((NULL != job->fwk_frame) || (NULL != job->fwk_src_buffer)) {
                /* unregister offline input buffer */
                int32_t inputBufIndex =
                        obj->mOfflineMemory.getGrallocBufferIndex((uint32_t)resultFrameNumber);
                if (0 <= inputBufIndex) {
                    rc = obj->mOfflineMemory.unregisterBuffer(inputBufIndex);
                } else {
                    LOGE("could not find the input buf index, frame number %d",
                             resultFrameNumber);
                }
                if (NO_ERROR != rc) {
                    LOGE("Error %d unregistering input buffer %d",
                             rc, bufIdx);
                }

                /* unregister offline meta buffer */
                int32_t metaBufIndex =
                        obj->mOfflineMetaMemory.getHeapBufferIndex((uint32_t)resultFrameNumber);
                if (0 <= metaBufIndex) {
                    Mutex::Autolock lock(obj->mFreeOfflineMetaBuffersLock);
                    obj->mFreeOfflineMetaBuffersList.push_back((uint32_t)metaBufIndex);
                } else {
                    LOGE("could not find the input meta buf index, frame number %d",
                             resultFrameNumber);
                }
            }else {
                obj->m_postprocessor.releaseOfflineBuffers(false);
            }
            obj->m_postprocessor.releaseJpegJobData(job);
            free(job);
            if(isHDR) {
               LOGD("Calling PostProc Stopped sending ");
               if (obj->mChannelCB) {
                   obj->mChannelCB(NULL,
                           &result,
                           (uint32_t)resultFrameNumber,
                           false,
                           obj->mUserData);
               }
               obj->m_postprocessor.stop(true);
            }

            QCamera3HardwareInterface *hw = (QCamera3HardwareInterface *)obj->mUserData;
            if (hw->isQuadCfaSensor()) {
                hw->deleteQCFARawChannel();
            }

        }

        return;
        // }
    } else {
        LOGE("Null userdata in jpeg callback");
    }
}

QCamera3PicChannel::QCamera3PicChannel(uint32_t cam_handle,
                    uint32_t channel_handle,
                    mm_camera_ops_t *cam_ops,
                    channel_cb_routine cb_routine,
                    channel_cb_buffer_err cb_buf_err,
                    cam_padding_info_t *paddingInfo,
                    void *userData,
                    camera3_stream_t *stream,
                    cam_feature_mask_t postprocess_mask,
                    __unused bool is4KVideo,
                    bool isInputStreamConfigured,
                    QCamera3Channel *metadataChannel,
                    uint32_t numBuffers) :
                        QCamera3ProcessingChannel(cam_handle, channel_handle,
                                cam_ops, cb_routine, cb_buf_err, paddingInfo, userData,
                                stream, CAM_STREAM_TYPE_SNAPSHOT,
                                postprocess_mask, metadataChannel, numBuffers),
                        mNumSnapshotBufs(0),
                        mInputBufferHint(isInputStreamConfigured),
                        mYuvMemory(NULL),
                        mFrameLen(0),
                        mAuxPicChannel(NULL),
                        mNeedPPUpscale(false)
{
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    m_max_pic_dim = hal_obj->calcMaxJpegDim();
    mYuvWidth = stream->width;
    mYuvHeight = stream->height;
    mStreamType = CAM_STREAM_TYPE_SNAPSHOT;
    // Use same pixelformat for 4K video case
    mStreamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_SNAPSHOT,
            stream->width, stream->height);
    int32_t rc = m_postprocessor.initJpeg(jpegEvtHandle, mpoEvtHandle, &m_max_pic_dim, this);
    if (rc != 0) {
        LOGE("Init Postprocessor failed");
    }

    if (is_dual_camera_by_handle(cam_handle)) {
            m_camHandle = get_main_camera_handle(cam_handle);
            m_handle = get_main_camera_handle(channel_handle);
            mAuxPicChannel = new QCamera3PicChannel(get_aux_camera_handle(cam_handle),
                        get_aux_camera_handle(channel_handle), cam_ops,
                        cb_routine, cb_buf_err,
                        paddingInfo, userData,
                        stream, postprocess_mask,
                        is4KVideo, isInputStreamConfigured,
                        metadataChannel, numBuffers);
            setDualChannelMode(true);
    }
}

/*===========================================================================
 * FUNCTION   : flush
 *
 * DESCRIPTION: flush pic channel, which will stop all processing within, including
 *              the reprocessing channel in postprocessor and YUV stream.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PicChannel::flush()
{
    int32_t rc = NO_ERROR;
    if(!m_bIsActive) {
        LOGE("Attempt to flush inactive channel");
        return NO_INIT;
    }

    rc = m_postprocessor.flush();
    if (rc == 0) {
        LOGE("Postprocessor flush failed, rc = %d", rc);
        return rc;
    }

    if (0 < mOfflineMetaMemory.getCnt()) {
        mOfflineMetaMemory.deallocate();
    }
    if (0 < mOfflineMemory.getCnt()) {
        mOfflineMemory.unregisterBuffers();
    }
    Mutex::Autolock lock(mFreeBuffersLock);
    mFreeBufferList.clear();

    for (uint32_t i = 0; i < mCamera3Stream->max_buffers; i++) {
        mFreeBufferList.push_back(i);
    }

    {
        Mutex::Autolock lock(mFreeJpegBufferLock);
        mFreeJpegBufferList.clear();
    }

    if (mAuxPicChannel) {
        rc = mAuxPicChannel->flush();
    }
    return rc;
}


QCamera3PicChannel::~QCamera3PicChannel()
{
    if(0 < mJpegMemory.getCnt())
    {
        mJpegMemory.deallocate();
        Mutex::Autolock lock(mFreeJpegBufferLock);
        mFreeJpegBufferList.clear();
    }
    if (mAuxPicChannel) {
        delete mAuxPicChannel;
        mAuxPicChannel = NULL;
    }
}

int32_t QCamera3PicChannel::start()
{
    int32_t rc = NO_ERROR;
    QCamera3Channel::start();
    if (mAuxPicChannel) {
        rc = mAuxPicChannel->start();
    }
    return rc;
}

int32_t QCamera3PicChannel::stop()
{
    int32_t rc = NO_ERROR;
    QCamera3ProcessingChannel::stop();
    if (mAuxPicChannel) {
        rc = mAuxPicChannel->stop();
    }
    return rc;
}

void QCamera3PicChannel::setDualChannelMode(bool bMode)
{
    QCamera3Channel::setDualChannelMode(bMode);
    if(NULL != mAuxPicChannel)
    {
        mAuxPicChannel->setDualChannelMode(bMode);
    }
}

int32_t QCamera3PicChannel::initialize(cam_is_type_t isType)
{
    int32_t rc = NO_ERROR;
    cam_dimension_t streamDim;
    cam_stream_type_t streamType;
    cam_format_t streamFormat;

    if (NULL == mCamera3Stream) {
        LOGE("Camera stream uninitialized");
        return NO_INIT;
    }

    if (1 <= m_numStreams) {
        // Only one stream per channel supported in v3 Hal
        return NO_ERROR;
    }

    mIsType = isType;
    streamType = mStreamType;
    streamFormat = mStreamFormat;
    streamDim.width = (int32_t)mYuvWidth;
    streamDim.height = (int32_t)mYuvHeight;

    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if(m_bDualChannel)
    {
        //if 2 channels are configure with different dimensions
        //for low configuration don't change the stream dimension.
        hal_obj->rectifyStreamDimIfNeeded(
                streamDim, (mAuxPicChannel != NULL) ? CAM_TYPE_MAIN: CAM_TYPE_AUX, mNeedPPUpscale);
        mYuvWidth = streamDim.width;
        mYuvHeight = streamDim.height;
    }

    mNumSnapshotBufs = mCamera3Stream->max_buffers;
    rc = QCamera3Channel::addStream(streamType, streamFormat, streamDim,
            ROTATE_0, (uint8_t)mCamera3Stream->max_buffers, mPostProcMask,
            mIsType);

    if (NO_ERROR != rc) {
        LOGE("Initialize failed, rc = %d", rc);
        return rc;
    }

    /* initialize offline meta memory for input reprocess */
    rc = QCamera3ProcessingChannel::initialize(isType);
    if (NO_ERROR != rc) {
        LOGE("Processing Channel initialize failed, rc = %d",
                 rc);
    }

    if (mAuxPicChannel) {
        mAuxPicChannel->initialize(isType);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : request
 *
 * DESCRIPTION: handle the request - either with an input buffer or a direct
 *              output request
 *
 * PARAMETERS :
 * @buffer       : pointer to the output buffer
 * @frameNumber  : frame number of the request
 * @pInputBuffer : pointer to input buffer if an input request
 * @metadata     : parameters associated with the request
 * @internalreq      : boolean to indicate if this is purely internal request
 *                    needing internal buffer allocation
 * @meteringonly    : boolean indicating metering only frame subset of internal
 *                    not consumed by postprocessor
 *
 * RETURN     : 0 on a success start of capture
 *              -EINVAL on invalid input
 *              -ENODEV on serious error
 *==========================================================================*/
int32_t QCamera3PicChannel::request(buffer_handle_t *buffer,
        uint32_t frameNumber,
        camera3_stream_buffer_t *pInputBuffer,
        metadata_buffer_t *metadata, int &indexUsed,
        bool internalRequest, bool meteringOnly)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PIC_CH_REQ);
    //FIX ME: Return buffer back in case of failures below.

    int32_t rc = NO_ERROR;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;

    reprocess_config_t reproc_cfg;
    cam_dimension_t dim;
    memset(&reproc_cfg, 0, sizeof(reprocess_config_t));
    //make sure to set the correct input stream dim in case of YUV size override
    //and recalculate the plane info
    dim.width = (int32_t)mYuvWidth;
    dim.height = (int32_t)mYuvHeight;

    setReprocConfig(reproc_cfg, pInputBuffer, metadata, mStreamFormat, dim, mNeedPPUpscale);

    if (hal_obj->m_bQuadraCfaRequest) {
        LOGI("override reprocess input config for quadra cfa");
        reproc_cfg.input_stream_dim = hal_obj->getQuadraCfaDim();
        reproc_cfg.stream_format = hal_obj->mRdiModeFmt;
        reproc_cfg.stream_type = CAM_STREAM_TYPE_RAW;
        mm_stream_calc_offset_raw(reproc_cfg.stream_format, &reproc_cfg.input_stream_dim,
                reproc_cfg.padding, &reproc_cfg.input_stream_plane_info);
    }

    // Picture stream has already been started before any request comes in
    if (!m_bIsActive) {
        LOGE("Channel not started!!");
        return NO_INIT;
    }

    // Start postprocessor
    LOGD(" Starting Postproc Channel");
    startPostProc(reproc_cfg);

    //B+M DualCam mode: Main channel will always be Bayre.
    //                 mAuxPicChannel will always be Mono.
    //W+T DualCam mode: Main channel will always be Wide.
    //                 mAuxPicChannel will always be Tele.
    //bIsMaster will be true if current pic channel is mMasterCam.
    bool bIsMaster = true;
    if(m_bDualChannel)
    {
        bIsMaster = mAuxPicChannel ? (mMasterCam == CAM_TYPE_MAIN) : (mMasterCam == CAM_TYPE_AUX);
    }

    //if needHALPP() is false then take snapshot from mMaster channel,
    // else request on both channels.
    if(hal_obj->needHALPP() || bIsMaster)
    {
        if (!internalRequest) {
            if(hal_obj->needHALPP() && (hal_obj->getHalPPType() == CAM_HAL_PP_TYPE_BOKEH))
            {   //BOKEH MODE
                //Master channel: allocate 2 jpeg settings one for main image
                //and other for bokeh with respective output buffer index.
                //Non Master will allocate buffer for depth image.
                int index;
                int numOfJpegSettings = 1;
                if(bIsMaster)
                {
                    numOfJpegSettings = 2;
                }
                for (int i =0; i < numOfJpegSettings; i++)
                {
                    Mutex::Autolock lock(mFreeJpegBufferLock);
                    if (mFreeJpegBufferList.empty()) {
                        if(bIsMaster)
                        {
                            index = mJpegMemory.allocateOne(mCamera3Stream->width *
                                                            mCamera3Stream->height);
                        } else {
                            cam_dimension_t dim = {0,0};
#ifdef ENABLE_QC_BOKEH
                            qrcp::getDepthMapSize(mCamera3Stream->width,
                                                    mCamera3Stream->height,
                                                    dim.width, dim.height);
#endif //ENABLE_QC_BOKEH
                            index = mJpegMemory.allocateOne(dim.width * dim.height);
                        }
                        if(index < 0)
                        {
                            LOGE("Could not allocate HEAP Memory");
                            return index;
                        }
                    } else{
                       auto it = mFreeJpegBufferList.begin();
                       index = *it;
                       mFreeJpegBufferList.erase(it);
                    }
                    LOGD("buffer index %d, frameNumber: %u %p", index, frameNumber, &mJpegMemory);

                    rc = mJpegMemory.markFrameNumber((uint32_t)index, frameNumber);

                    // Queue jpeg settings
                    if (bIsMaster) {
                        rc = queueJpegSetting((uint32_t)index, metadata, (i==0)?
                                        CAM_HAL3_JPEG_TYPE_MAIN : CAM_HAL3_JPEG_TYPE_BOKEH);
                    } else {
                        rc = queueJpegSetting((uint32_t)index, metadata, CAM_HAL3_JPEG_TYPE_DEPTH);
                    }
                }
                if(bIsMaster)
                {
                    //Get the index of framework allocated buffer, will be required while composeMpo
                    int index = mMemory.getMatchBufIndex((void*)buffer);
                    if(index < 0) {
                        rc = registerBuffer(buffer, mIsType);
                        if (NO_ERROR != rc) {
                            LOGE("On-the-fly buffer registration failed %d", rc);
                            return rc;
                        }

                        index = mMemory.getMatchBufIndex((void*)buffer);
                        if (index < 0) {
                            LOGE("Could not find object among registered buffers");
                            return DEAD_OBJECT;
                        }
                    }

                    mMpoOutBufIndex = (uint32_t) index;

                    rc = mMemory.markFrameNumber((uint32_t)index, frameNumber);
                }
            }
            else {
                // For FUSION usecase i.e. needHALPP is true in non-BOKEH mode:
                // jpeg_setting for slave is only required for overideMetadata
                // so setting invalid index.
                int index = -1;
                if(bIsMaster)
                {
                    index = mMemory.getMatchBufIndex((void*)buffer);

                    if(index < 0) {
                        rc = registerBuffer(buffer, mIsType);
                        if (NO_ERROR != rc) {
                            LOGE("On-the-fly buffer registration failed %d",
                                     rc);
                            return rc;
                        }

                        index = mMemory.getMatchBufIndex((void*)buffer);
                        if (index < 0) {
                            LOGE("Could not find object among registered buffers");
                            return DEAD_OBJECT;
                        }
                    }
                    LOGD("buffer index %d, frameNumber: %u", index, frameNumber);

                    rc = mMemory.markFrameNumber((uint32_t)index, frameNumber);
                }
                // Queue jpeg settings
                if(hal_obj->getHalPPType() == CAM_HAL_PP_TYPE_DUAL_FOV)
                {
                     rc = queueJpegSetting((uint32_t)index, metadata, bIsMaster ?
                                            CAM_HAL3_JPEG_TYPE_FUSION : CAM_HAL3_JPEG_TYPE_AUX);
                } else {
                    rc = queueJpegSetting((uint32_t)index, metadata, bIsMaster ?
                                            CAM_HAL3_JPEG_TYPE_MAIN : CAM_HAL3_JPEG_TYPE_AUX);
                }
            }
        } else {
            LOGD("Internal request @ Picchannel");
        }

        if (pInputBuffer == NULL && hal_obj->m_bQuadraCfaRequest) {
            LOGI("trigger reprocess for quadra cfa");
            QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
            QCamera3QCfaRawChannel *pChannel = hal_obj->mQCFARawChannel;
            if (pChannel == NULL || !pChannel->meta_received || !pChannel->raw_received) {
                LOGE("request failed!");
                return -1;
            }

            mm_camera_super_buf_t *super_buf =
                    (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
            if (super_buf == NULL) {
                LOGE("fail to allocate memory");
                return -1;
            }
            memcpy(super_buf, &(pChannel->meta_frame), sizeof(mm_camera_super_buf_t));
            m_postprocessor.processPPMetadata(super_buf, frameNumber, false);

            super_buf = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
            if (super_buf == NULL) {
                LOGE("fail to allocate memory");
                return -1;
            }
            memcpy(super_buf, &(pChannel->raw_frame), sizeof(mm_camera_super_buf_t));
            m_postprocessor.processData(super_buf, NULL, frameNumber);
        } else if (pInputBuffer == NULL) {
            Mutex::Autolock lock(mFreeBuffersLock);
            uint32_t bufIdx;
            if (mFreeBufferList.empty()) {
                rc = mYuvMemory->allocateOne(mFrameLen);
                if (rc < 0) {
                    LOGE("Failed to allocate heap buffer. Fatal");
                    return rc;
                } else {
                    bufIdx = (uint32_t)rc;
                }
            } else {
                List<uint32_t>::iterator it = mFreeBufferList.begin();
                bufIdx = *it;
                mFreeBufferList.erase(it);
            }
            if (meteringOnly) {
                mYuvMemory->markFrameNumber(bufIdx, 0xFFFFFFFF);
            } else {
                mYuvMemory->markFrameNumber(bufIdx, frameNumber);
            }
            mStreams[0]->bufDone(bufIdx);
            indexUsed = bufIdx;
        } else {
            qcamera_fwk_input_pp_data_t *src_frame = NULL;
            src_frame = (qcamera_fwk_input_pp_data_t *)calloc(1,
                    sizeof(qcamera_fwk_input_pp_data_t));
            if (src_frame == NULL) {
                LOGE("No memory for src frame");
                return NO_MEMORY;
            }
            rc = setFwkInputPPData(src_frame, pInputBuffer, &reproc_cfg, metadata,
                    NULL /*fwk output buffer*/, frameNumber);
            if (NO_ERROR != rc) {
                LOGE("Error %d while setting framework input PP data", rc);
                free(src_frame);
                return rc;
            }
            LOGH("Post-process started");
            m_postprocessor.processData(src_frame);
        }
    }

    if (mAuxPicChannel && (!bIsMaster || hal_obj->needHALPP()) ) {
        int AuxIndexUsed = 0;
        rc = mAuxPicChannel->request(buffer, frameNumber,
                pInputBuffer, metadata, AuxIndexUsed,
                internalRequest, meteringOnly);
        //Only mMasterCam indexUsed is required, slave is in freeRunIdx
        if((rc != -1) && (!bIsMaster))
        {
            indexUsed = AuxIndexUsed;
        }
    }

    return rc;
}



/*===========================================================================
 * FUNCTION   : dataNotifyCB
 *
 * DESCRIPTION: Channel Level callback used for super buffer data notify.
 *              This function is registered with mm-camera-interface to handle
 *              data notify
 *
 * PARAMETERS :
 *   @recvd_frame   : stream frame received
 *   userdata       : user data ptr
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3PicChannel::dataNotifyCB(mm_camera_super_buf_t *recvd_frame,
                                 void *userdata)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_PIC_CH_DATA_NOTIFY_CB);
    LOGD("E\n");
    QCamera3PicChannel *channel = (QCamera3PicChannel *)userdata;

    if (channel == NULL) {
        LOGE("invalid channel pointer");
        return;
    }

    if(channel->m_numStreams != 1) {
        LOGE("Error: Bug: This callback assumes one stream per channel");
        return;
    }


    if(channel->mStreams[0] == NULL) {
        LOGE("Error: Invalid Stream object");
        return;
    }

    channel->QCamera3PicChannel::streamCbRoutine(recvd_frame, channel->mStreams[0]);

    LOGD("X\n");
    return;
}

int32_t QCamera3PicChannel::releaseOfflineMemory(uint32_t resultFrameNumber)
{
    return QCamera3ProcessingChannel::releaseOfflineMemory(resultFrameNumber);
}

void QCamera3PicChannel::freeBufferForFrame(mm_camera_super_buf_t *frame)
{
    Mutex::Autolock lock(mFreeBuffersLock);
    LOGD(" Freeing the Buffer");
    mFreeBufferList.push_back(frame->bufs[0]->buf_idx);
}

void QCamera3PicChannel::freeBufferForJpeg(int& index)
{
    Mutex::Autolock lock(mFreeJpegBufferLock);
    mFreeJpegBufferList.push_back(index);
}


/*===========================================================================
 * FUNCTION   : streamCbRoutine
 *
 * DESCRIPTION:
 *
 * PARAMETERS :
 * @super_frame : the super frame with filled buffer
 * @stream      : stream on which the buffer was requested and filled
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3PicChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                            QCamera3Stream *stream)
{
    KPI_ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_CAPTURE_CH_CB);
    //TODO
    //Used only for getting YUV. Jpeg callback will be sent back from channel
    //directly to HWI. Refer to func jpegEvtHandle

    //Got the yuv callback. Calling yuv callback handler in PostProc
    uint8_t frameIndex;
    mm_camera_super_buf_t* frame = NULL;
    cam_dimension_t dim;
    cam_frame_len_offset_t offset;

    memset(&dim, 0, sizeof(dim));
    memset(&offset, 0, sizeof(cam_frame_len_offset_t));

    if (checkStreamCbErrors(super_frame, stream) != NO_ERROR) {
        LOGE("Error with the stream callback");
        return;
    }

    frameIndex = (uint8_t)super_frame->bufs[0]->buf_idx;
    LOGD("recvd buf_idx: %u for further processing",
         (uint32_t)frameIndex);
    if(frameIndex >= mNumSnapshotBufs) {
         LOGE("Error, Invalid index for buffer");
         if(stream) {
             Mutex::Autolock lock(mFreeBuffersLock);
             mFreeBufferList.push_back(frameIndex);
             stream->bufDone(frameIndex);
         }
         return;
    }

    if ((uint32_t)mYuvMemory->getFrameNumber(frameIndex) == EMPTY_FRAMEWORK_FRAME_NUMBER) {
        LOGD("Internal Request recycle frame");
        Mutex::Autolock lock(mFreeBuffersLock);
        mFreeBufferList.push_back(frameIndex);
        free(super_frame);
        return;
    }

    frame = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
    if (frame == NULL) {
       LOGE("Error allocating memory to save received_frame structure.");
       if(stream) {
           Mutex::Autolock lock(mFreeBuffersLock);
           mFreeBufferList.push_back(frameIndex);
           stream->bufDone(frameIndex);
       }
       return;
    }
    *frame = *super_frame;
    stream->getFrameDimension(dim);
    stream->getFrameOffset(offset);
    dumpYUV(frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_INPUT_REPROCESS);

    if (IS_BUFFER_ERROR(super_frame->bufs[0]->flags)) {
        mChannelCbBufErr(this, mYuvMemory->getFrameNumber(frameIndex),
                CAMERA3_BUFFER_STATUS_ERROR, mUserData);
    }
    LOGD(" Sending frame :%d ",mYuvMemory->getFrameNumber(frameIndex));
    m_postprocessor.processData(frame, NULL, (uint32_t)mYuvMemory->getFrameNumber(frameIndex));
    free(super_frame);
    return;
}

QCamera3StreamMem* QCamera3PicChannel::getStreamBufs(uint32_t len)
{
    mYuvMemory = new QCamera3StreamMem(mCamera3Stream->max_buffers, false);
    if (!mYuvMemory) {
        LOGE("unable to create metadata memory");
        return NULL;
    }
    mFrameLen = len;

    return mYuvMemory;
}

void QCamera3PicChannel::putStreamBufs()
{
    QCamera3ProcessingChannel::putStreamBufs();

    mYuvMemory->deallocate();
    delete mYuvMemory;
    mYuvMemory = NULL;
    mFreeBufferList.clear();
    // Clear offlineMemory
    mOfflineMemory.clear();
}

int32_t QCamera3PicChannel::queueJpegSetting(uint32_t index, metadata_buffer_t *metadata,
                                                                 cam_hal3_JPEG_type_t imagetype)
{
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    jpeg_settings_t *settings =
            (jpeg_settings_t *)malloc(sizeof(jpeg_settings_t));

    if (!settings) {
        LOGE("out of memory allocating jpeg_settings");
        return -ENOMEM;
    }

    memset(settings, 0, sizeof(jpeg_settings_t));

    settings->out_buf_index = index;
    settings->image_type = imagetype;

    settings->jpeg_orientation = 0;
    IF_META_AVAILABLE(int32_t, orientation, CAM_INTF_META_JPEG_ORIENTATION, metadata) {
        settings->jpeg_orientation = *orientation;
    }

    settings->jpeg_quality = 85;
    IF_META_AVAILABLE(uint32_t, quality1, CAM_INTF_META_JPEG_QUALITY, metadata) {
        settings->jpeg_quality = (uint8_t) *quality1;
    }

    if((hal_obj->getHalPPType() == CAM_HAL_PP_TYPE_BOKEH && hal_obj->needHALPP())?
                                    (imagetype == CAM_HAL3_JPEG_TYPE_BOKEH) : true)
    {
        IF_META_AVAILABLE(uint32_t, quality2, CAM_INTF_META_JPEG_THUMB_QUALITY, metadata) {
            settings->jpeg_thumb_quality = (uint8_t) *quality2;
        }

        IF_META_AVAILABLE(cam_dimension_t, dimension, CAM_INTF_META_JPEG_THUMB_SIZE, metadata) {
            settings->thumbnail_size = *dimension;
        }
    }

    if(hal_obj->getHalPPType() == CAM_HAL_PP_TYPE_BOKEH && hal_obj->needHALPP())
    {
        settings->encode_type = MM_JPEG_TYPE_MPO;
    } else {
       settings->encode_type = MM_JPEG_TYPE_JPEG;
    }

    settings->is_dim_valid = false;
    if((hal_obj->getHalPPType() == CAM_HAL_PP_TYPE_BOKEH) &&
                        (imagetype != CAM_HAL3_JPEG_TYPE_DEPTH))
    {
        settings->output_dim.width = mCamera3Stream->width;
        settings->output_dim.height = mCamera3Stream->height;
        settings->is_dim_valid = true;
    }

    settings->gps_timestamp_valid = 0;
    IF_META_AVAILABLE(int64_t, timestamp, CAM_INTF_META_JPEG_GPS_TIMESTAMP, metadata) {
        settings->gps_timestamp = *timestamp;
        settings->gps_timestamp_valid = 1;
    }

    settings->gps_coordinates_valid = 0;
    IF_META_AVAILABLE(double, coordinates, CAM_INTF_META_JPEG_GPS_COORDINATES, metadata) {
        memcpy(settings->gps_coordinates, coordinates, 3*sizeof(double));
        settings->gps_coordinates_valid = 1;
    }

    IF_META_AVAILABLE(uint8_t, proc_methods, CAM_INTF_META_JPEG_GPS_PROC_METHODS, metadata) {
        memset(settings->gps_processing_method, 0,
                sizeof(settings->gps_processing_method));
        strlcpy(settings->gps_processing_method, (const char *)proc_methods,
                sizeof(settings->gps_processing_method));
    }

    settings->hdr_snapshot = 0;
    IF_META_AVAILABLE(cam_hdr_param_t, hdr_info, CAM_INTF_PARM_HAL_BRACKETING_HDR, metadata) {
        if (hdr_info->hdr_enable) {
            settings->hdr_snapshot = 1;
        }
    }


    // Image description
    const char *eepromVersion = hal_obj->getEepromVersionInfo();
    const uint32_t *ldafCalib = hal_obj->getLdafCalib();
    if ((eepromVersion && strlen(eepromVersion)) ||
            ldafCalib) {
        int len = 0;
        settings->image_desc_valid = true;
        if (eepromVersion && strlen(eepromVersion)) {
            len = snprintf(settings->image_desc, sizeof(settings->image_desc),
                    "M:%s ", eepromVersion);
        }
        if (ldafCalib) {
            snprintf(settings->image_desc + len,
                    sizeof(settings->image_desc) - len, "L:%u-%u",
                    ldafCalib[0], ldafCalib[1]);
        }
    }

    return m_postprocessor.processJpegSettingData(settings);
}


void QCamera3PicChannel::overrideYuvSize(uint32_t width, uint32_t height)
{
   LOGH("override Yuv size to: %d x %d", width, height);
   mYuvWidth = width;
   mYuvHeight = height;
   if(mAuxPicChannel)
       mAuxPicChannel->overrideYuvSize(width,height);
}

/*===========================================================================
 * FUNCTION   : getReprocessType
 *
 * DESCRIPTION: get the type of reprocess output supported by this channel
 *
 * PARAMETERS : NONE
 *
 * RETURN     : reprocess_type_t : type of reprocess
 *==========================================================================*/
reprocess_type_t QCamera3PicChannel::getReprocessType()
{
    /* a picture channel could either use the postprocessor for reprocess+jpeg
       or only for reprocess */
    reprocess_type_t expectedReprocess;
    if (mPostProcMask == CAM_QCOM_FEATURE_NONE || mInputBufferHint) {
        expectedReprocess = REPROCESS_TYPE_JPEG;
    } else {
        expectedReprocess = REPROCESS_TYPE_NONE;
    }
    LOGH("expectedReprocess from Pic Channel is %d", expectedReprocess);
    return expectedReprocess;
}


/*===========================================================================
 * FUNCTION   : timeoutFrame
 *
 * DESCRIPTION: Method to indicate to channel that a given frame has take too
 *              long to be generated
 *
 * PARAMETERS : framenumber indicating the framenumber of the buffer timingout
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PicChannel::timeoutFrame(uint32_t frameNumber)
{
    int32_t bufIdx;

    bufIdx = mYuvMemory->getBufferIndex(frameNumber);

    if (bufIdx < 0) {
        LOGE("X: Buffer not found for frame:%d", frameNumber);
        return -1;
    }

    mStreams[0]->timeoutFrame(bufIdx);

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : queueReprocMetadata
 *
 * DESCRIPTION: queue the reprocess metadata to the postprocessor
 *
 * PARAMETERS : metadata : the metadata corresponding to the pp frame
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PicChannel::queueReprocMetadata(mm_camera_super_buf_t *metadata,
                            uint32_t framenum, bool dropFrame)
{
    if(mAuxPicChannel && metadata->camera_handle != m_camHandle)
    {
        return mAuxPicChannel->queueReprocMetadata(metadata, framenum, dropFrame);
    } else {
        return QCamera3ProcessingChannel::queueReprocMetadata(metadata, framenum, dropFrame);
    }
    LOGE("Should never be here");
    return -1;
}


/*===========================================================================
 * FUNCTION   : switchMaster
 *
 * DESCRIPTION: Set the current master in pic channel.
 *
 * PARAMETERS : masterCam : MAIN or AUX cam type
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
void QCamera3PicChannel::switchMaster(uint32_t masterCam)
{
    mMasterCam = masterCam;
    if(mAuxPicChannel)
        mAuxPicChannel->switchMaster(masterCam);
}

void QCamera3PicChannel::overridePPConfig(cam_feature_mask_t pp_mask)
{
    LOGD("overriding ppmask to %" PRIx64, pp_mask);
    mPostProcMask = pp_mask;
    if(NULL != mAuxPicChannel)
    {
        mAuxPicChannel->overridePPConfig(pp_mask);
    }
}

/*===========================================================================
 * FUNCTION   : setBundleInfo
 *
 * DESCRIPTION: setting bundle information in stream params
 *
 * PARAMETERS :
 *   @bundleInfo  : stream bundle information.
 *   @cam_type    : MAIN or AUX.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3PicChannel::setBundleInfo(const cam_bundle_config_t &bundleInfo, uint32_t cam_type)
{
    if ((cam_type == CAM_TYPE_AUX) && (NULL != mAuxPicChannel)) {
        return mAuxPicChannel->setBundleInfo(bundleInfo);
    }
    int32_t rc = NO_ERROR;
    cam_stream_parm_buffer_t param;
    memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
    param.type = CAM_STREAM_PARAM_TYPE_SET_BUNDLE_INFO;
    param.bundleInfo = bundleInfo;
    if (m_numStreams > 0 && mStreams[0]) {
        rc = mStreams[0]->setParameter(param, cam_type);
        if (rc != NO_ERROR) {
            LOGE("stream setParameter for set bundle failed");
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : QCamera3ReprocessChannel
 *
 * DESCRIPTION: constructor of QCamera3ReprocessChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @pp_mask    : post-proccess feature mask
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3ReprocessChannel::QCamera3ReprocessChannel(uint32_t cam_handle,
                                                 uint32_t channel_handle,
                                                 mm_camera_ops_t *cam_ops,
                                                 channel_cb_routine cb_routine,
                                                 channel_cb_buffer_err cb_buf_err,
                                                 cam_padding_info_t *paddingInfo,
                                                 cam_feature_mask_t postprocess_mask,
                                                 void *userData, void *ch_hdl) :
    /* In case of framework reprocessing, pproc and jpeg operations could be
     * parallelized by allowing 1 extra buffer for reprocessing output:
     * ch_hdl->getNumBuffers() + 1 */
    QCamera3Channel(cam_handle, channel_handle, cam_ops, cb_routine, cb_buf_err, paddingInfo,
                    postprocess_mask, userData,
                    ((QCamera3Channel *)ch_hdl)->getNumBuffers()
                              + (MAX_REPROCESS_PIPELINE_STAGES - 1)),
    inputChHandle(ch_hdl),
    mOfflineBuffersIndex(-1),
    mFrameLen(0),
    mFrameLenMeta(0),
    mReprocessType(REPROCESS_TYPE_NONE),
    m_pSrcChannel(NULL),
    m_pMetaChannel(NULL),
    mMemory(NULL),
    mMemoryMeta(NULL),
    mGrallocMemory(0),
    mReprocessPerfMode(false),
    m_bOfflineIsp(false),
    m_ppIndex(0)
{
    memset(mSrcStreamHandles, 0, sizeof(mSrcStreamHandles));
    mOfflineBuffersIndex = mNumBuffers -1;
    mOfflineMetaIndex = (int32_t) (2*mNumBuffers -1);
    memset(&m_processedMetaBuf, 0, sizeof(mm_camera_buf_def_t));
}


/*===========================================================================
 * FUNCTION   : QCamera3ReprocessChannel
 *
 * DESCRIPTION: constructor of QCamera3ReprocessChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @pp_mask    : post-proccess feature mask
 *
 * RETURN     : none
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::initialize(cam_is_type_t isType)
{
    int32_t rc = NO_ERROR;
    mm_camera_channel_attr_t attr;

    memset(&attr, 0, sizeof(mm_camera_channel_attr_t));
    attr.notify_mode = MM_CAMERA_SUPER_BUF_NOTIFY_CONTINUOUS;
    attr.max_unmatched_frames = 1;

    m_handle = m_camOps->add_channel(m_camHandle,
                                      &attr,
                                      NULL,
                                      this);
    if (m_handle == 0) {
        LOGE("Add channel failed");
        return UNKNOWN_ERROR;
    }

    mIsType = isType;
    return rc;
}

/*===========================================================================
 * FUNCTION   : registerBuffer
 *
 * DESCRIPTION: register streaming buffer to the channel object
 *
 * PARAMETERS :
 *   @buffer     : buffer to be registered
 *   @isType     : the image stabilization type for the buffer
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::registerBuffer(buffer_handle_t *buffer,
        cam_is_type_t isType)
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_REPROC_CH_REG_BUF);
    int rc = 0;
    mIsType = isType;
    cam_stream_type_t streamType;

    if (buffer == NULL) {
        LOGE("Error: Cannot register a NULL buffer");
        return BAD_VALUE;
    }

    if ((uint32_t)mGrallocMemory.getCnt() > (mNumBuffers - 1)) {
        LOGE("Trying to register more buffers than initially requested");
        return BAD_VALUE;
    }

    if (0 == m_numStreams) {
        rc = initialize(mIsType);
        if (rc != NO_ERROR) {
            LOGE("Couldn't initialize camera stream %d",
                     rc);
            return rc;
        }
    }

    streamType = mStreams[0]->getMyType();
    rc = mGrallocMemory.registerBuffer(buffer, streamType);
    if (ALREADY_EXISTS == rc) {
        return NO_ERROR;
    } else if (NO_ERROR != rc) {
        LOGE("Buffer %p couldn't be registered %d", buffer, rc);
        return rc;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : QCamera3ReprocessChannel
 *
 * DESCRIPTION: constructor of QCamera3ReprocessChannel
 *
 * PARAMETERS :
 *   @cam_handle : camera handle
 *   @cam_ops    : ptr to camera ops table
 *   @pp_mask    : post-proccess feature mask
 *
 * RETURN     : none
 *==========================================================================*/
void QCamera3ReprocessChannel::streamCbRoutine(mm_camera_super_buf_t *super_frame,
                                  QCamera3Stream *stream)
{
    //Got the pproc data callback. Now send to jpeg encoding
    uint8_t frameIndex;
    uint32_t resultFrameNumber;
    mm_camera_super_buf_t* frame = NULL;
    LOGD("E. current reproc ch idx:%d", m_ppIndex);

    // input channel is the pic/yuv channel (processing channel) for all reprocess channels
    QCamera3ProcessingChannel *obj = (QCamera3ProcessingChannel *)inputChHandle;
    cam_dimension_t dim;
    cam_frame_len_offset_t offset;

    // TODO: here assume meta always comes befofe raw reprocess stream
    if (m_bOfflineIsp && isMetaReprocStream(stream)) {
        m_processedMetaBuf = *(super_frame->bufs[0]);
        // as we cached the meta reproc output,
        // we can deallocate original meta reproc stream buffers here
        // righ now, deallocate buffer in QCamera3ReprocessChannel::bufDone()/putStreamBufs()
        return;
    }

    memset(&dim, 0, sizeof(dim));
    memset(&offset, 0, sizeof(cam_frame_len_offset_t));
    if(!super_frame) {
         LOGE("Invalid Super buffer");
         return;
    }

    if(super_frame->num_bufs != 1) {
         LOGE("Multiple streams are not supported");
         return;
    }
    if(super_frame->bufs[0] == NULL ) {
         LOGE("Error, Super buffer frame does not contain valid buffer");
         return;
    }
    frameIndex = (uint8_t)super_frame->bufs[0]->buf_idx;


    if (mReprocessType == REPROCESS_TYPE_JPEG) {
        resultFrameNumber =  mMemory->getFrameNumber(frameIndex);
        frame = (mm_camera_super_buf_t *)malloc(sizeof(mm_camera_super_buf_t));
        if (frame == NULL) {
           LOGE("Error allocating memory to save received_frame structure.");
           if(stream) {
               stream->bufDone(frameIndex);
           }
           return;
        }
        LOGI("bufIndex: %u recvd from post proc",
                 (uint32_t)frameIndex);
        *frame = *super_frame;

        stream->getFrameDimension(dim);
        stream->getFrameOffset(offset);
        dumpYUV(frame->bufs[0], dim, offset, QCAMERA_DUMP_FRM_INPUT_JPEG);
        bool isInputBuf = obj->isFwkInputBuffer((uint32_t)resultFrameNumber);
        if (isInputBuf) {
            obj->m_postprocessor.releaseOfflineBuffers(false);
            obj->releaseInputBuffer(resultFrameNumber);
        }
        if (mChannelCB && m_ppIndex == 0) {
            mChannelCB(NULL, NULL, resultFrameNumber, true, mUserData);
        }

        obj->m_postprocessor.processPPData(frame,
                m_bOfflineIsp ? ((const metadata_buffer_t*)m_processedMetaBuf.buffer) : NULL);
    } else {
        buffer_handle_t *resultBuffer;
        frameIndex = (uint8_t)super_frame->bufs[0]->buf_idx;
        resultBuffer = (buffer_handle_t *)mGrallocMemory.getBufferHandle(frameIndex);
        resultFrameNumber = mGrallocMemory.getFrameNumber(frameIndex);
        LOGD(" Reprocessed Frame Number is : %d",resultFrameNumber);
        int32_t rc = stream->bufRelease(frameIndex);
        if (NO_ERROR != rc) {
            LOGE("Error %d releasing stream buffer %d",
                     rc, frameIndex);
        }
        rc = mGrallocMemory.unregisterBuffer(frameIndex);
        if (NO_ERROR != rc) {
            LOGE("Error %d unregistering stream buffer %d",
                     rc, frameIndex);
        }

        obj->m_postprocessor.releaseOfflineBuffers(false);
        obj->reprocessCbRoutine(resultBuffer, resultFrameNumber);
        qcamera_hal3_pp_data_t *pp_job = obj->m_postprocessor.dequeuePPJob(resultFrameNumber);
        if (pp_job != NULL) {
            obj->m_postprocessor.releasePPJobData(pp_job);
        }
        free(pp_job);
        resetToCamPerfNormal(resultFrameNumber);
    }
    free(super_frame);
    return;
}


/*===========================================================================
 * FUNCTION   : resetToCamPerfNormal
 *
 * DESCRIPTION: Set the perf mode to normal if all the priority frames
 *              have been reprocessed
 *
 * PARAMETERS :
 *      @frameNumber: Frame number of the reprocess completed frame
 *
 * RETURN     : QCamera3StreamMem *
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::resetToCamPerfNormal(uint32_t frameNumber)
{
    int32_t rc = NO_ERROR;
    bool resetToPerfNormal = false;
    {
        Mutex::Autolock lock(mPriorityFramesLock);
        /* remove the priority frame number from the list */
        for (size_t i = 0; i < mPriorityFrames.size(); i++) {
            if (mPriorityFrames[i] == frameNumber) {
                mPriorityFrames.removeAt(i);
            }
        }
        /* reset the perf mode if pending priority frame list is empty */
        if (mReprocessPerfMode && mPriorityFrames.empty()) {
            resetToPerfNormal = true;
        }
    }
    if (resetToPerfNormal) {
        QCamera3Stream *pStream = mStreams[0];
        cam_stream_parm_buffer_t param;
        memset(&param, 0, sizeof(cam_stream_parm_buffer_t));

        param.type = CAM_STREAM_PARAM_TYPE_REQUEST_OPS_MODE;
        param.perf_mode = CAM_PERF_NORMAL;
        rc = pStream->setParameter(param);
        {
            Mutex::Autolock lock(mPriorityFramesLock);
            mReprocessPerfMode = false;
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : getOfflineMetaStreamBufs
 *
 * DESCRIPTION: register the buffers of the meta reprocess stream
 *
 * PARAMETERS : none
 *
 * RETURN     : QCamera3StreamMem *
 *==========================================================================*/
QCamera3StreamMem* QCamera3ReprocessChannel::getMetaStreamBufs(uint32_t len)
{
    LOGD("E. len:%d", len);
    mMemoryMeta = new QCamera3StreamMem(mNumBuffers, false);
    mFrameLenMeta = len;
    if (!mMemoryMeta) {
        LOGE("unable to create meta reproc memory");
        return NULL;
    }

    return mMemoryMeta;
}

/*===========================================================================
 * FUNCTION   : getStreamBufs
 *
 * DESCRIPTION: register the buffers of the reprocess channel
 *
 * PARAMETERS : none
 *
 * RETURN     : QCamera3StreamMem *
 *==========================================================================*/
QCamera3StreamMem* QCamera3ReprocessChannel::getStreamBufs(uint32_t len)
{
    if (mReprocessType == REPROCESS_TYPE_JPEG) {
        mMemory = new QCamera3StreamMem(mNumBuffers, false);
        if (!mMemory) {
            LOGE("unable to create reproc memory");
            return NULL;
        }
        mFrameLen = len;
        return mMemory;
    }
    return &mGrallocMemory;
}

/*===========================================================================
 * FUNCTION   : putStreamBufs
 *
 * DESCRIPTION: release the reprocess channel buffers
 *
 * PARAMETERS : none
 *
 * RETURN     :
 *==========================================================================*/
void QCamera3ReprocessChannel::putStreamBufs()
{
   if (mReprocessType == REPROCESS_TYPE_JPEG) {
       mMemory->deallocate();
       delete mMemory;
       mMemory = NULL;
       mFreeBufferList.clear();
   } else {
       mGrallocMemory.unregisterBuffers();
   }
}

/*===========================================================================
 * FUNCTION   : putMetaStreamBufs
 *
 * DESCRIPTION: release the reprocess channel buffers
 *
 * PARAMETERS : none
 *
 * RETURN     :
 *==========================================================================*/
void QCamera3ReprocessChannel::putMetaStreamBufs()
{
    LOGD("put stream bufs for offline meta");
    if (mMemoryMeta != NULL) {
        mMemoryMeta->deallocate();
        delete mMemoryMeta;
        mMemoryMeta = NULL;
    }
    return;
}

/*===========================================================================
 * FUNCTION   : ~QCamera3ReprocessChannel
 *
 * DESCRIPTION: destructor of QCamera3ReprocessChannel
 *
 * PARAMETERS : none
 *
 * RETURN     : none
 *==========================================================================*/
QCamera3ReprocessChannel::~QCamera3ReprocessChannel()
{
    destroy();

    if (m_handle) {
        m_camOps->delete_channel(m_camHandle, m_handle);
        LOGD("deleting channel %d", m_handle);
        m_handle = 0;
    }
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: start reprocess channel.
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::start()
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_REPROC_CH_START);
    int32_t rc = NO_ERROR;

    rc = QCamera3Channel::start();

    if (rc == NO_ERROR) {
       rc = m_camOps->start_channel(m_camHandle, m_handle);

       // Check failure
       if (rc != NO_ERROR) {
           LOGE("start_channel failed %d", rc);
           QCamera3Channel::stop();
       }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop reprocess channel.
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::stop()
{
    ATRACE_CAMSCOPE_CALL(CAMSCOPE_HAL3_REPROC_CH_STOP);
    int32_t rc = NO_ERROR;
    rc = QCamera3Channel::stop();
    rc |= m_camOps->stop_channel(m_camHandle, m_handle);
    // Unmapping the buffers
    unmapOfflineBuffers(true);
    return rc;
}

/*===========================================================================
 * FUNCTION   : getStreamBySrcHandle
 *
 * DESCRIPTION: find reprocess stream by its source stream handle
 *
 * PARAMETERS :
 *   @srcHandle : source stream handle
 *
 * RETURN     : ptr to reprocess stream if found. NULL if not found
 *==========================================================================*/
QCamera3Stream * QCamera3ReprocessChannel::getStreamBySrcHandle(uint32_t srcHandle)
{
    QCamera3Stream *pStream = NULL;

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mSrcStreamHandles[i] == srcHandle) {
            pStream = mStreams[i];
            break;
        }
    }
    return pStream;
}

/*===========================================================================
 * FUNCTION   : getSrcStreamBySrcHandle
 *
 * DESCRIPTION: find source stream by source stream handle
 *
 * PARAMETERS :
 *   @srcHandle : source stream handle
 *
 * RETURN     : ptr to reprocess stream if found. NULL if not found
 *==========================================================================*/
QCamera3Stream * QCamera3ReprocessChannel::getSrcStreamBySrcHandle(uint32_t srcHandle)
{
    QCamera3Stream *pStream = NULL;

    if (NULL == m_pSrcChannel) {
        return NULL;
    }

    for (uint32_t i = 0; i < m_numStreams; i++) {
        if (mSrcStreamHandles[i] == srcHandle) {
            pStream = m_pSrcChannel->getStreamByIndex(i);
            break;
        }
    }
    return pStream;
}

/*===========================================================================
 * FUNCTION   : unmapOfflineBuffers
 *
 * DESCRIPTION: Unmaps offline buffers
 *
 * PARAMETERS : none
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::unmapOfflineBuffers(bool all)
{
    LOGD("E. all = %d, m_numStreams:%d", all, m_numStreams);
    int unmap_cnt = m_numStreams;
    int rc = NO_ERROR;
    {
        Mutex::Autolock lock(mOfflineBuffersLock);
        if (!mOfflineBuffers.empty()) {
            QCamera3Stream *stream = NULL;
            List<OfflineBuffer>::iterator it = mOfflineBuffers.begin();
           for (; it != mOfflineBuffers.end(); it++) {
               stream = (*it).stream;
               if (NULL != stream) {
                   rc = stream->unmapBuf((*it).type,
                                         (*it).index,
                                            -1);
                   if (NO_ERROR != rc) {
                       LOGE("Error during offline buffer unmap %d",
                              rc);
                   }
                   LOGD("Unmapped buffer with index %d", (*it).index);
               }
               if (!all) {
                   mOfflineBuffers.erase(it);
                   unmap_cnt--;
                   if (unmap_cnt == 0) {
                       break;
                   }
               }
            }
            if (all) {
               mOfflineBuffers.clear();
            }
        }
    }

    Mutex::Autolock lock(mOfflineMetaBuffersLock);
    if (!mOfflineMetaBuffers.empty()) {
        unmap_cnt = m_numStreams;
        QCamera3Stream *stream = NULL;
        List<OfflineBuffer>::iterator it = mOfflineMetaBuffers.begin();
        for (; it != mOfflineMetaBuffers.end(); it++) {
           stream = (*it).stream;
           if (NULL != stream) {
               rc = stream->unmapBuf((*it).type,
                                     (*it).index,
                                        -1);
               if (NO_ERROR != rc) {
                   LOGE("Error during offline buffer unmap %d",
                          rc);
               }
               LOGD("Unmapped meta buffer with index %d", (*it).index);
           }
           if (!all) {
               mOfflineMetaBuffers.erase(it);
               unmap_cnt--;
               if (unmap_cnt == 0) {
                   break;
               }
           }
        }
        if (all) {
           mOfflineMetaBuffers.clear();
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : bufDone
 *
 * DESCRIPTION: Return reprocess stream buffer to free buffer list.
 *              Note that this function doesn't queue buffer back to kernel.
 *              It's up to doReprocessOffline to do that instead.
 * PARAMETERS :
 *   @recvd_frame  : stream buf frame to be returned
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::bufDone(mm_camera_super_buf_t *recvd_frame)
{
    int rc = NO_ERROR;
    if (recvd_frame && recvd_frame->num_bufs == 1) {
        Mutex::Autolock lock(mFreeBuffersLock);
        uint32_t buf_idx = recvd_frame->bufs[0]->buf_idx;
        mFreeBufferList.push_back(buf_idx);

        if (m_bOfflineIsp && mMemoryMeta != NULL) {
            LOGD("deallocate memory for meta reproc stream");
            mMemoryMeta->deallocate();
        }
    } else {
        LOGE("Fatal. Not supposed to be here");
        rc = BAD_VALUE;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : overrideMetadata
 *
 * DESCRIPTION: Override metadata entry such as rotation, crop, and CDS info.
 *
 * PARAMETERS :
 *   @frame     : input frame from source stream
 *   meta_buffer: metadata buffer
 *   @metadata  : corresponding metadata
 *   @fwk_frame :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::overrideMetadata(qcamera_hal3_pp_buffer_t *pp_buffer,
        mm_camera_buf_def_t *meta_buffer, jpeg_settings_t *jpeg_settings,
        qcamera_fwk_input_pp_data_t &fwk_frame)
{
    int32_t rc = NO_ERROR;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if ((NULL == meta_buffer) || (NULL == pp_buffer) || (NULL == pp_buffer->input) ||
            (NULL == hal_obj)) {
        return BAD_VALUE;
    }

    metadata_buffer_t *meta = (metadata_buffer_t *)meta_buffer->buffer;
    mm_camera_super_buf_t *frame = pp_buffer->input;
    if (NULL == meta) {
        return BAD_VALUE;
    }

    for (uint32_t i = 0; i < frame->num_bufs; i++) {
        QCamera3Stream *pStream = getStreamBySrcHandle(frame->bufs[i]->stream_id);
        QCamera3Stream *pSrcStream = getSrcStreamBySrcHandle(frame->bufs[i]->stream_id);

        if (pStream != NULL && pSrcStream != NULL) {
            if (jpeg_settings) {
                // Find rotation info for reprocess stream
                cam_rotation_info_t rotation_info;
                memset(&rotation_info, 0, sizeof(rotation_info));
                if (jpeg_settings->jpeg_orientation == 0) {
                   rotation_info.rotation = ROTATE_0;
                } else if (jpeg_settings->jpeg_orientation == 90) {
                   rotation_info.rotation = ROTATE_90;
                } else if (jpeg_settings->jpeg_orientation == 180) {
                   rotation_info.rotation = ROTATE_180;
                } else if (jpeg_settings->jpeg_orientation == 270) {
                   rotation_info.rotation = ROTATE_270;
                }
                if (jpeg_settings->hdr_snapshot ||
                    hal_obj->getHalPPType() == CAM_HAL_PP_TYPE_BOKEH) {
                    rotation_info.rotation = ROTATE_0;
                }
                rotation_info.streamId = mStreams[0]->getMyServerID();
                ADD_SET_PARAM_ENTRY_TO_BATCH(meta, CAM_INTF_PARM_ROTATION, rotation_info);
                LOGD("rotation_info.rotation: %d", rotation_info.rotation);
            }

            // Find and insert crop info for reprocess stream
            IF_META_AVAILABLE(cam_crop_data_t, crop_data, CAM_INTF_META_CROP_DATA, meta) {
                if (MAX_NUM_STREAMS > crop_data->num_of_streams) {
                    for (int j = 0; j < crop_data->num_of_streams; j++) {
                        if (crop_data->crop_info[j].stream_id ==
                                pSrcStream->getMyServerID()) {

                            // Store crop/roi information for offline reprocess
                            // in the reprocess stream slot
                            crop_data->crop_info[crop_data->num_of_streams].crop =
                                    crop_data->crop_info[j].crop;
                            crop_data->crop_info[crop_data->num_of_streams].roi_map =
                                    crop_data->crop_info[j].roi_map;
                            crop_data->crop_info[crop_data->num_of_streams].stream_id =
                                    mStreams[0]->getMyServerID();
                            crop_data->num_of_streams++;

                            if((!hal_obj->needHALPP()) &&
                                   (hal_obj->getHalPPType() ==  CAM_HAL_PP_TYPE_BOKEH) &&
                                    (jpeg_settings != NULL))
                            {
                                //In bokeh mode, reprocess don't do cropping irresepctive of scene
                                //is bokeh eligible or not, so setting jpeg crop.
                                jpeg_settings->crop = crop_data->crop_info[j].crop;
                                jpeg_settings->crop.width = PAD_TO_SIZE(
                                                                jpeg_settings->crop.width,2);
                                jpeg_settings->crop.height = PAD_TO_SIZE(
                                                                jpeg_settings->crop.height,2);
                                jpeg_settings->is_crop_valid = true;
                            }

                            LOGD("Reprocess stream server id: %d",
                                     mStreams[0]->getMyServerID());
                            LOGD("Found offline reprocess crop %dx%d %dx%d",
                                    crop_data->crop_info[j].crop.left,
                                    crop_data->crop_info[j].crop.top,
                                    crop_data->crop_info[j].crop.width,
                                    crop_data->crop_info[j].crop.height);
                            LOGD("Found offline reprocess roimap %dx%d %dx%d",
                                    crop_data->crop_info[j].roi_map.left,
                                    crop_data->crop_info[j].roi_map.top,
                                    crop_data->crop_info[j].roi_map.width,
                                    crop_data->crop_info[j].roi_map.height);

                            break;
                        }
                    }
                } else {
                    LOGE("No space to add reprocess stream crop/roi information");
                }
            }

            IF_META_AVAILABLE(cam_cds_data_t, cdsInfo, CAM_INTF_META_CDS_DATA, meta) {
                uint8_t cnt = cdsInfo->num_of_streams;
                if (cnt <= MAX_NUM_STREAMS) {
                    cam_stream_cds_info_t repro_cds_info;
                    memset(&repro_cds_info, 0, sizeof(repro_cds_info));
                    repro_cds_info.stream_id = mStreams[0]->getMyServerID();
                    for (size_t i = 0; i < cnt; i++) {
                        if (cdsInfo->cds_info[i].stream_id ==
                                pSrcStream->getMyServerID()) {
                            repro_cds_info.cds_enable =
                                    cdsInfo->cds_info[i].cds_enable;
                            break;
                        }
                    }
                    cdsInfo->num_of_streams = 1;
                    cdsInfo->cds_info[0] = repro_cds_info;
                } else {
                    LOGE("No space to add reprocess stream cds information");
                }
            }

            fwk_frame.input_buffer = *frame->bufs[i];
            fwk_frame.metadata_buffer = *meta_buffer;
            fwk_frame.output_buffer = pp_buffer->output;
            break;
        } else {
            LOGE("Source/Re-process streams are invalid");
            rc |= BAD_VALUE;
        }
    }

    return rc;
}

int32_t QCamera3ReprocessChannel::overrideMetadata(metadata_buffer_t *meta_buffer,
        jpeg_settings_t *jpeg_settings, uint32_t input_stream_svr_id)
{
    int32_t rc = NO_ERROR;
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if ((NULL == meta_buffer) || (NULL == hal_obj) || input_stream_svr_id == 0) {
        return BAD_VALUE;
    }

    LOGD("E, input stream id:%d", input_stream_svr_id);

    if (jpeg_settings != NULL) {
        // Find rotation info for reprocess stream
        cam_rotation_info_t rotation_info;
        memset(&rotation_info, 0, sizeof(rotation_info));
        if (jpeg_settings->jpeg_orientation == 0) {
           rotation_info.rotation = ROTATE_0;
        } else if (jpeg_settings->jpeg_orientation == 90) {
           rotation_info.rotation = ROTATE_90;
        } else if (jpeg_settings->jpeg_orientation == 180) {
           rotation_info.rotation = ROTATE_180;
        } else if (jpeg_settings->jpeg_orientation == 270) {
           rotation_info.rotation = ROTATE_270;
        }
        if (jpeg_settings->hdr_snapshot) {
            rotation_info.rotation = ROTATE_0;
        }
        rotation_info.streamId = mStreams[0]->getMyServerID();
        LOGD("roration:%d, stream id:%d", rotation_info.rotation, rotation_info.streamId);
        ADD_SET_PARAM_ENTRY_TO_BATCH(meta_buffer, CAM_INTF_PARM_ROTATION, rotation_info);
    }

    IF_META_AVAILABLE(cam_cds_data_t, cdsInfo, CAM_INTF_META_CDS_DATA, meta_buffer) {
        uint8_t cnt = cdsInfo->num_of_streams;
        LOGD("num of streams in cdsInfo:%d", cnt);
        if (cnt <= MAX_NUM_STREAMS) {
            cam_stream_cds_info_t repro_cds_info;
            memset(&repro_cds_info, 0, sizeof(repro_cds_info));
            repro_cds_info.stream_id = mStreams[0]->getMyServerID();
            for (size_t i = 0; i < cnt; i++) {
                if (cdsInfo->cds_info[i].stream_id == input_stream_svr_id) {
                    repro_cds_info.cds_enable = cdsInfo->cds_info[i].cds_enable;
                    break;
                }
            }
            cdsInfo->num_of_streams = 1;
            cdsInfo->cds_info[0] = repro_cds_info;
        }
    }

    return rc;
}

/*===========================================================================
* FUNCTION : overrideFwkMetadata
*
* DESCRIPTION: Override frameworks metadata such as rotation, crop, and CDS data.
*
* PARAMETERS :
* @frame : input frame for reprocessing
*
* RETURN : int32_t type of status
* NO_ERROR -- success
* none-zero failure code
*==========================================================================*/
int32_t QCamera3ReprocessChannel::overrideFwkMetadata(
        qcamera_fwk_input_pp_data_t *frame)
{
    if (NULL == frame) {
        LOGE("Incorrect input frame");
        return BAD_VALUE;
    }

    if (NULL == frame->metadata_buffer.buffer) {
        LOGE("No metadata available");
        return BAD_VALUE;
    }
    metadata_buffer_t *meta = (metadata_buffer_t *) frame->metadata_buffer.buffer;

    // Not doing rotation at all for YUV to YUV reprocess
    if (mReprocessType != REPROCESS_TYPE_JPEG) {
        LOGD("Override rotation to 0 for channel reprocess type %d",
                mReprocessType);
        cam_rotation_info_t rotation_info;
        memset(&rotation_info, 0, sizeof(rotation_info));
        rotation_info.rotation = ROTATE_0;
        rotation_info.streamId = mStreams[0]->getMyServerID();
        ADD_SET_PARAM_ENTRY_TO_BATCH(meta, CAM_INTF_PARM_ROTATION, rotation_info);
    }

    // Find and insert crop info for reprocess stream
    IF_META_AVAILABLE(cam_crop_data_t, crop_data, CAM_INTF_META_CROP_DATA, meta) {
        if (1 == crop_data->num_of_streams) {
            // Store crop/roi information for offline reprocess
            // in the reprocess stream slot
            crop_data->crop_info[crop_data->num_of_streams].crop =
                    crop_data->crop_info[0].crop;
            crop_data->crop_info[crop_data->num_of_streams].roi_map =
                    crop_data->crop_info[0].roi_map;
            crop_data->crop_info[crop_data->num_of_streams].stream_id =
                    mStreams[0]->getMyServerID();
            crop_data->num_of_streams++;

            LOGD("Reprocess stream server id: %d",
                     mStreams[0]->getMyServerID());
            LOGD("Found offline reprocess crop %dx%d %dx%d",
                    crop_data->crop_info[0].crop.left,
                    crop_data->crop_info[0].crop.top,
                    crop_data->crop_info[0].crop.width,
                    crop_data->crop_info[0].crop.height);
            LOGD("Found offline reprocess roi map %dx%d %dx%d",
                    crop_data->crop_info[0].roi_map.left,
                    crop_data->crop_info[0].roi_map.top,
                    crop_data->crop_info[0].roi_map.width,
                    crop_data->crop_info[0].roi_map.height);
        } else {
            LOGE("Incorrect number of offline crop data entries %d",
                    crop_data->num_of_streams);
            return BAD_VALUE;
        }
    } else {
        LOGW("Crop data not present");
    }

    IF_META_AVAILABLE(cam_cds_data_t, cdsInfo, CAM_INTF_META_CDS_DATA, meta) {
        if (1 == cdsInfo->num_of_streams) {
            cdsInfo->cds_info[0].stream_id = mStreams[0]->getMyServerID();
        } else {
            LOGE("Incorrect number of offline cds info entries %d",
                     cdsInfo->num_of_streams);
            return BAD_VALUE;
        }
    }

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : doMetaReprocessOffline
 *
 * DESCRIPTION: request to do a reprocess on meta frame
 *
 * PARAMETERS :
 *   @frame     : input frame for reprocessing
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
 int32_t  QCamera3ReprocessChannel::doMetaReprocessOffline(
        qcamera_fwk_input_pp_data_t *frame)
{
    int32_t rc = 0;
    OfflineBuffer mappedBuffer;
    LOGD("E");

    if (!m_bOfflineIsp) {
        LOGE("meta reprocess is not allowed when m_bOfflineIsp is not set");
        return BAD_VALUE;
    }

    uint32_t i;
    for (i = 0; i < m_numStreams; i++) {
        if (isMetaReprocStream(mStreams[i])) {
            LOGD("found meta reproc stream idx:%d, src stream handle:%x", i, mSrcStreamHandles[i]);
            break;
        }
    }
    if (i >= m_numStreams) {
        LOGE("can't find meta reproc stream!");
        return BAD_VALUE;
    }
    QCamera3Stream* pMetaReprocStream = mStreams[i];

    if (mMemoryMeta == NULL) {
        LOGE("mMemoryMeta shouldn't be NULL.");
        return BAD_VALUE;
    }

    uint32_t bufIdx = 0;
    rc = mMemoryMeta->allocateOne(mFrameLenMeta);
    if (rc < 0) {
        LOGE("Failed allocating heap buffer. Fatal");
        return BAD_VALUE;
    } else {
        bufIdx = (uint32_t)rc;
    }

    LOGD("allocate one buffer, len:%d, idx:%d", mFrameLenMeta, bufIdx);
    mMemoryMeta->markFrameNumber(bufIdx, frame->frameNumber);
    rc = pMetaReprocStream->bufDone(bufIdx);

    // map offline buffers
    uint32_t buf_idx = mOfflineBuffersIndex;

    //Do cache ops before sending for reprocess
    mMemoryMeta->cleanInvalidateCache(buf_idx);

    rc = pMetaReprocStream->mapBuf(
            CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF,
            buf_idx, -1,
            frame->metadata_buffer.fd, frame->metadata_buffer.buffer,
            frame->metadata_buffer.frame_len);
    if (NO_ERROR == rc) {
        mappedBuffer.index = buf_idx;
        mappedBuffer.stream = pMetaReprocStream;
        mappedBuffer.type = CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF;
        mOfflineBuffers.push_back(mappedBuffer);
        LOGD("Mapped buffer with index %d", buf_idx);
    }

    uint32_t meta_buf_idx = mOfflineMetaIndex;
    rc |= pMetaReprocStream->mapBuf(
            CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF,
            meta_buf_idx, -1,
            frame->metadata_buffer.fd, frame->metadata_buffer.buffer,
            frame->metadata_buffer.frame_len);
    if (NO_ERROR == rc) {
        mappedBuffer.index = meta_buf_idx;
        mappedBuffer.stream = pMetaReprocStream;
        mappedBuffer.type = CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF;
        mOfflineMetaBuffers.push_back(mappedBuffer);
        LOGD("Mapped meta buffer with index %d", meta_buf_idx);
    }

    cam_stream_parm_buffer_t param;
    memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
    param.type = CAM_STREAM_PARAM_TYPE_DO_REPROCESS;
    param.reprocess.buf_index = buf_idx;
    param.reprocess.frame_idx = frame->input_buffer.frame_idx;
    param.reprocess.meta_present = 1;
    param.reprocess.meta_buf_index = meta_buf_idx;

    LOGI("Offline reprocessing id = %d buf Id = %d meta index = %d",
                param.reprocess.frame_idx, param.reprocess.buf_index,
                param.reprocess.meta_buf_index);
    rc = pMetaReprocStream->setParameter(param);

    LOGE("X, rc = %d", rc);
    return rc;
}


int32_t QCamera3ReprocessChannel::timeoutFrame(uint32_t frameNumber)
{
    int32_t index = -1, ret = NO_ERROR;
    LOGH("E, frameNumber: %d", frameNumber)

    if (mReprocessType ==  REPROCESS_TYPE_JPEG) {
        index = mMemory->getFrameNumber(frameNumber);
    } else {
        index = mGrallocMemory.getGrallocBufferIndex(frameNumber);
    }

    if (index < 0) {
        LOGH("Error!! X, frameNumber: %d index(%d) not found", frameNumber, index)
        return BAD_VALUE;
    }
    ret = mStreams[0]->cancelBuffer(index);
    LOGH("X, frameNumber: %d", frameNumber)
    return ret;
}

/*===========================================================================
 * FUNCTION   : doReprocessOffline
 *
 * DESCRIPTION: request to do a reprocess on the frame
 *
 * PARAMETERS :
 *   @frame     : input frame for reprocessing
 *   @isPriorityFrame: Hint that this frame is of priority, equivalent to
 *              real time, even though it is processed in offline mechanism
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
 int32_t  QCamera3ReprocessChannel::doReprocessOffline(
        qcamera_fwk_input_pp_data_t *frame, bool isPriorityFrame)
{
    int32_t rc = 0;
    int index;
    OfflineBuffer mappedBuffer;

    if (m_numStreams < 1) {
        LOGE("No reprocess stream is created");
        return -1;
    }

    if (NULL == frame) {
        LOGE("Incorrect input frame");
        return BAD_VALUE;
    }

    if (NULL == frame->metadata_buffer.buffer) {
        LOGE("No metadata available");
        return BAD_VALUE;
    }

    if (NULL == frame->input_buffer.buffer) {
        LOGE("No input buffer available");
        return BAD_VALUE;
    }

    if ((0 == m_numStreams) || (NULL == mStreams[0])) {
        LOGE("Reprocess stream not initialized!");
        return NO_INIT;
    }

    if (m_bOfflineIsp) {
        LOGH("do meta offline reprocess first");
        rc = doMetaReprocessOffline(frame);
        if (rc != NO_ERROR) {
            LOGE("meta stream setParameter for reprocess failed");
        }
    }

    QCamera3Stream *pStream = mStreams[0];

    //qbuf the output buffer if it was allocated by the framework
    if (mReprocessType != REPROCESS_TYPE_JPEG && frame->output_buffer != NULL) {
        if(!m_bIsActive) {
            rc = registerBuffer(frame->output_buffer, mIsType);
            if (NO_ERROR != rc) {
                LOGE("On-the-fly buffer registration failed %d",
                         rc);
                return rc;
            }

            rc = start();
            if (NO_ERROR != rc) {
                return rc;
            }
        }
        index = mGrallocMemory.getMatchBufIndex((void*)frame->output_buffer);
        if(index < 0) {
            rc = registerBuffer(frame->output_buffer, mIsType);
            if (NO_ERROR != rc) {
                LOGE("On-the-fly buffer registration failed %d",
                         rc);
                return rc;
            }

            index = mGrallocMemory.getMatchBufIndex((void*)frame->output_buffer);
            if (index < 0) {
                LOGE("Could not find object among registered buffers");
                return DEAD_OBJECT;
            }
        }
        rc = mGrallocMemory.markFrameNumber(index, frame->frameNumber);
        LOGD("Offline Reprocessing Starting for Buf Index : %d  and frame number :%d ",
                index, frame->frameNumber);
        if(rc != NO_ERROR) {
            LOGE("Failed to mark frame#:%d, index:%d",frame->frameNumber,index);
            return rc;
        }
        rc = pStream->bufDone(index);
        if(rc != NO_ERROR) {
            LOGE("Failed to Q new buffer to stream");
            mGrallocMemory.markFrameNumber(index, -1);
            return rc;
        }

    } else if (mReprocessType == REPROCESS_TYPE_JPEG) {
        Mutex::Autolock lock(mFreeBuffersLock);
        uint32_t bufIdx;
        if (mFreeBufferList.empty()) {
            rc = mMemory->allocateOne(mFrameLen);
            if (rc < 0) {
                LOGE("Failed allocating heap buffer. Fatal");
                return BAD_VALUE;
            } else {
                bufIdx = (uint32_t)rc;
            }
        } else {
            bufIdx = *(mFreeBufferList.begin());
            mFreeBufferList.erase(mFreeBufferList.begin());
        }

        mMemory->markFrameNumber(bufIdx, frame->frameNumber);
        rc = pStream->bufDone(bufIdx);
        if (rc != NO_ERROR) {
            LOGE("Failed to queue new buffer to stream");
            return rc;
        }
    }

    int32_t max_idx = (int32_t) (mNumBuffers - 1);
    //loop back the indices if max burst count reached
    if (mOfflineBuffersIndex == max_idx) {
       mOfflineBuffersIndex = -1;
    }
    uint32_t buf_idx = (uint32_t)(mOfflineBuffersIndex + 1);

    //Do cache ops before sending for reprocess
    if (mMemory != NULL) {
        mMemory->cleanInvalidateCache(buf_idx);
    }

    rc = pStream->mapBuf(
            CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF,
            buf_idx, -1,
            frame->input_buffer.fd, frame->input_buffer.buffer,
            frame->input_buffer.frame_len);
    if (NO_ERROR == rc) {
        mappedBuffer.index = buf_idx;
        mappedBuffer.stream = pStream;
        mappedBuffer.type = CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF;
        Mutex::Autolock lock(mOfflineBuffersLock);
        mOfflineBuffers.push_back(mappedBuffer);
        mOfflineBuffersIndex = (int32_t)buf_idx;
        LOGD("Mapped buffer with index %d", mOfflineBuffersIndex);
    }

    max_idx = (int32_t) ((mNumBuffers * 2) - 1);
    //loop back the indices if max burst count reached
    if (mOfflineMetaIndex == max_idx) {
       mOfflineMetaIndex = (int32_t) (mNumBuffers - 1);
    }
    uint32_t meta_buf_idx = (uint32_t)(mOfflineMetaIndex + 1);
    rc |= pStream->mapBuf(
            CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF,
            meta_buf_idx, -1,
            frame->metadata_buffer.fd, frame->metadata_buffer.buffer,
            frame->metadata_buffer.frame_len);
    if (NO_ERROR == rc) {
        mappedBuffer.index = meta_buf_idx;
        mappedBuffer.stream = pStream;
        mappedBuffer.type = CAM_MAPPING_BUF_TYPE_OFFLINE_META_BUF;
        Mutex::Autolock lock(mOfflineMetaBuffersLock);
        mOfflineMetaBuffers.push_back(mappedBuffer);
        mOfflineMetaIndex = (int32_t)meta_buf_idx;
        LOGD("Mapped meta buffer with index %d", mOfflineMetaIndex);
    }

    if (rc == NO_ERROR) {
        cam_stream_parm_buffer_t param;
        uint32_t numPendingPriorityFrames = 0;

        if(isPriorityFrame && (mReprocessType != REPROCESS_TYPE_JPEG)) {
            Mutex::Autolock lock(mPriorityFramesLock);
            /* read the length before pushing the frame number to check if
             * vector is empty */
            numPendingPriorityFrames = mPriorityFrames.size();
            mPriorityFrames.push(frame->frameNumber);
        }

        if(isPriorityFrame && !numPendingPriorityFrames &&
            (mReprocessType != REPROCESS_TYPE_JPEG)) {
            memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
            param.type = CAM_STREAM_PARAM_TYPE_REQUEST_OPS_MODE;
            param.perf_mode = CAM_PERF_HIGH_PERFORMANCE;
            rc = pStream->setParameter(param);
            if (rc != NO_ERROR) {
                LOGE("%s: setParameter for CAM_PERF_HIGH_PERFORMANCE failed",
                    __func__);
            }
            {
                Mutex::Autolock lock(mPriorityFramesLock);
                mReprocessPerfMode = true;
            }
        }

        memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
        param.type = CAM_STREAM_PARAM_TYPE_DO_REPROCESS;
        param.reprocess.buf_index = buf_idx;
        param.reprocess.frame_idx = frame->input_buffer.frame_idx;
        param.reprocess.meta_present = 1;
        param.reprocess.meta_buf_index = meta_buf_idx;

        LOGI("Offline reprocessing id = %d buf Id = %d meta index = %d",
                    param.reprocess.frame_idx, param.reprocess.buf_index,
                    param.reprocess.meta_buf_index);
        rc = pStream->setParameter(param);
        if (rc != NO_ERROR) {
            LOGE("stream setParameter for reprocess failed");
            resetToCamPerfNormal(frame->frameNumber);
        }
    } else {
        LOGE("Input buffer memory map failed: %d", rc);
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : doReprocess
 *
 * DESCRIPTION: request to do a reprocess on the frame
 *
 * PARAMETERS :
 *   @buf_fd     : fd to the input buffer that needs reprocess
 *   @buffer     : Buffer ptr
 *   @buf_lenght : length of the input buffer
 *   @ret_val    : result of reprocess.
 *                 Example: Could be faceID in case of register face image.
 *   @meta_frame : metadata frame.
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::doReprocess(int buf_fd, void *buffer, size_t buf_length,
        int32_t &ret_val, mm_camera_super_buf_t *meta_frame)
{
    int32_t rc = 0;
    if (m_numStreams < 1) {
        LOGE("No reprocess stream is created");
        return -1;
    }
    if (meta_frame == NULL) {
        LOGE("Did not get corresponding metadata in time");
        return -1;
    }

    uint8_t buf_idx = 0;
    for (uint32_t i = 0; i < m_numStreams; i++) {
        rc = mStreams[i]->mapBuf(CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF,
                                 buf_idx, -1,
                                 buf_fd, buffer, buf_length);

        //Do cache ops before sending for reprocess
        if (mMemory != NULL) {
            mMemory->cleanInvalidateCache(buf_idx);
        }

        if (rc == NO_ERROR) {
            cam_stream_parm_buffer_t param;
            memset(&param, 0, sizeof(cam_stream_parm_buffer_t));
            param.type = CAM_STREAM_PARAM_TYPE_DO_REPROCESS;
            param.reprocess.buf_index = buf_idx;
            param.reprocess.meta_present = 1;
            param.reprocess.meta_stream_handle = m_pMetaChannel->mStreams[0]->getMyServerID();
            param.reprocess.meta_buf_index = meta_frame->bufs[0]->buf_idx;

            LOGI("Online reprocessing id = %d buf Id = %d meta index = %d",
                    param.reprocess.frame_idx, param.reprocess.buf_index,
                    param.reprocess.meta_buf_index);
            rc = mStreams[i]->setParameter(param);
            if (rc == NO_ERROR) {
                ret_val = param.reprocess.ret_val;
            }
            mStreams[i]->unmapBuf(CAM_MAPPING_BUF_TYPE_OFFLINE_INPUT_BUF,
                                  buf_idx, -1);
        }
    }
    return rc;
}

/*===========================================================================
 * FUNCTION   : addMetaReprocStream
 *
 * DESCRIPTION: add metadata reprocess stream from metadata channel
 *
 * PARAMETERS :
 *   @config         : pp feature configuration
 *   @src_config     : source reprocess configuration
 *   @isType         : type of image stabilization required on this stream
 *   @pMetaChannel   : ptr to metadata channel to get corresp. metadata
 *
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::addMetaReprocStream(QCamera3Channel *pMetaChannel)
{
    int32_t rc = 0;
    cam_stream_reproc_config_t reproc_cfg;
    cam_dimension_t meta_dim = {0, 0};
    cam_format_t meta_stream_fmt = CAM_FORMAT_MAX;

    LOGD("E");

    QCamera3Stream *pMetaStream = pMetaChannel->getStreamByIndex(0);
    if (pMetaStream == NULL) {
        LOGE("fail to get metadata stream.");
        return -1;
    }
    mSrcStreamHandles[m_numStreams] = pMetaStream->getMyHandle();
    LOGD("meta stream:%p, handle:%p", pMetaStream, mSrcStreamHandles[m_numStreams]);

    meta_dim.width = (int32_t)sizeof(metadata_buffer_t),
    meta_dim.height = 1;

    memset(&reproc_cfg, 0, sizeof(cam_stream_reproc_config_t));
    reproc_cfg.pp_type = CAM_OFFLINE_REPROCESS_TYPE;

    // input stream config
    reproc_cfg.offline.input_fmt = meta_stream_fmt;
    reproc_cfg.offline.input_dim = meta_dim;
    mm_stream_calc_offset_metadata(&meta_dim, pMetaChannel->getPaddingInfo(),
            &reproc_cfg.offline.input_buf_planes);
    reproc_cfg.offline.num_of_bufs = mNumBuffers;
    reproc_cfg.offline.input_type = CAM_STREAM_TYPE_METADATA;

    reproc_cfg.pp_feature_config.feature_mask = CAM_QCOM_FEATURE_METADATA_PROCESSING;;

    QCamera3Stream *pStream = new QCamera3Stream(m_camHandle,
            m_handle,
            m_camOps,
            pMetaChannel->getPaddingInfo(),
            (QCamera3Channel*)this);

    rc = pStream->init(CAM_STREAM_TYPE_OFFLINE_PROC, meta_stream_fmt,
            meta_dim, ROTATE_0, &reproc_cfg,
            (uint8_t)mNumBuffers,
            reproc_cfg.pp_feature_config.feature_mask,
            IS_TYPE_NONE,
            0,/* batchSize */
            QCamera3Channel::streamCbRoutine, this);

    if (rc == 0) {
        mStreams[m_numStreams] = pStream;
        m_numStreams++;
    } else {
        LOGE("failed to create reprocess stream");
        delete pStream;
    }

    LOGD("X. rc = %d", rc);
    return rc;
}

/*===========================================================================
 * FUNCTION   : addReprocStreamsFromSource
 *
 * DESCRIPTION: add reprocess streams from input source channel
 *
 * PARAMETERS :
 *   @config         : pp feature configuration
 *   @src_config     : source reprocess configuration
 *   @isType         : type of image stabilization required on this stream
 *   @pMetaChannel   : ptr to metadata channel to get corresp. metadata
 *
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCamera3ReprocessChannel::addReprocStreamsFromSource(cam_pp_feature_config_t &pp_config,
        const reprocess_config_t &src_config , cam_is_type_t is_type,
        QCamera3Channel *pMetaChannel)
{
    int32_t rc = 0;
    cam_stream_reproc_config_t reprocess_config;
    cam_stream_type_t streamType;

    cam_dimension_t streamDim = src_config.output_stream_dim;
    cam_format_t streamFormat = src_config.output_stream_format;

    if (NULL != src_config.src_channel) {
        QCamera3Stream *pSrcStream = src_config.src_channel->getStreamByIndex(0);
        if (pSrcStream == NULL) {
           LOGE("source channel doesn't have a stream");
           return BAD_VALUE;
        }
        mSrcStreamHandles[m_numStreams] = pSrcStream->getMyHandle();
    }

    streamType = CAM_STREAM_TYPE_OFFLINE_PROC;
    reprocess_config.pp_type = CAM_OFFLINE_REPROCESS_TYPE;

    reprocess_config.offline.input_fmt = src_config.stream_format;
    reprocess_config.offline.input_dim = src_config.input_stream_dim;
    reprocess_config.offline.input_buf_planes.plane_info =
            src_config.input_stream_plane_info.plane_info;
    reprocess_config.offline.num_of_bufs = (uint8_t)mNumBuffers;
    reprocess_config.offline.input_type = src_config.stream_type;

    //overwrite input/output stream foramt/type/frame_offset in multi pass reproc
    QCamera3HardwareInterface* hal_obj = (QCamera3HardwareInterface*)mUserData;
    if (hal_obj->getReprocChannelCnt() == 2) {
        LOGI("modify input/output format for multi pass reprocess");
        if (pp_config.feature_mask & CAM_QCOM_FEATURE_QUADRA_CFA) {
            streamFormat = getStreamDefaultFormat(streamType, streamDim.width, streamDim.height);
            LOGD("override output format from:%d to %d",
                src_config.output_stream_format, streamFormat);
        } else if (pp_config.feature_mask & CAM_QCOM_FEATURE_RAW_PROCESSING) {
            reprocess_config.offline.input_fmt =
                getStreamDefaultFormat(streamType, streamDim.width, streamDim.height);
            LOGD("override input format from:%d to %d",
                src_config.stream_format, reprocess_config.offline.input_fmt);
        }
    }

    reprocess_config.pp_feature_config = pp_config;
    QCamera3Stream *pStream = new QCamera3Stream(m_camHandle,
            m_handle,
            m_camOps,
            &mPaddingInfo,
            (QCamera3Channel*)this);
    if (pStream == NULL) {
        LOGE("No mem for Stream");
        return NO_MEMORY;
    }

    rc = pStream->init(streamType, streamFormat,
            streamDim, ROTATE_0, &reprocess_config,
            (uint8_t)mNumBuffers,
            reprocess_config.pp_feature_config.feature_mask,
            is_type,
            0,/* batchSize */
            QCamera3Channel::streamCbRoutine, this);

    if (rc == 0) {
        mStreams[m_numStreams] = pStream;
        m_numStreams++;
    } else {
        LOGE("failed to create reprocess stream");
        delete pStream;
    }

    if (rc == NO_ERROR) {
        m_pSrcChannel = src_config.src_channel;
        m_pMetaChannel = pMetaChannel;
        mReprocessType = src_config.reprocess_type;
        LOGD("mReprocessType is %d", mReprocessType);
    }

    // meta reproc stream always appends as the last reproc stream
    if (pp_config.feature_mask & CAM_QCOM_FEATURE_RAW_PROCESSING) {
        LOGH("raw process mask is set, need offline isp and meta reprocess");
        m_bOfflineIsp = TRUE;
        addMetaReprocStream(pMetaChannel);
    }

    mm_camera_req_buf_t buf;
    memset(&buf, 0x0, sizeof(buf));
    buf.type = MM_CAMERA_REQ_SUPER_BUF;
    buf.num_buf_requested = 1;
    if(m_camOps->request_super_buf(m_camHandle,m_handle, &buf) < 0) {
        LOGE("Request for super buffer failed");
    }
    return rc;
}

bool QCamera3ReprocessChannel::isMetaReprocStream(QCamera3Stream *stream) {
    return (stream->getStreamInfo()->reprocess_config.offline.input_type
            == CAM_STREAM_TYPE_METADATA);
}


/* QCamera3SupportChannel methods */

cam_dimension_t QCamera3SupportChannel::kDim = {640, 480};

QCamera3SupportChannel::QCamera3SupportChannel(uint32_t cam_handle,
                    uint32_t channel_handle,
                    mm_camera_ops_t *cam_ops,
                    cam_padding_info_t *paddingInfo,
                    cam_feature_mask_t postprocess_mask,
                    cam_stream_type_t streamType,
                    cam_dimension_t *dim,
                    cam_format_t streamFormat,
                    cam_color_filter_arrangement_t color_arrangement,
                    void *userData, uint32_t numBuffers) :
                        QCamera3Channel(cam_handle, channel_handle, cam_ops,
                                NULL, NULL, paddingInfo, postprocess_mask,
                                userData, numBuffers),
                        mMemory(NULL)
{
    memcpy(&mDim, dim, sizeof(cam_dimension_t));
    mStreamType = streamType;
    mStreamFormat = streamFormat;
   // Make Analysis same as Preview format
   if ((mStreamFormat != CAM_FORMAT_Y_ONLY) &&
           (mStreamType == CAM_STREAM_TYPE_ANALYSIS) &&
           (color_arrangement != CAM_FILTER_ARRANGEMENT_Y)) {
        mStreamFormat = getStreamDefaultFormat(CAM_STREAM_TYPE_PREVIEW,
                dim->width, dim->height);
   }
}

QCamera3SupportChannel::~QCamera3SupportChannel()
{
    destroy();

    if (mMemory) {
        mMemory->deallocate();
        delete mMemory;
        mMemory = NULL;
    }
}

int32_t QCamera3SupportChannel::initialize(cam_is_type_t isType)
{
    int32_t rc;

    if (mMemory || m_numStreams > 0) {
        LOGE("metadata channel already initialized");
        return -EINVAL;
    }

    mIsType = isType;
    rc = QCamera3Channel::addStream(mStreamType,
            mStreamFormat, mDim, ROTATE_0, MIN_STREAMING_BUFFER_NUM,
            mPostProcMask, mIsType);
    if (rc < 0) {
        LOGE("addStream failed");
    }
    return rc;
}

int32_t QCamera3SupportChannel::request(buffer_handle_t * /*buffer*/,
                                                uint32_t /*frameNumber*/,
                                                int & /*indexUsed*/)
{
    return NO_ERROR;
}

void QCamera3SupportChannel::streamCbRoutine(
                        mm_camera_super_buf_t *super_frame,
                        QCamera3Stream * /*stream*/)
{
    if (super_frame == NULL || super_frame->num_bufs != 1) {
        LOGE("super_frame is not valid");
        return;
    }
    bufDone(super_frame);
    free(super_frame);
}

QCamera3StreamMem* QCamera3SupportChannel::getStreamBufs(uint32_t len)
{
    int rc;
    mMemory = new QCamera3StreamMem(mNumBuffers);
    if (!mMemory) {
        LOGE("unable to create heap memory");
        return NULL;
    }
    rc = mMemory->allocateAll(len);
    if (rc < 0) {
        LOGE("unable to allocate heap memory");
        delete mMemory;
        mMemory = NULL;
        return NULL;
    }
    return mMemory;
}

void QCamera3SupportChannel::putStreamBufs()
{
    mMemory->deallocate();
    delete mMemory;
    mMemory = NULL;
}

}; // namespace qcamera
