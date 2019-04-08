/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "ClearSight"
// System dependencies
#include <dlfcn.h>
#include <utils/Errors.h>
#include <stdio.h>
#include <stdlib.h>
// Camera dependencies
#include "QCameraClearSight.h"
#include "QCameraTrace.h"
#include "cam_intf.h"
extern "C" {
#include "mm_camera_dbg.h"
}

#define NUM_CLEARSIGHT_BUFS 3

namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraClearSight
 *
 * DESCRIPTION: constructor of QCameraClearSight.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraClearSight::QCameraClearSight()
    : QCameraHALPP()
{
    m_dlHandle = NULL;
    m_pCaps = NULL;
}

/*===========================================================================
 * FUNCTION   : ~QCameraClearSight
 *
 * DESCRIPTION: destructor of QCameraClearSight.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraClearSight::~QCameraClearSight()
{
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of QCameraClearSight
 *
 * PARAMETERS :
 *   @bufNotifyCb      : call back function after HALPP process
 *   @getOutputCb      : call back function to request output buffe
 *   @pUserData        : Parent of HALPP, i.e. QCameraPostProc
 *   @pStaticParam     : holds dual camera calibration data in an array and its size
 *                       (expected size is 264 bytes)
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              non-zero failure code
 *==========================================================================*/
int32_t QCameraClearSight::init(halPPBufNotify bufNotifyCb, halPPGetOutput getOutputCb,
        void *pUserData, void *pStaticParam)
{
    LOGD("E");
    int32_t rc = NO_ERROR;
    QCameraHALPP::init(bufNotifyCb, getOutputCb, pUserData);

    m_pCaps = (cam_capability_t *)pStaticParam;

    doClearSightInit();

    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : deinit
 *
 * DESCRIPTION: de initialization of QCameraClearSight
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraClearSight::deinit()
{
    int32_t rc = NO_ERROR;
    LOGD("E");

    m_dlHandle = NULL;

    QCameraHALPP::deinit();
    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: starting QCameraClearSight
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraClearSight::start()
{
    int32_t rc = NO_ERROR;
    LOGD("E");

    rc = QCameraHALPP::start();

    LOGD("X");
    return rc;
}


/*===========================================================================
 * FUNCTION   : feedInput
 *
 * DESCRIPTION: function to feed input data.
 *              Enqueue the frame index to inputQ if it is new frame
 *              Also, add the input image data to frame hash map
 *
 * PARAMETERS :
 *   @pInput  : ptr to input data
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraClearSight::feedInput(qcamera_hal_pp_data_t *pInputData)
{
    int32_t rc = NO_ERROR;
    LOGD("E");
    if (NULL != pInputData) {
        mm_camera_buf_def_t *pInputSnapshotBuf = getSnapshotBuf(pInputData);
        if (pInputSnapshotBuf != NULL) {
            uint32_t frameIndex = pInputSnapshotBuf->frame_idx;
            std::vector<qcamera_hal_pp_data_t*> *pVector = getFrameVector(frameIndex);
            if(pVector == NULL) {
                LOGD("insert new frame index = %d", frameIndex);
                // new the vector first
                pVector = new std::vector<qcamera_hal_pp_data_t*>(MM_CAMERA_MAX_CAM_CNT);
                // Add vector to the hash map
                m_frameMap[frameIndex] = pVector;
            } else {
                uint32_t *pFrameIndex = new uint32_t;
                *pFrameIndex = frameIndex;
                // Enqueue the frame index (i.e. key of vector) to queue
                if (false == m_inputQ.enqueue((void*)pFrameIndex)) {
                    LOGE("Input Q is not active!!!");
                    releaseData(pInputData);
                    m_frameMap.erase(frameIndex);
                    delete pFrameIndex;
                    delete pVector;
                    rc = INVALID_OPERATION;
                    return rc;
                }
                if (m_inputQ.getCurrentSize() == MIN_CLEARSIGHT_BUFS) {
                    m_halPPGetOutputCB(frameIndex, m_pHalPPMgr);
                }
            }
            pInputData->frameIndex = frameIndex;
            // Check if frame is from main camera
            bool bIsMain = true;
            uint32_t mainHandle = get_main_camera_handle(
                    pInputData->src_reproc_frame->camera_handle);
            if (mainHandle == 0) {
                bIsMain = false;
            }
            LOGD("mainHandle = %d, is main frame = %d", mainHandle, bIsMain);
            // Add input data to vector
            if (bIsMain) {
                pVector->at(BAYER_INPUT) = pInputData;
            } else {
                pVector->at(MONO_INPUT) = pInputData;
            }
        }
    } else {
        LOGE("pInput is NULL");
        rc = UNEXPECTED_NULL;
    }
    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : feedOutput
 *
 * DESCRIPTION: function to feed output buffer and metadata
 *
 * PARAMETERS :
 *   @pOutput     : ptr to output data
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraClearSight::feedOutput(qcamera_hal_pp_data_t *pOutputData)
{
    int32_t rc = NO_ERROR;
    LOGD("E");
    if (NULL != pOutputData) {
        uint32_t frameIndex = pOutputData->frameIndex;
        std::vector<qcamera_hal_pp_data_t*> *pVector = getFrameVector(frameIndex);
        // Get the main input frame in order to get output buffer len,
        // and copy metadata buffer.
        if (pVector != NULL && pVector->at(BAYER_INPUT) != NULL) {
            qcamera_hal_pp_data_t *pInputData = pVector->at(BAYER_INPUT);
            rc = getOutputBuffer(pInputData, pOutputData);
            // Enqueue output_data to m_outgoingQ
            if (false == m_outgoingQ.enqueue((void *)pOutputData)) {
                LOGE("outgoing Q is not active!!!");
                releaseData(pOutputData);
                rc = INVALID_OPERATION;
            }
        }
    } else {
        LOGE("pOutput is NULL");
        rc = UNEXPECTED_NULL;
    }
    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : process
 *
 * DESCRIPTION: function to start ClearSight blending process
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraClearSight::process()
{
    int32_t rc = NO_ERROR;

    /* dump in/out frames */
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.vendor.camera.dualfov.dumpimg", prop, "0");
    int dumpimg = atoi(prop);

    LOGD("E");

    if (!canProcess()) {
        LOGD("Yet to receive all input/output bufs");
        return NO_ERROR;
    }

    LOGI("start clearsight process");

    // TODO: dequeue from m_inputQ and start process logic
    // Start the blending process when it is ready
    while (!m_inputQ.isEmpty()) {
        uint32_t *pFrameIndex = (uint32_t *)m_inputQ.dequeue();
        if (pFrameIndex == NULL) {
            LOGE("frame index is null");
            return UNEXPECTED_NULL;
        }
        uint32_t frameIndex = *pFrameIndex;
        std::vector<qcamera_hal_pp_data_t*> *pVector = getFrameVector(frameIndex);
        // Search vector of input frames in frame map
        if (pVector == NULL) {
            LOGE("Cannot find vecotr of input frames");
            return UNEXPECTED_NULL;
        }
        // Get input and output frame buffer
        qcamera_hal_pp_data_t *pInputMainData =
                (qcamera_hal_pp_data_t *)pVector->at(BAYER_INPUT);
        if (pInputMainData == NULL) {
            LOGE("Cannot find input main data");
            return UNEXPECTED_NULL;
        }
        if (pInputMainData->src_reproc_frame == NULL) {
            LOGI("process pInputMainData->src_reproc_frame = NULL");
        }
        //mm_camera_super_buf_t *input_main_frame = input_main_data->frame;
        qcamera_hal_pp_data_t *pInputAuxData =
                (qcamera_hal_pp_data_t *)pVector->at(MONO_INPUT);
        if (pInputAuxData == NULL) {
            LOGE("Cannot find input aux data");
            return UNEXPECTED_NULL;
        }

        mm_camera_buf_def_t *main_snapshot_buf =
                getSnapshotBuf(pInputMainData);
        if (main_snapshot_buf == NULL) {
            LOGE("main_snapshot_buf is NULL");
            return UNEXPECTED_NULL;
        }
        mm_camera_buf_def_t *main_meta_buf = getMetadataBuf(pInputMainData);
        if (main_meta_buf == NULL) {
            LOGE("main_meta_buf is NULL");
            return UNEXPECTED_NULL;
        }
        mm_camera_buf_def_t *aux_snapshot_buf = getSnapshotBuf(pInputAuxData);
        if (aux_snapshot_buf == NULL) {
            LOGE("aux_snapshot_buf is NULL");
            return UNEXPECTED_NULL;
        }
        mm_camera_buf_def_t *aux_meta_buf = getMetadataBuf(pInputAuxData);
        if (aux_meta_buf == NULL) {
            LOGE("aux_meta_buf is NULL");
            return UNEXPECTED_NULL;
        }

        // Use offset info from reproc stream
        cam_frame_len_offset_t frm_offset = pInputMainData->snap_offset;
        LOGI("Main stride:%d, scanline:%d, frame len:%d",
                frm_offset.mp[0].stride, frm_offset.mp[0].scanline,
                frm_offset.frame_len);

        if (dumpimg) {
            dumpYUVtoFile((uint8_t *)main_snapshot_buf->buffer, frm_offset,
                    main_snapshot_buf->frame_idx, "bayer");
            dumpYUVtoFile((uint8_t *)aux_snapshot_buf->buffer,  frm_offset,
                    aux_snapshot_buf->frame_idx,  "mono");
        }

        //Get input and output parameter
        clearsight_input_params_t inParams;
        getInputParams(main_meta_buf, aux_meta_buf,
                                pInputMainData->snap_offset,
                                pInputAuxData->snap_offset,
                inParams);

        if (main_snapshot_buf->frame_idx == aux_snapshot_buf->frame_idx)
            inParams.frame_idx = main_snapshot_buf->frame_idx;
        else {
            LOGE("something wrong !!!");
        }
        dumpInputParams(inParams);

        qcamera_hal_pp_data_t *pOutputData = NULL;
        if (m_inputQ.isEmpty()) {
            pOutputData = (qcamera_hal_pp_data_t*)m_outgoingQ.dequeue();
            if (pOutputData == NULL) {
                LOGE("Cannot find output data");
                return UNEXPECTED_NULL;
            }
            mm_camera_super_buf_t *output_frame = pOutputData->frame;
            mm_camera_buf_def_t *output_snapshot_buf = output_frame->bufs[0];

            doClearSightProcess((const uint8_t *)main_snapshot_buf->buffer,
                            (const uint8_t *)aux_snapshot_buf->buffer,
                            inParams,
                            (uint8_t *)output_snapshot_buf->buffer);

            if (dumpimg) {
                dumpYUVtoFile((uint8_t *)output_snapshot_buf->buffer, frm_offset,
                        main_snapshot_buf->frame_idx, "out");
            }

            /* clean and invalidate caches, for input and output buffers*/
            pOutputData->snapshot_heap->cleanInvalidateCache(0);
        }

        QCameraMemory *pMem = (QCameraMemory *)main_snapshot_buf->mem_info;
        pMem->invalidateCache(main_snapshot_buf->buf_idx);

        pMem = (QCameraMemory *)aux_snapshot_buf->mem_info;
        pMem->invalidateCache(aux_snapshot_buf->buf_idx);


        // Calling cb function to return output_data after processed.
        if (pOutputData)
            m_halPPBufNotifyCB(pOutputData, m_pHalPPMgr);

        // also send input buffer to postproc.
        m_halPPBufNotifyCB(pInputMainData, m_pHalPPMgr);
        m_halPPBufNotifyCB(pInputAuxData, m_pHalPPMgr);
        //releaseData(pInputMainData);
        //releaseData(pInputAuxData);

        // Release internal resource
        m_frameMap.erase(frameIndex);
        delete pFrameIndex;
        delete pVector;
    }
    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : canProcess
 *
 * DESCRIPTION: function to release internal resources
 * RETURN     : If ClearSight module can start blending process
 *==========================================================================*/
bool QCameraClearSight::canProcess()
{
    LOGD("E");
    bool ready = false;
    if(!m_inputQ.isEmpty() && !m_outgoingQ.isEmpty()) {
        ready = true;
    }
    LOGD("X: ready = %d", ready);
    return ready;
}

/*===========================================================================
 * FUNCTION   : getInputParams
 *
 * DESCRIPTION: Helper function to get input params from input metadata
 *==========================================================================*/
void QCameraClearSight::getInputParams(__unused mm_camera_buf_def_t *pMainMetaBuf,
        __unused mm_camera_buf_def_t *pAuxMetaBuf, cam_frame_len_offset_t main_offset,
                cam_frame_len_offset_t aux_offset, clearsight_input_params_t& inParams)
{
    LOGD("E");
    memset(&inParams, 0, sizeof(clearsight_input_params_t));

    // bayer frame size
    inParams.bayer.width     = main_offset.mp[0].width;
    inParams.bayer.height    = main_offset.mp[0].height;
    inParams.bayer.stride    = main_offset.mp[0].stride;
    inParams.bayer.scanline  = main_offset.mp[0].scanline;
    inParams.bayer.frame_len = main_offset.frame_len;

    // mono frame size
    inParams.mono.width     = aux_offset.mp[0].width;
    inParams.mono.height    = aux_offset.mp[0].height;
    inParams.mono.stride    = aux_offset.mp[0].stride;
    inParams.mono.scanline  = aux_offset.mp[0].scanline;
    inParams.mono.frame_len = aux_offset.frame_len;

    LOGD("X");
}

/*===========================================================================
 * FUNCTION   : doClearSightInit
 *
 * DESCRIPTION: function to do required initialization for clearsight processing.
 * RETURN     :  int32_t type of status
 *                   NO_ERROR  -- success
 *                   non-zero -- failure code
 *==========================================================================*/
int32_t QCameraClearSight::doClearSightInit()
{
    LOGD("E");
    int rc = NO_ERROR;

    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : doClearSightProcess
 *
 * DESCRIPTION: clearsight processing routine
 * RETURN     :  int32_t type of status
 *                   NO_ERROR  -- success
 *                   non-zero -- failure code
 *==========================================================================*/

int32_t QCameraClearSight::doClearSightProcess(const uint8_t* pBayer, const uint8_t* pMono,
        clearsight_input_params_t inParams, uint8_t* pOut)
{
    LOGD("E");

    // trace begin

    // half image from bayer, and half image from mono

    // Y
    memcpy(pOut, pBayer, inParams.bayer.stride * inParams.bayer.scanline / 2);
    memcpy(pOut  + inParams.bayer.stride * inParams.bayer.scanline / 2,
           pMono + inParams.bayer.stride * inParams.bayer.scanline / 2,
           inParams.bayer.stride * inParams.bayer.scanline / 2);

    // UV
    uint32_t uv_offset = inParams.bayer.stride * inParams.bayer.scanline;
    memcpy(pOut  + uv_offset,
           pBayer + uv_offset,
           inParams.bayer.stride * (inParams.bayer.scanline / 2) / 2);
    memcpy(pOut  + uv_offset + inParams.bayer.stride * (inParams.bayer.scanline / 2) / 2,
           pMono + uv_offset + inParams.bayer.stride * (inParams.bayer.scanline / 2) / 2,
           inParams.bayer.stride * (inParams.bayer.scanline / 2) / 2);

    // trace end

    LOGW("X.");
    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : dumpYUVtoFile
 *
 * DESCRIPTION: dumps yuv for debug purpose
 * RETURN     :  None
 *==========================================================================*/
void QCameraClearSight::dumpYUVtoFile(const uint8_t* pBuf, cam_frame_len_offset_t offset,
        uint32_t idx, const char* name_prefix)
{
    LOGD("E.");
    char filename[256];

    snprintf(filename, sizeof(filename), QCAMERA_DUMP_FRM_LOCATION"%s_%dx%d_%d.yuv",
                name_prefix, offset.mp[0].stride, offset.mp[0].scanline, idx);

    QCameraHALPP::dumpYUVtoFile(pBuf,(const char*)filename, offset.frame_len);

    LOGD("X.");
}

/*===========================================================================
 * FUNCTION   : dumpInputParams
 *
 * DESCRIPTION: dumps input params for debug purpose
 * RETURN     :  None
 *==========================================================================*/
void QCameraClearSight::dumpInputParams(const clearsight_input_params_t& p)
{
    LOGD("E : frame idx %d", p.frame_idx);

    const cam_frame_size_t* s = NULL;

    s = &p.bayer;
    LOGD("bayer frame size: %d, %d, stride:%d, scanline:%d",
            s->width, s->height, s->stride, s->scanline);

    s = &p.mono;
    LOGD("bayer frame size: %d, %d, stride:%d, scanline:%d",
            s->width, s->height, s->stride, s->scanline);

    LOGD("X");
}

} // namespace qcamera
