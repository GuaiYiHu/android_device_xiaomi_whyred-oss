/* Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "DualFOVPP"
// System dependencies
#include <dlfcn.h>
#include <utils/Errors.h>
#include <stdio.h>
#include <stdlib.h>
// Camera dependencies
#include "QCameraDualFOVPP.h"
#include "QCameraTrace.h"
#include "cam_intf.h"
extern "C" {
#include "mm_camera_dbg.h"
}

#define LIB_PATH_LENGTH 100

namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraDualFOVPP
 *
 * DESCRIPTION: constructor of QCameraDualFOVPP.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraDualFOVPP::QCameraDualFOVPP()
    : QCameraHALPP()
{
    m_dlHandle = NULL;
    m_pCaps = NULL;
}

/*===========================================================================
 * FUNCTION   : ~QCameraDualFOVPP
 *
 * DESCRIPTION: destructor of QCameraDualFOVPP.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraDualFOVPP::~QCameraDualFOVPP()
{
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of QCameraDualFOVPP
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
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraDualFOVPP::init(halPPBufNotify bufNotifyCb, halPPGetOutput getOutputCb,
        void *pUserData, void *pStaticParam)
{
    LOGD("E");
    int32_t rc = NO_ERROR;
    QCameraHALPP::init(bufNotifyCb, getOutputCb, pUserData);

    m_pCaps = (cam_capability_t *)pStaticParam;

    /* we should load 3rd libs here, with dlopen/dlsym */
    doDualFovPPInit();

    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : deinit
 *
 * DESCRIPTION: de initialization of QCameraDualFOVPP
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraDualFOVPP::deinit()
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
 * DESCRIPTION: starting QCameraDualFOVPP
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraDualFOVPP::start()
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
int32_t QCameraDualFOVPP::feedInput(qcamera_hal_pp_data_t *pInputData)
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
                uint32_t *pFrameIndex = new uint32_t;
                *pFrameIndex = frameIndex;
                // new the vector first
                pVector = new std::vector<qcamera_hal_pp_data_t*>(WIDE_TELE_CAMERA_NUMBER);
                pVector->at(WIDE_INPUT) = NULL;
                pVector->at(TELE_INPUT) = NULL;
                // Add vector to the hash map
                m_frameMap[frameIndex] = pVector;
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
            }
            pInputData->frameIndex = frameIndex;
            // Check if frame is from main wide camera
            bool bIsMain = true;
            uint32_t mainHandle = get_main_camera_handle(
                    pInputData->src_reproc_frame->camera_handle);
            if (mainHandle == 0) {
                bIsMain = false;
            }
            LOGD("mainHandle = %d, is main frame = %d", mainHandle, bIsMain);
            // Add input data to vector
            if (bIsMain) {
                pVector->at(WIDE_INPUT) = pInputData;
            } else {
                pVector->at(TELE_INPUT) = pInputData;
            }

            // request output buffer only if both wide and tele input data are recieved
            if (pVector->at(0) != NULL && pVector->at(1) != NULL) {
                m_halPPGetOutputCB(frameIndex, m_pHalPPMgr);
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
int32_t QCameraDualFOVPP::feedOutput(qcamera_hal_pp_data_t *pOutputData)
{
    int32_t rc = NO_ERROR;
    LOGD("E");

    if (NULL != pOutputData) {
        uint32_t frameIndex = pOutputData->frameIndex;
        std::vector<qcamera_hal_pp_data_t*> *pVector = getFrameVector(frameIndex);
        // Get the Wide/Tele input frame in order to decide output buffer len,
        // and copy metadata buffer.
        if (pVector != NULL && pVector->at(WIDE_INPUT) != NULL && pVector->at(TELE_INPUT) != NULL) {
            qcamera_hal_pp_data_t *pInputDataWide = pVector->at(WIDE_INPUT);
            qcamera_hal_pp_data_t *pInputDataTele = pVector->at(TELE_INPUT);

            mm_camera_buf_def_t *pBufWide = getSnapshotBuf(pInputDataWide);
            mm_camera_buf_def_t *pBufTele = getSnapshotBuf(pInputDataTele);
            if (pBufWide == NULL || pBufTele == NULL) {
                LOGE("%s buf is null",(pBufWide == NULL) ? "wide" : "tele");
                return UNEXPECTED_NULL;
            }
            LOGH("snapshot frame len, wide:%d, tele:%d", pBufWide->frame_len, pBufTele->frame_len);
            qcamera_hal_pp_data_t *pInputData = pInputDataWide;
            if (pBufTele->frame_len > pBufWide->frame_len) {
                pInputData = pInputDataTele;
            }

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
 * DESCRIPTION: Start FOV post process
 *
 * PARAMETERS : None
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraDualFOVPP::process()
{
    int32_t rc = NO_ERROR;

    /* dump in/out frames */
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    property_get("persist.vendor.camera.dualfov.dumpimg", prop, "0");
    int dumpimg = atoi(prop);

    LOGD("E");

    // TODO: dequeue from m_inputQ and start process logic
    // Start the blending process when it is ready
    if (canProcess()) {
        LOGI("start Dual FOV process");
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
                (qcamera_hal_pp_data_t *)pVector->at(WIDE_INPUT);
        if (pInputMainData == NULL) {
            LOGE("Cannot find input main data");
            return UNEXPECTED_NULL;
        }
        if (pInputMainData->src_reproc_frame == NULL) {
            LOGI("process pInputMainData->src_reproc_frame = NULL");
        }
        //mm_camera_super_buf_t *input_main_frame = input_main_data->frame;
        qcamera_hal_pp_data_t *pInputAuxData =
                (qcamera_hal_pp_data_t *)pVector->at(TELE_INPUT);
        if (pInputAuxData == NULL) {
            LOGE("Cannot find input aux data");
            return UNEXPECTED_NULL;
        }

        //mm_camera_super_buf_t *input_aux_frame = input_aux_data->frame;
        qcamera_hal_pp_data_t *pOutputData =
                (qcamera_hal_pp_data_t*)m_outgoingQ.dequeue();
        if (pOutputData == NULL) {
            LOGE("Cannot find output data");
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

        mm_camera_super_buf_t *output_frame = pOutputData->frame;
        mm_camera_buf_def_t *output_snapshot_buf = output_frame->bufs[0];

        // Use offset info from reproc stream
        cam_frame_len_offset_t frm_offset = pInputMainData->snap_offset;
        LOGI("<Wide> stride:%d, scanline:%d, frame len:%d",
                frm_offset.mp[0].stride, frm_offset.mp[0].scanline,
                frm_offset.frame_len);
        if (dumpimg) {
            dumpYUVtoFile((uint8_t *)main_snapshot_buf->buffer, frm_offset,
                    main_snapshot_buf->frame_idx, "wide");
        }

        frm_offset = pInputAuxData->snap_offset;
        LOGI("<Tele> stride:%d, scanline:%d, frame len:%d",
                frm_offset.mp[0].stride, frm_offset.mp[0].scanline,
                frm_offset.frame_len);
        if (dumpimg) {
            dumpYUVtoFile((uint8_t *)aux_snapshot_buf->buffer,  frm_offset,
                    aux_snapshot_buf->frame_idx,  "tele");
        }

        //Get input and output parameter
        dualfov_input_params_t inParams;
        getInputParams(main_meta_buf, aux_meta_buf,
                                pInputMainData->snap_offset,
                                pInputMainData->snap_offset,
                inParams);
        dumpInputParams(inParams);

        doDualFovPPProcess((const uint8_t *)main_snapshot_buf->buffer,
                        (const uint8_t *)aux_snapshot_buf->buffer,
                        inParams,
                        (uint8_t *)output_snapshot_buf->buffer);

        if (dumpimg) {
            frm_offset = pInputMainData->snap_offset;
            if (aux_snapshot_buf->frame_len > main_snapshot_buf->frame_len) {
                frm_offset = pInputAuxData->snap_offset;
            }
            dumpYUVtoFile((uint8_t *)output_snapshot_buf->buffer, frm_offset,
                    main_snapshot_buf->frame_idx, "out");
        }

        /* clean and invalidate caches, for input and output buffers*/
        pOutputData->snapshot_heap->cleanInvalidateCache(0);
        if (pInputMainData->jpeg_settings || pInputAuxData->jpeg_settings)
        {
            pOutputData->jpeg_settings = (jpeg_settings_t *)calloc(1,sizeof(jpeg_settings_t));
            if (pOutputData->jpeg_settings == NULL) {
                LOGE("No memory for src frame");
                free(pOutputData);
                return NO_MEMORY;
            }
            if(pInputMainData->jpeg_settings) {
               memcpy(pOutputData->jpeg_settings, pInputMainData->jpeg_settings,
                                                     sizeof(jpeg_settings_t));
            } else if (pInputAuxData->jpeg_settings) {
               memcpy(pOutputData->jpeg_settings, pInputAuxData->jpeg_settings,
                                                      sizeof(jpeg_settings_t));
            }
        }

        // Calling cb function to return output_data after processed.
        LOGH("CB for output");
        m_halPPBufNotifyCB(pOutputData, m_pHalPPMgr);

        // also send input buffer to postproc.
        LOGH("CB for wide input");
        m_halPPBufNotifyCB(pInputMainData, m_pHalPPMgr);
        LOGH("CB for tele input");
        m_halPPBufNotifyCB(pInputAuxData, m_pHalPPMgr);

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
 * RETURN     : true if Dual FOV is ready to process
 *==========================================================================*/
bool QCameraDualFOVPP::canProcess()
{
    LOGD("E");
    bool ready = false;
    if(!m_inputQ.isEmpty() && !m_outgoingQ.isEmpty()) {
        ready = true;
    }
    LOGD("X");
    return ready;
}

/*===========================================================================
 * FUNCTION   : getInputParams
 *
 * DESCRIPTION: Helper function to get input params from input metadata
 *==========================================================================*/
void QCameraDualFOVPP::getInputParams(mm_camera_buf_def_t *pMainMetaBuf,
        mm_camera_buf_def_t *pAuxMetaBuf, cam_frame_len_offset_t main_offset,
        cam_frame_len_offset_t aux_offset, dualfov_input_params_t& inParams)
{
    LOGD("E");
    memset(&inParams, 0, sizeof(dualfov_input_params_t));
    metadata_buffer_t *pMainMeta = (metadata_buffer_t *)pMainMetaBuf->buffer;
    metadata_buffer_t *pAuxMeta = (metadata_buffer_t *)pAuxMetaBuf->buffer;

    // Wide frame size
    inParams.wide.width     = main_offset.mp[0].width;
    inParams.wide.height    = main_offset.mp[0].height;
    inParams.wide.stride    = main_offset.mp[0].stride;
    inParams.wide.scanline  = main_offset.mp[0].scanline;
    inParams.wide.frame_len = main_offset.frame_len;

    // Tele frame size
    inParams.tele.width     = aux_offset.mp[0].width;
    inParams.tele.height    = aux_offset.mp[0].height;
    inParams.tele.stride    = aux_offset.mp[0].stride;
    inParams.tele.scanline  = aux_offset.mp[0].scanline;
    inParams.tele.frame_len = aux_offset.frame_len;

    // user_zoom
    int32_t zoom_level = -1; // 0 means zoom 1x.
    IF_META_AVAILABLE(cam_zoom_info_t, zoomInfoMain, CAM_INTF_PARM_USERZOOM, pMainMeta) {
        // Get main camera zoom value
        for (uint32_t i = 0; i < zoomInfoMain->num_streams; ++i) {
            if (zoomInfoMain->stream_zoom_info[i].stream_type == CAM_STREAM_TYPE_SNAPSHOT) {
                zoom_level = zoomInfoMain->stream_zoom_info[i].stream_zoom;
            }
        }
        LOGD("zoom level in main meta:%d", zoom_level);
    }
    inParams.user_zoom= getUserZoomRatio(zoom_level);
    LOGI("dual fov total zoom ratio: %d", inParams.user_zoom);

    IF_META_AVAILABLE(cam_zoom_info_t, zoomInfoAux, CAM_INTF_PARM_USERZOOM, pAuxMeta) {
        zoom_level = -1;
        // Get aux camera zoom value
        for (uint32_t i = 0; i < zoomInfoAux->num_streams; ++i) {
            if (zoomInfoAux->stream_zoom_info[i].stream_type == CAM_STREAM_TYPE_SNAPSHOT) {
                zoom_level = zoomInfoAux->stream_zoom_info[i].stream_zoom;
            }
        }
        LOGD("zoom level in aux meta:%d", zoom_level);
    }

    IF_META_AVAILABLE(uint32_t, afState, CAM_INTF_META_AF_STATE, pMainMeta) {
        if (((*afState) == CAM_AF_STATE_FOCUSED_LOCKED) ||
            ((*afState) == CAM_AF_STATE_PASSIVE_FOCUSED)) {
            inParams.af_status = AF_STATUS_VALID;
        } else {
            inParams.af_status = AF_STATUS_INVALID;
        }
        LOGD("af state:%d, output af status:%d", *afState, inParams.af_status);
    }

    IF_META_AVAILABLE(uint32_t, auxAfState, CAM_INTF_META_AF_STATE, pAuxMeta) {
        int aux_af_status = 0;
        if (((*auxAfState) == CAM_AF_STATE_FOCUSED_LOCKED) ||
            ((*auxAfState) == CAM_AF_STATE_PASSIVE_FOCUSED)) {
            aux_af_status = AF_STATUS_VALID;
        } else {
            aux_af_status = AF_STATUS_INVALID;
        }
        LOGD("aux af state:%d, output af status:%d", *auxAfState, aux_af_status);
    }


    LOGD("X");
}


int32_t QCameraDualFOVPP::doDualFovPPInit()
{
    LOGD("E");
    int rc = NO_ERROR;

    LOGD("X");
    return rc;
}

int32_t QCameraDualFOVPP::doDualFovPPProcess(const uint8_t* pWide, const uint8_t* pTele,
                                                    dualfov_input_params_t inParams,
                                                    uint8_t* pOut)
{
    LOGW("E.");

    // trace begin

    if (inParams.tele.frame_len == inParams.wide.frame_len) {
        LOGD("copy to output, half from wide, half from tele..");

        // half image from main, and half image from tele

        // Y
        memcpy(pOut, pWide, inParams.wide.stride * inParams.wide.scanline / 2);
        memcpy(pOut  + inParams.wide.stride * inParams.wide.scanline / 2,
               pTele + inParams.wide.stride * inParams.wide.scanline / 2,
               inParams.wide.stride * inParams.wide.scanline / 2);

        // UV
        uint32_t uv_offset = inParams.wide.stride * inParams.wide.scanline;
        memcpy(pOut  + uv_offset,
               pWide + uv_offset,
               inParams.wide.stride * (inParams.wide.scanline / 2) / 2);
        memcpy(pOut  + uv_offset + inParams.wide.stride * (inParams.wide.scanline / 2) / 2,
               pTele + uv_offset + inParams.wide.stride * (inParams.wide.scanline / 2) / 2,
               inParams.wide.stride * (inParams.wide.scanline / 2) / 2);

    } else {
        // copy the larger input buffer to output
        const uint8_t* pIn = pWide;
        uint32_t len = inParams.wide.frame_len;

        if (inParams.tele.frame_len > inParams.wide.frame_len) {
            LOGD("copy tele to output");
            pIn = pTele;
            len = inParams.tele.frame_len;
        } else {
            LOGD("copy wide to output");
            pIn = pWide;
            len = inParams.wide.frame_len;
        }

        memcpy(pOut, pIn, len);
    }

    // trace end

    LOGW("X.");
    return NO_ERROR;
}

uint32_t QCameraDualFOVPP::getUserZoomRatio(int32_t zoom_level)
{
    uint32_t zoom_ratio = 4096;

    LOGD("E. input zoom level:%d", zoom_level);

    if (zoom_level < 0) {
        LOGW("invalid zoom evel!");
        /* got the zoom value from QCamera2HWI Parameters */
        zoom_level = 0;
    }

    // user_zoom_ratio = qcom_zoom_ratio * 4096 / 100
    if (m_pCaps != NULL) {
        zoom_ratio *= m_pCaps->zoom_ratio_tbl[zoom_level];
        zoom_ratio /= 100;
        LOGD("converted zoom ratio:%d", zoom_ratio);
    }

    LOGD("X. zoom_ratio:%d", zoom_ratio);
    return zoom_ratio;
}

void QCameraDualFOVPP::dumpYUVtoFile(const uint8_t* pBuf, cam_frame_len_offset_t offset, uint32_t idx, const char* name_prefix)
{
    LOGD("E.");
    char filename[256];

    snprintf(filename, sizeof(filename), QCAMERA_DUMP_FRM_LOCATION"%s_%dx%d_%d.yuv",
                name_prefix, offset.mp[0].stride, offset.mp[0].scanline, idx);

    QCameraHALPP::dumpYUVtoFile(pBuf,(const char*)filename, offset.frame_len);

    LOGD("X.");
}

void QCameraDualFOVPP::dumpInputParams(const dualfov_input_params_t& p)
{
    LOGD("E");

    const cam_frame_size_t* s = NULL;

    s = &p.wide;
    LOGD("wide frame size: %d, %d, stride:%d, scanline:%d",
            s->width, s->height, s->stride, s->scanline);

    s = &p.tele;
    LOGD("wide frame size: %d, %d, stride:%d, scanline:%d",
            s->width, s->height, s->stride, s->scanline);

    LOGD("zoom ratio: %f", p.user_zoom / 4096.0);
    LOGD("X");
}


/*===========================================================================
 * FUNCTION   : dumpOISData
 *
 * DESCRIPTION: Read Sensor OIS data from metadata and dump it
 *
 * PARAMETERS :
 * @pMetadata : Frame metadata
 *
 * RETURN     : None
 *
 *==========================================================================*/
void QCameraDualFOVPP::dumpOISData(metadata_buffer_t*  pMetadata)
{
    if (!pMetadata) {
        LOGD("OIS data not available");
        return;
    }

    IF_META_AVAILABLE(cam_ois_data_t, pOisData, CAM_INTF_META_OIS_READ_DATA, pMetadata) {
        LOGD("Ois Data: data size: %d", pOisData->size);
        uint8_t *data = pOisData->data;
        if (pOisData->size == 8) {
            LOGD("Ois Data Buffer : %d %d %d %d %d %d %d %d ",
                    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        }
    }
    return;
}


} // namespace qcamera
