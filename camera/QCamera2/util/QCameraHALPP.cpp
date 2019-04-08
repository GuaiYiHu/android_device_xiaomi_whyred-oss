/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "QCameraHALPP"

// Camera dependencies
#include "QCameraTrace.h"
#include "QCameraHALPP.h"
#include "QCameraQueue.h"
extern "C" {
#include "mm_camera_dbg.h"
}

namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraHALPP
 *
 * DESCRIPTION: constructor of QCameraHALPP.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraHALPP::QCameraHALPP()
    : m_inputQ(releaseInputDataCb, this),
      m_outgoingQ(releaseOngoingDataCb, this),
      m_halPPBufNotifyCB(NULL),
      m_halPPGetOutputCB(NULL),
      m_pHalPPMgr(NULL)
{
}

/*===========================================================================
 * FUNCTION   : ~QCameraHALPP
 *
 * DESCRIPTION: destructor of QCameraHALPP.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraHALPP::~QCameraHALPP()
{
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: initialization of QCameraHALPP
 *
 * PARAMETERS :
 *   @bufNotifyCb      : call back function after HALPP process done and return frame
 *   @getOutputCb      : call back function to request output buffer
 *   @pUserData        : Parent of HALPP, i.e. QCameraPostProc
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraHALPP::init(halPPBufNotify bufNotifyCb, halPPGetOutput getOutputCb, void *pUserData)
{
    int32_t rc = NO_ERROR;
    LOGH("E");
    // connect HALPP call back function
    m_halPPBufNotifyCB = bufNotifyCb;
    m_halPPGetOutputCB = getOutputCb;
    m_pHalPPMgr = (QCameraHALPPManager*)pUserData;
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : deinit
 *
 * DESCRIPTION: de-initialization of QCameraHALPP
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraHALPP::deinit()
{
    int32_t rc = NO_ERROR;
    m_halPPBufNotifyCB = NULL;
    m_halPPGetOutputCB = NULL;
    m_pHalPPMgr = NULL;
    return rc;
}

/*===========================================================================
 * FUNCTION   : start
 *
 * DESCRIPTION: starting QCameraHALPP
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraHALPP::start()
{
    int32_t rc = NO_ERROR;
    LOGD("E");

    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : stop
 *
 * DESCRIPTION: stop QCameraHALPP
 *
 * PARAMETERS :
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraHALPP::stop()
{
    int32_t rc = NO_ERROR;
    LOGD("E");

    LOGD("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : flushQ
 *
 * DESCRIPTION: flush m_inputQ and m_outgoingQ.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
int32_t QCameraHALPP::flushQ()
{
    int32_t rc = NO_ERROR;
    m_inputQ.flush();
    m_outgoingQ.flush();
    return rc;
}

/*===========================================================================
 * FUNCTION   : initQ
 *
 * DESCRIPTION: init m_inputQ and m_outgoingQ.
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
int32_t QCameraHALPP::initQ()
{
    int32_t rc = NO_ERROR;
    m_inputQ.init();
    m_outgoingQ.init();
    return rc;
}

/*===========================================================================
 * FUNCTION   : getFrameVector
 *
 * DESCRIPTION: get vector of input frames from map
 *
 * PARAMETERS :
 *   @frameIndex      : frame index (key of the map)
 *
 * RETURN     : vector pointer
 *==========================================================================*/
std::vector<qcamera_hal_pp_data_t*>*
        QCameraHALPP::getFrameVector(uint32_t frameIndex)
{
    std::vector<qcamera_hal_pp_data_t*> *pVector = NULL;
    // Search vector of input frames in frame map
    if (m_frameMap.find(frameIndex) != m_frameMap.end()) {
        pVector = m_frameMap[frameIndex];
    }
    return pVector;
}

/*===========================================================================
 * FUNCTION   : releaseData
 *
 * DESCRIPTION: release buffer in qcamera_hal_pp_data_t
 *
 * PARAMETERS :
 *   @pData      : hal pp data
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraHALPP::releaseData(qcamera_hal_pp_data_t *pData)
{
    LOGH("E");
    m_pHalPPMgr->releaseData(pData);
    LOGH("X");
}

/*===========================================================================
 * FUNCTION   : releaseOngoingDataCb
 *
 * DESCRIPTION: callback function to release ongoing data node
 *
 * PARAMETERS :
 *   @pData     : ptr to ongoing job data
 *   @pUserData : user data ptr (QCameraHALPP)
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraHALPP::releaseOngoingDataCb(void *pData, void *pUserData)
{
    if (pUserData != NULL && pData != NULL) {
        QCameraHALPP *pme = (QCameraHALPP *)pUserData;
        LOGH("releasing Data!!!!");
        pme->releaseData((qcamera_hal_pp_data_t*)pData);
    }
}

/*===========================================================================
 * FUNCTION   : releaseInputDataCb
 *
 * DESCRIPTION: callback function to release input data node
 *
 * PARAMETERS :
 *   @pData     : ptr to input job data
 *   @pUserData : user data ptr (QCameraHALPP)
 *
 * RETURN     : None
 *==========================================================================*/
void QCameraHALPP::releaseInputDataCb(void *pData, void *pUserData)
{
    if (pUserData != NULL && pData != NULL) {
        LOGH("releasing now");
        QCameraHALPP *pme = (QCameraHALPP *)pUserData;
        // what enqueued to the input queue is just the frame index
        // we need to use hash map to find the vector of frames and release the buffers
        uint32_t *pFrameIndex = (uint32_t *)pData;
        uint32_t frameIndex = *pFrameIndex;
        std::vector<qcamera_hal_pp_data_t*> *pVector = pme->getFrameVector(frameIndex);
        if (pVector != NULL) {
            for (size_t i = 0; i < pVector->size(); i++) {
                if (pVector->at(i) != NULL) {
                    pme->releaseData(pVector->at(i));
                }
            }
            delete pVector;
            pVector = NULL;
        }
        delete pFrameIndex;
        pFrameIndex = NULL;
    }
}

/*===========================================================================
 * FUNCTION   : getOutputBuffer
 *
 * DESCRIPTION: function to get snapshot buf def and the stream from frame
 * PARAMETERS :
 *   @pInputData   : input frame pp data
 *   @pOutputData : ouput frame pp data
 * RETURN             : NO_ERROR if successful else error type
 *==========================================================================*/
int32_t QCameraHALPP::getOutputBuffer(
        qcamera_hal_pp_data_t *pInputData,
        qcamera_hal_pp_data_t *pOutputData)
{
    int32_t rc = NO_ERROR;
    LOGH("E");
    if (!pInputData || !pOutputData) {
        LOGE("Error!! Cannot generate output buffer : pInputData: %p pOutputData:%p, ",
                pInputData, pOutputData);
        return UNEXPECTED_NULL;
    }

    mm_camera_super_buf_t *pInputFrame = pInputData->frame;
    mm_camera_buf_def_t *pInputSnapshotBuf = getSnapshotBuf(pInputData);
    mm_camera_buf_def_t *pInputMetadataBuf = getMetadataBuf(pInputData);
    mm_camera_super_buf_t *pOutputFrame = pOutputData->frame;
    mm_camera_buf_def_t *pOutputBufDefs = pOutputData->bufs;

    if ((pInputSnapshotBuf == NULL) || (pInputMetadataBuf == NULL)) {
        LOGE("cannot get sanpshot or metadata buf def");
        releaseData(pOutputData);
        return UNEXPECTED_NULL;
    }

    // Copy main input frame info to output frame
    pOutputFrame->camera_handle = pInputFrame->camera_handle;
    pOutputFrame->ch_id = pInputFrame->ch_id;
    pOutputFrame->num_bufs = HAL_PP_NUM_BUFS;//snapshot and metadata
    pOutputFrame->bUnlockAEC = pInputFrame->bUnlockAEC;
    pOutputFrame->bReadyForPrepareSnapshot = pInputFrame->bReadyForPrepareSnapshot;

    // Reconstruction of output_frame super buffer
    pOutputFrame->bufs[0] = &pOutputBufDefs[0];
    pOutputFrame->bufs[1] = &pOutputBufDefs[1];

    // Allocate heap buffer for output image frame
    LOGH("pInputSnapshotBuf->frame_len = %d", pInputSnapshotBuf->frame_len);
    rc = pOutputData->snapshot_heap->allocate(1, pInputSnapshotBuf->frame_len);
    if (rc < 0) {
        LOGE("Unable to allocate heap memory for image buf");
        releaseData(pOutputData);
        return NO_MEMORY;
    }

    memcpy(&pOutputBufDefs[0], pInputSnapshotBuf, sizeof(mm_camera_buf_def_t));
    LOGH("pOutputFrame->bufs[0]->fd = %d, pOutputFrame->bufs[0]->buffer = %x",
            pOutputFrame->bufs[0]->fd, pOutputFrame->bufs[0]->buffer);
    pOutputData->snapshot_heap->getBufDef(pInputData->snap_offset, pOutputBufDefs[0], 0);
    LOGH("pOutputFrame->bufs[0]->fd = %d, pOutputFrame->bufs[0]->buffer = %x",
            pOutputFrame->bufs[0]->fd, pOutputFrame->bufs[0]->buffer);

    // Allocate heap buffer for output metadata
    LOGH("pInputMetadataBuf->frame_len = %d", pInputMetadataBuf->frame_len);
    rc = pOutputData->metadata_heap->allocate(1, pInputMetadataBuf->frame_len);
    if (rc < 0) {
        LOGE("Unable to allocate heap memory for metadata buf");
        releaseData(pOutputData);
        return NO_MEMORY;
    }

    memcpy(&pOutputBufDefs[1], pInputMetadataBuf, sizeof(mm_camera_buf_def_t));
    pOutputData->metadata_heap->getBufDef(pInputData->meta_offset, pOutputBufDefs[1], 0);
    // copy the whole metadata
    memcpy(pOutputBufDefs[1].buffer, pInputMetadataBuf->buffer,
            pInputMetadataBuf->frame_len);

    pOutputData->pUserData = pInputData->pUserData;
    pOutputData->jpeg_settings = pInputData->output_jpeg_settings;
    LOGH("X");
    return rc;
}

/*===========================================================================
 * FUNCTION   : getSnapshotBuf
 *
 * DESCRIPTION: function to get snapshot buf def and the stream from frame
 * PARAMETERS :
 *   @pData           : input frame super buffer
 * RETURN             : snapshot buf def
 *==========================================================================*/
mm_camera_buf_def_t* QCameraHALPP::getSnapshotBuf(qcamera_hal_pp_data_t* pData)
{
    LOGH("getSnapBuffer");
    return m_pHalPPMgr->getSnapshotBuf(pData);
}

/*===========================================================================
 * FUNCTION   : getMetadataBuf
 *
 * DESCRIPTION: function to get metadata buf def and the stream from frame
 * PARAMETERS :
 *   @pData     : input frame super buffer
 * RETURN     : metadata buf def
 *==========================================================================*/
mm_camera_buf_def_t* QCameraHALPP::getMetadataBuf(qcamera_hal_pp_data_t *pData)
{
    LOGH("getMetaBuf");
    return m_pHalPPMgr->getMetadataBuf(pData);
}

void QCameraHALPP::dumpYUVtoFile(const uint8_t* pBuf, const char *name, ssize_t buf_len)
{
    LOGD("E.");

    int file_fd = open(name, O_RDWR | O_CREAT, 0777);
    if (file_fd > 0) {
        fchmod(file_fd, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        ssize_t writen_bytes = 0;
        writen_bytes = write(file_fd, pBuf, buf_len);
        close(file_fd);
        LOGD("dump output frame to file: %s, size:%d", name, buf_len);
    }

    LOGD("X.");
}

} // namespace qcamera