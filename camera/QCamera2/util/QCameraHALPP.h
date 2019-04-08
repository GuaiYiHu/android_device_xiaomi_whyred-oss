/* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#ifndef __QCAMERA_HAL_PP_H__
#define __QCAMERA_HAL_PP_H__

// Camera dependencies
#include "QCamera2HWI.h"
#include "QCameraPprocManager.h"

// STL dependencies
#include <unordered_map>
#include <vector>
#include <sys/stat.h>
extern "C" {
#include "mm_camera_interface.h"
#include "mm_jpeg_interface.h"
}

typedef struct _cam_frame_size_t {
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t scanline;
    uint32_t frame_len;
    cam_frame_len_offset_t offset;
} cam_frame_size_t;

namespace qcamera {


class QCameraHALPP
{
public:
    virtual ~QCameraHALPP();
    virtual int32_t init(halPPBufNotify bufNotifyCb, halPPGetOutput getOutputCb, void *pUserData, void *pStaticParam) = 0;
    virtual int32_t init(halPPBufNotify bufNotifyCb, halPPGetOutput getOutputCb, void *pUserData);
    virtual int32_t deinit();
    virtual int32_t start();
    virtual int32_t stop();
    virtual int32_t flushQ();
    virtual int32_t initQ();
    virtual int32_t feedInput(qcamera_hal_pp_data_t *pInputData) = 0;
    virtual int32_t feedOutput(qcamera_hal_pp_data_t *pOutputData) = 0;
    virtual int32_t process() = 0;

protected:
    QCameraHALPP();
    virtual bool canProcess() = 0;
    virtual void releaseData(qcamera_hal_pp_data_t *pData);
    std::vector<qcamera_hal_pp_data_t*>* getFrameVector(uint32_t frameIndex);
    static void releaseInputDataCb(void *pData, void *pUserData);
    static void releaseOngoingDataCb(void *pData, void *pUserData);
    void dumpYUVtoFile(const uint8_t* pBuf, const char *name, ssize_t buf_len);
    int32_t getOutputBuffer(
            qcamera_hal_pp_data_t *pInputData,
            qcamera_hal_pp_data_t *pOutputData);

    mm_camera_buf_def_t* getSnapshotBuf(qcamera_hal_pp_data_t* pData);
    mm_camera_buf_def_t* getMetadataBuf(qcamera_hal_pp_data_t* pData);

protected:
    QCameraQueue m_inputQ;
    QCameraQueue m_outgoingQ;

    // hash map with frame index as key, and vecotr of input frames as value
    std::unordered_map<uint32_t, std::vector<qcamera_hal_pp_data_t*>*> m_frameMap;

    halPPBufNotify m_halPPBufNotifyCB;
    halPPGetOutput m_halPPGetOutputCB;
    QCameraHALPPManager *m_pHalPPMgr;
}; // QCameraHALPP class
}; // namespace qcamera

#endif /* __QCAMERA_HAL_PP_H__ */



