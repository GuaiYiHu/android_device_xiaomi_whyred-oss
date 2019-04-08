/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#ifndef __QCAMERA_CLEAR_SIGHT_H__
#define __QCAMERA_CLEAR_SIGHT_H__

// Camera dependencies
#include "QCameraHALPP.h"

#define NUM_CAM 2

enum halInputType {
    BAYER_INPUT = 0,
    MONO_INPUT = 1
};

typedef struct _clearsight_input_params_t {
    cam_frame_size_t bayer;
    cam_frame_size_t mono;
    uint32_t frame_idx;
} clearsight_input_params_t;

typedef struct _clearsight_output_params_t {
    cam_frame_size_t out;
    uint32_t result;
} clearsight_output_params_t;


namespace qcamera {

class QCameraClearSight : public QCameraHALPP
{
public:
    QCameraClearSight();
    ~QCameraClearSight();
    int32_t init(halPPBufNotify bufNotifyCb, halPPGetOutput getOutputCb, void *pUserData,
            void *pStaticParam);
    int32_t deinit();
    int32_t start();
    int32_t feedInput(qcamera_hal_pp_data_t *pInputData);
    int32_t feedOutput(qcamera_hal_pp_data_t *pOutputData);
    int32_t process();
protected:
    bool canProcess();
private:
    void getInputParams(mm_camera_buf_def_t *pMainMetaBuf,
                        mm_camera_buf_def_t *pAuxMetaBuf,
                        cam_frame_len_offset_t main_offset,
                        cam_frame_len_offset_t aux_offset,
            clearsight_input_params_t& inParams);
    int32_t doClearSightInit();
    int32_t doClearSightProcess(const uint8_t* pWide, const uint8_t* pTele,
            clearsight_input_params_t inParams, uint8_t* pOut);
    void dumpYUVtoFile(const uint8_t* pBuf, cam_frame_len_offset_t offset, uint32_t idx,
            const char* name_prefix);
    void dumpInputParams(const clearsight_input_params_t& p);

private:
    void *m_dlHandle;
    const cam_capability_t *m_pCaps;
}; // QCameraClearSight class
}; // namespace qcamera

#endif /* __QCAMERA_CLEAR_SIGHT_H__ */



