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

#ifndef __QCAMERA_BOKEH_H__
#define __QCAMERA_BOKEH_H__

// Camera dependencies
#include "QCameraHALPP.h"
#include "QCameraPostProc.h"

typedef struct  {
    cam_frame_size_t main;
    cam_frame_size_t aux;
    cam_frame_size_t depth;
    cam_frame_size_t bokehOut;
    String8 sMainReprocessInfo;
    String8 sAuxReprocessInfo;
    String8 sCalibData;
    float blurLevel;
    cam_rect_t afROI;
    cam_rect_t afROIMap;
    cam_rect_t zoomROI;
} bokeh_input_params_t;

typedef struct  {
    cam_frame_size_t out;
    uint32_t result;
} bokeh_output_params_t;


namespace qcamera {

typedef struct  {
    qcamera_hal_pp_data_t* main_input;
    qcamera_hal_pp_data_t* aux_input;
    qcamera_hal_pp_data_t* bokeh_output;
    qcamera_hal_pp_data_t* depth_output;
} bokeh_data_t;

class QCameraBokeh : public QCameraHALPP
{
public:
    QCameraBokeh();
    ~QCameraBokeh();
    int32_t init(
            halPPBufNotify bufNotifyCb,
            halPPGetOutput getOutputCb,
            void *pUserData,
            void *pStaticParam);
    int32_t deinit();
    int32_t start();
    int32_t stop();
    int32_t feedInput(qcamera_hal_pp_data_t *pInputData);
    int32_t feedOutput(qcamera_hal_pp_data_t *pOutputData);
    int32_t process();
protected:
    bool canProcess();
private:
    void getInputParams(bokeh_input_params_t& inParams);
    int32_t doBokehInit();
    int32_t doBokehProcess(
            const uint8_t* pMain,
            const uint8_t* pAux,
            bokeh_input_params_t &inParams,
            uint8_t* pOut);
    void dumpYUVtoFile(
            const uint8_t* pBuf,
            cam_frame_len_offset_t offset,
            uint32_t idx,
            const char* name_prefix);
    String8 buildCommaSeparatedString(float array[], size_t length);
    String8 flattenCropInfo(cam_stream_crop_info_t* crop, uint8_t index);
    String8 extractReprocessInfo(metadata_buffer_t *metadata);
    String8 extractCalibrationData();
    void dumpInputParams(const char* filename, String8 str, uint32_t idx);
    int32_t allocateDepthBuf(cam_frame_size_t depthSize);

private:
    void *m_dlHandle;
    const cam_capability_t *m_pCaps;
    bokeh_data_t mBokehData;
    bool bNeedCamSwap;
    String8 mDebugData;
    bool m_bDebug;
}; // QCameraBokeh class
}; // namespace qcamera

#endif /* __QCAMERA_BOKEH_H__ */

