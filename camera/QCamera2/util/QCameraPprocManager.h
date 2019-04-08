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

#ifndef __QCAMERA_HAL_PPROC_MANAGER_H__
#define __QCAMERA_HAL_PPROC_MANAGER_H__

// Camera dependencies
#include "QCameraQueue.h"
#include "QCamera3HALHeader.h"
#include "QCameraCmdThread.h"
#include <utils/Mutex.h>

extern "C" {
#include "mm_camera_interface.h"
}

using namespace android;

namespace qcamera {

class QCameraHeapMemory;
typedef struct {
    mm_camera_super_buf_t *frame; // source frame
    mm_camera_buf_def_t   *bufs;  // source buf_defs
    bool is_offset_valid;
    cam_frame_len_offset_t snap_offset;
    cam_frame_len_offset_t meta_offset;

    //out params
    bool is_crop_valid;
    cam_rect_t outputCrop;
    bool is_dim_valid;
    cam_dimension_t outputDim;
    bool is_format_valid;
    cam_format_t outputFormat;

    uint32_t frameIndex;          // source frame index
    bool halPPAllocatedBuf;       // true if src frame buffer is allocated by HAL PP block
    QCameraHeapMemory    *snapshot_heap;    // output image heap buffer
    QCameraHeapMemory    *metadata_heap;    // metadata heap buffer

    /* buffer in qcamera_pp_data_t need to be release when done */
    bool reproc_frame_release;       // false release original buffer
                                     // true don't release it
    mm_camera_buf_def_t *src_reproc_bufs;
    mm_camera_super_buf_t *src_reproc_frame;// source frame (need to be
                                            //returned back to kernel after done)
    mm_camera_super_buf_t *src_frame;

    uint8_t offline_buffer;
    mm_camera_buf_def_t *offline_reproc_buf; //HAL processed buffer
    bool needEncode;

    //HAL3 specific
    metadata_buffer_t *metadata;
    jpeg_settings_t *jpeg_settings;
    jpeg_settings_t *output_jpeg_settings;
    mm_camera_super_buf_t *src_metadata;
    void* pUserData;
} qcamera_hal_pp_data_t;

/** halPPBufNotify: function definition for frame notify
*   handling
*    @pOutput  : received qcamera_hal_pp_data_t data
*    @pUserData: user data pointer
**/
typedef void (*halPPBufNotify) (qcamera_hal_pp_data_t *pOutput,
                                        void *pUserData);
/** halPPGetOutput: function definition for get output buffer
*    @frameIndex: output frame index should match input frame index
*    @pUserData: user data pointer
**/
typedef void (*halPPGetOutput) (uint32_t frameIndex, void *pUserData);

typedef void (*halPPReleaseSuperbuf) (mm_camera_super_buf_t *super_buf, void* pUserData);

class QCameraHALPP;

class QCameraHALPPManager
{
public:
    static QCameraHALPPManager* getInstance();
    void release();
    int32_t init(cam_hal_pp_type_t type, halPPBufNotify bufNotifyCb,
                          halPPReleaseSuperbuf releaseBufCb, void *pStaticParam);
    int32_t deinit();
    int32_t start();
    int32_t stop();
    int32_t feedInput(qcamera_hal_pp_data_t *pInput);
    int32_t feedOutput(qcamera_hal_pp_data_t *pOutputData);
    void releaseData(qcamera_hal_pp_data_t *pData);
    mm_camera_buf_def_t* getSnapshotBuf(qcamera_hal_pp_data_t* pData);
    mm_camera_buf_def_t* getMetadataBuf(qcamera_hal_pp_data_t* pData);
    cam_hal_pp_type_t getPprocType() { return m_pprocType;};

private:
    QCameraHALPPManager();
    ~QCameraHALPPManager();
    static void *dataProcessRoutine(void *pData);
    void getHalPPOutputBuffer(uint32_t frameIndex);

protected:
    static void releaseDataCb(void *pData, void *pUserData);
    static void processHalPPDataCB(qcamera_hal_pp_data_t *pOutput, void* pUserData);
    static void getHalPPOutputBufferCB(uint32_t frameIndex, void* pUserData);

protected:
    static QCameraHALPPManager* s_pInstance;
    QCameraQueue m_inputQ;
    QCameraHALPP *m_pPprocModule;
    cam_hal_pp_type_t m_pprocType;
    QCameraCmdThread m_pprocTh;      // thread for data processing
    bool m_bInited;
    bool m_bStarted;

    halPPBufNotify m_halPPBufNotifyCB;
    halPPReleaseSuperbuf m_halPPReleaseBufCB;
    Mutex mLock;
}; // QCameraHALPPManager class
}; // namespace qcamera

#endif /* __QCAMERA_HAL_PPROC_MANAGER_H__ */


