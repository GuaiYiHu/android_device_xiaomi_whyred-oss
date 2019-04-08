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

#ifndef __QCAMERAFOVCONTROL_H__
#define __QCAMERAFOVCONTROL_H__

#include <utils/Mutex.h>
#include "cam_intf.h"
#include "QCameraExtZoomTranslator.h"

using namespace android;

namespace qcamera {

typedef enum {
    AE_SETTLED,
    AE_CONVERGING
} ae_status;

typedef enum {
    AF_VALID,
    AF_INVALID
} af_status;

typedef enum {
    CAM_POSITION_LEFT,
    CAM_POSITION_RIGHT
} cam_relative_position;

typedef enum {
    STATE_WIDE,
    STATE_TRANSITION,
    STATE_TELE
} dual_cam_state;

typedef enum {
    ZOOM_STABLE,
    ZOOM_IN,
    ZOOM_OUT
} dual_cam_zoom_dir;

typedef enum {
    FOVCONTROL_FLAG_FORCE_CAMERA_WAKEUP = 0,
    FOVCONTROL_FLAG_THERMAL_THROTTLE,
    FOVCONTROL_FLAG_UPDATE_RESULT_STATE,
    FOVCONTROL_FLAG_COUNT
} fov_control_flag;

typedef enum {
    DUAL_CAM_WIDE_TELE,
    DUAL_CAM_BAYER_MONO,
    DUAL_CAM_MAX
} dual_cam_type;

typedef struct {
    ae_status status;
    float     luxIndex;
} ae_info;

typedef struct {
    af_status status;
    uint16_t  focusDistCm;
} af_info;

typedef struct {
    ae_info ae;
    af_info af;
} status_3A_t;

typedef struct {
    status_3A_t main;
    status_3A_t aux;
} dual_cam_3A_status_t;

typedef struct {
    int32_t shiftHorz;
    int32_t shiftVert;
} spatial_align_shift_t;

typedef struct {
    uint8_t               readyStatus;
    uint8_t               camMasterHint;
    uint8_t               camMasterPreview;
    uint8_t               camMaster3A;
    uint8_t               fallbackComplete;
    uint32_t              activeCameras;
    spatial_align_shift_t shiftWide;
    spatial_align_shift_t shiftTele;
    spatial_align_shift_t shiftAfRoiWide;
    spatial_align_shift_t shiftAfRoiTele;
} spatial_align_result_t;

typedef struct {
    float    cropRatio;
    float    cutOverFactor;
    float    cutOverWideToTele;
    float    cutOverTeleToWide;
    float    transitionHigh;
    float    transitionLow;
    uint32_t waitTimeForHandoffMs;
} dual_cam_transition_params_t;

typedef struct {
    bool    active;
    nsecs_t timeout;
} timer_t;

typedef struct {
    bool                         configCompleted;
    uint32_t                     zoomUser;
    uint32_t                     zoomUserPrev;
    uint32_t                     zoomWide;
    uint32_t                     zoomTele;
    uint32_t                     zoomRatioWide;
    uint32_t                     zoomRatioTele;
    uint32_t                     zoomWideIsp;
    uint32_t                     zoomTeleIsp;
    uint32_t                    *zoomRatioTable;
    uint32_t                     zoomRatioTableCount;
    dual_cam_zoom_dir            zoomDirection;
    zoom_trans_init_data         zoomTransInitData;
    cam_sync_type_t              camWide;
    cam_sync_type_t              camTele;
    dual_cam_state               camState;
    dual_cam_3A_status_t         status3A;
    cam_dimension_t              previewSize;
    cam_dimension_t              ispOutSize;
    spatial_align_result_t       spatialAlignResult;
    uint32_t                     availableSpatialAlignSolns;
    float                        camMainWidthMargin;
    float                        camMainHeightMargin;
    float                        camAuxWidthMargin;
    float                        camAuxHeightMargin;
    bool                         camcorderMode;
    bool                         wideCamStreaming;
    bool                         teleCamStreaming;
    bool                         fallbackEnabled;
    bool                         fallbackToWide;
    bool                         fallbackInitedInTransition;
    timer_t                      timerLowLitMacroScene;
    timer_t                      timerWellLitNonMacroScene;
    timer_t                      timerConstZoom;
    float                        basicFovRatio;
    dual_cam_transition_params_t transitionParams;
    uint32_t                     afStatusMain;
    uint32_t                     afStatusAux;
    bool                         forceCameraWakeup;
    bool                         thermalThrottle;
    bool                         lpmEnabled;
    bool                         updateResultState;
    uint8_t                      oisSetting;
    cam_stream_size_info_t       camStreamInfo;
    uint32_t                     frameCountWide;
    uint32_t                     frameCountTele;
} fov_control_data_t;

typedef struct {
    bool zoom_valid;
    int32_t zoom_value;
} fov_control_parm_t;

typedef struct {
    bool     enablePostProcess;
    float    zoomMin;
    float    zoomMax;
    uint16_t LuxIdxMax;
    uint16_t focusDistanceMin;
} snapshot_pp_config_t;

typedef struct {
    float    percentMarginHysterisis;
    float    percentMarginAux;
    float    percentMarginMain;
    uint16_t auxSwitchLuxIdxMax;
    uint16_t auxSwitchFocusDistCmMin;
    snapshot_pp_config_t snapshotPPConfig;
    uint32_t fallbackTimeout;
    uint32_t constZoomTimeout;
    uint32_t constZoomTimeoutSnapshotPPRange;
} fov_control_config_t;

typedef struct{
    uint32_t sensorStreamWidth;
    uint32_t sensorStreamHeight;
    float    focalLengthMm;
    float    pixelPitchUm;
    bool     isAFSupported;
} intrinsic_cam_params_t;

typedef struct {
    intrinsic_cam_params_t paramsMain;
    intrinsic_cam_params_t paramsAux;
} dual_cam_params_t;

typedef struct {
    bool            isValid;
    cam_sync_type_t camMasterPreview;
    cam_sync_type_t camMaster3A;
    uint32_t        activeCameras;
    bool            snapshotPostProcess;
    bool            snapshotPostProcessZoomRange;
    cam_ois_mode_t  oisMode;
    cam_fallback_mode_t fallback;
} fov_control_result_t;


class QCameraFOVControl {
public:
    ~QCameraFOVControl();
    static QCameraFOVControl* create(cam_capability_t *capsMainCam,
            cam_capability_t* capsAuxCam, uint8_t isHAL3 = false);
    int32_t updateConfigSettings(parm_buffer_t* paramsMainCam, parm_buffer_t* paramsAuxCam);
    cam_capability_t consolidateCapabilities(cam_capability_t* capsMainCam,
            cam_capability_t* capsAuxCam);
    int32_t translateInputParams(parm_buffer_t* paramsMainCam, parm_buffer_t *paramsAuxCam);
    metadata_buffer_t* processResultMetadata(metadata_buffer_t* metaMainCam,
            metadata_buffer_t* metaAuxCam);
    fov_control_result_t getFovControlResult();
    cam_frame_margins_t getFrameMargins(int8_t masterCamera);
    void setHalPPType(cam_hal_pp_type_t halPPtype);
    void UpdateFlag(fov_control_flag flag, void *value);
    inline bool isBayerMono() { return (mDualCamType == DUAL_CAM_BAYER_MONO); };
    void setDualCameraConfig(uint8_t type);
    bool isMainCamFovWider();

private:
    QCameraFOVControl(uint8_t isHAL3);
    bool validateAndExtractParameters(cam_capability_t  *capsMainCam,
            cam_capability_t  *capsAuxCam);
    bool calculateBasicFovRatio();
    bool combineFovAdjustment();
    void  calculateDualCamTransitionParams();
    void convertUserZoomToWideAndTele(uint32_t zoom);
    uint32_t readjustZoomForTele(uint32_t zoomWide);
    uint32_t readjustZoomForWide(uint32_t zoomTele);
    uint32_t findZoomRatio(uint32_t zoom);
    inline uint32_t findZoomValue(uint32_t zoomRatio);
    cam_face_detection_data_t translateRoiFD(cam_face_detection_data_t faceDetectionInfo,
            cam_sync_type_t cam);
    cam_roi_info_t translateFocusAreas(cam_roi_info_t roiAfMain, cam_sync_type_t cam);
    cam_set_aec_roi_t translateMeteringAreas(cam_set_aec_roi_t roiAecMain, cam_sync_type_t cam);
    void generateFovControlResult();
    bool isSpatialAlignmentReady();
    void resetVars();
    bool isMaster(cam_sync_type_t cam);
    bool canSwitchMasterTo(uint32_t cam);
    bool needDualZone();
    bool isTimedOut(timer_t timer);
    void startTimer(timer_t *timer, uint32_t time);
    void inactivateTimer(timer_t *timer);
    void setZoomParam(uint8_t cam_type, cam_zoom_info_t zoomInfo, uint32_t zoomTotal,
            uint32_t zoomIsp, bool snapshotPostProcess, parm_buffer_t* params, bool isHAL3);
    void setCropParam(uint8_t cam_type, uint32_t zoomStep, parm_buffer_t* params);
    cam_area_t translateRoi(cam_area_t roiMain, cam_sync_type_t cam);
    cam_face_detection_data_t translateHAL3FDRoi(
        cam_face_detection_data_t metaFD, cam_sync_type_t cam);

    Mutex                           mMutex;
    fov_control_config_t            mFovControlConfig;
    fov_control_data_t              mFovControlData;
    fov_control_result_t            mFovControlResult;
    fov_control_result_t            mFovControlResultCachedCopy;
    dual_cam_params_t               mDualCamParams;
    QCameraExtZoomTranslator       *mZoomTranslator;
    cam_hal_pp_type_t               mHalPPType;
    fov_control_parm_t              mFovControlParm;
    uint8_t                         mDualCamType;
    uint8_t                         mbIsHAL3;
};

}; // namespace qcamera

#endif /* __QCAMERAFOVCONTROL_H__ */
