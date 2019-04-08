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

#define LOG_TAG "QCameraFOVControl"

#include <stdlib.h>
#include <cutils/properties.h>
#include <utils/Timers.h>
#include <utils/Errors.h>
#include "QCameraFOVControl.h"
#include "QCameraDualCamSettings.h"


extern "C" {
#define CAM_MODULE CAM_HAL_MODULE
#include "mm_camera_dbg.h"
#include "mm_camera_interface.h"
}

namespace qcamera {

/*===========================================================================
 * FUNCTION   : QCameraFOVControl constructor
 *
 * DESCRIPTION: class constructor
 *
 * PARAMETERS : none
 *
 * RETURN     : void
 *
 *==========================================================================*/
QCameraFOVControl::QCameraFOVControl(uint8_t isHAL3)
    :mZoomTranslator(NULL),
     mHalPPType(CAM_HAL_PP_TYPE_NONE),
     mDualCamType(DUAL_CAM_WIDE_TELE),
     mbIsHAL3(isHAL3)
{
    memset(&mDualCamParams,    0, sizeof(dual_cam_params_t));
    memset(&mFovControlConfig, 0, sizeof(fov_control_config_t));
    memset(&mFovControlData,   0, sizeof(fov_control_data_t));
    memset(&mFovControlResult, 0, sizeof(fov_control_result_t));
    memset(&mFovControlParm,   0, sizeof(mFovControlParm));
    memset(&mFovControlResultCachedCopy, 0, sizeof(fov_control_result_t));
}


/*===========================================================================
 * FUNCTION   : QCameraFOVControl destructor
 *
 * DESCRIPTION: class destructor
 *
 * PARAMETERS : none
 *
 * RETURN     : void
 *
 *==========================================================================*/
QCameraFOVControl::~QCameraFOVControl()
{
    // De-initialize zoom translator lib
    if (mZoomTranslator && mZoomTranslator->isInitialized()) {
        mZoomTranslator->deInit();
    }
}


/*===========================================================================
 * FUNCTION   : create
 *
 * DESCRIPTION: This is a static method to create FOV-control object. It calls
 *              private constructor of the class and only returns a valid object
 *              if it can successfully initialize the FOV-control.
 *
 * PARAMETERS :
 *  @capsMain : The capabilities for the main camera
 *  @capsAux  : The capabilities for the aux camera
 *
 * RETURN     : Valid object pointer if succeeds
 *              NULL if fails
 *
 *==========================================================================*/
QCameraFOVControl* QCameraFOVControl::create(
        cam_capability_t *capsMainCam,
        cam_capability_t *capsAuxCam,
        uint8_t isHAL3)
{
    QCameraFOVControl *pFovControl  = NULL;

    if (capsMainCam && capsAuxCam) {
        // Create FOV control object
        pFovControl = new QCameraFOVControl(isHAL3);

        if (pFovControl) {
            bool  success = false;
            if (pFovControl->validateAndExtractParameters(capsMainCam, capsAuxCam)) {

                // Based on focal lengths, map main and aux camera to wide and tele
                if (pFovControl->mDualCamParams.paramsMain.focalLengthMm <
                    pFovControl->mDualCamParams.paramsAux.focalLengthMm) {
                    pFovControl->mFovControlData.camWide  = CAM_TYPE_MAIN;
                    pFovControl->mFovControlData.camTele  = CAM_TYPE_AUX;
                    pFovControl->mFovControlData.camState = STATE_WIDE;
                } else {
                    pFovControl->mFovControlData.camWide  = CAM_TYPE_AUX;
                    pFovControl->mFovControlData.camTele  = CAM_TYPE_MAIN;
                    pFovControl->mFovControlData.camState = STATE_TELE;
                }

                // Initialize the master info to main camera
                pFovControl->mFovControlResult.camMasterPreview  = CAM_TYPE_MAIN;
                pFovControl->mFovControlResult.camMaster3A       = CAM_TYPE_MAIN;

                // Check if LPM is enabled
                char prop[PROPERTY_VALUE_MAX];
                int lpmEnable = 1;
                property_get("persist.vendor.dualcam.lpm.enable", prop, "1");
                lpmEnable = atoi(prop);
                if ((lpmEnable == 0) || (DUALCAM_LPM_ENABLE == 0)) {
                    pFovControl->mFovControlData.lpmEnabled = false;
                } else {
                    pFovControl->mFovControlData.lpmEnabled = true;
                }

                // Open the external zoom translation library if requested
                if (FOVC_USE_EXTERNAL_ZOOM_TRANSLATOR) {
                    pFovControl->mZoomTranslator =
                            QCameraExtZoomTranslator::create();
                    if (!pFovControl->mZoomTranslator) {
                        LOGE("Unable to open zoom translation lib");
                    }
                }
                success = true;
            }

            if (!success) {
                LOGE("FOV-control: Failed to create an object");
                delete pFovControl;
                pFovControl = NULL;
            }
        } else {
            LOGE("FOV-control: Failed to allocate memory for FOV-control object");
        }
    }

    return pFovControl;
}


/*===========================================================================
 * FUNCTION    : consolidateCapabilities
 *
 * DESCRIPTION : Combine the capabilities from main and aux cameras to return
 *               the consolidated capabilities.
 *
 * PARAMETERS  :
 * @capsMainCam: Capabilities for the main camera
 * @capsAuxCam : Capabilities for the aux camera
 *
 * RETURN      : Consolidated capabilities
 *
 *==========================================================================*/
cam_capability_t QCameraFOVControl::consolidateCapabilities(
        cam_capability_t *capsMainCam,
        cam_capability_t *capsAuxCam)
{
    cam_capability_t capsConsolidated;
    memset(&capsConsolidated, 0, sizeof(cam_capability_t));

    if ((capsMainCam != NULL) &&
        (capsAuxCam  != NULL)) {

        memcpy(&capsConsolidated, capsMainCam, sizeof(cam_capability_t));

        // Consolidate preview sizes
        uint32_t previewSizesTblCntMain  = capsMainCam->preview_sizes_tbl_cnt;
        uint32_t previewSizesTblCntAux   = capsAuxCam->preview_sizes_tbl_cnt;
        uint32_t previewSizesTblCntFinal = 0;

        for (uint32_t i = 0; i < previewSizesTblCntMain; ++i) {
            for (uint32_t j = 0; j < previewSizesTblCntAux; ++j) {
                if ((capsMainCam->preview_sizes_tbl[i].width ==
                     capsAuxCam->preview_sizes_tbl[j].width) &&
                    (capsMainCam->preview_sizes_tbl[i].height ==
                     capsAuxCam->preview_sizes_tbl[j].height)) {
                    if (previewSizesTblCntFinal != i) {
                        capsConsolidated.preview_sizes_tbl[previewSizesTblCntFinal].width =
                           capsAuxCam->preview_sizes_tbl[j].width;
                        capsConsolidated.preview_sizes_tbl[previewSizesTblCntFinal].height =
                           capsMainCam->preview_sizes_tbl[j].height;
                    }
                    ++previewSizesTblCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.preview_sizes_tbl_cnt = previewSizesTblCntFinal;

        // Consolidate video sizes
        uint32_t videoSizesTblCntMain  = capsMainCam->video_sizes_tbl_cnt;
        uint32_t videoSizesTblCntAux   = capsAuxCam->video_sizes_tbl_cnt;
        uint32_t videoSizesTblCntFinal = 0;

        for (uint32_t i = 0; i < videoSizesTblCntMain; ++i) {
            for (uint32_t j = 0; j < videoSizesTblCntAux; ++j) {
                if ((capsMainCam->video_sizes_tbl[i].width ==
                     capsAuxCam->video_sizes_tbl[j].width) &&
                    (capsMainCam->video_sizes_tbl[i].height ==
                     capsAuxCam->video_sizes_tbl[j].height)) {
                    if (videoSizesTblCntFinal != i) {
                        capsConsolidated.video_sizes_tbl[videoSizesTblCntFinal].width =
                           capsAuxCam->video_sizes_tbl[j].width;
                        capsConsolidated.video_sizes_tbl[videoSizesTblCntFinal].height =
                           capsMainCam->video_sizes_tbl[j].height;
                    }
                    ++videoSizesTblCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.video_sizes_tbl_cnt = videoSizesTblCntFinal;

        // Consolidate livesnapshot sizes
        uint32_t livesnapshotSizesTblCntMain  = capsMainCam->livesnapshot_sizes_tbl_cnt;
        uint32_t livesnapshotSizesTblCntAux   = capsAuxCam->livesnapshot_sizes_tbl_cnt;
        uint32_t livesnapshotSizesTblCntFinal = 0;

        for (uint32_t i = 0; i < livesnapshotSizesTblCntMain; ++i) {
            for (uint32_t j = 0; j < livesnapshotSizesTblCntAux; ++j) {
                if ((capsMainCam->livesnapshot_sizes_tbl[i].width ==
                     capsAuxCam->livesnapshot_sizes_tbl[j].width) &&
                    (capsMainCam->livesnapshot_sizes_tbl[i].height ==
                     capsAuxCam->livesnapshot_sizes_tbl[j].height)) {
                    if (livesnapshotSizesTblCntFinal != i) {
                       capsConsolidated.livesnapshot_sizes_tbl[livesnapshotSizesTblCntFinal].width=
                          capsAuxCam->livesnapshot_sizes_tbl[j].width;
                       capsConsolidated.livesnapshot_sizes_tbl[livesnapshotSizesTblCntFinal].height=
                          capsMainCam->livesnapshot_sizes_tbl[j].height;
                    }
                    ++livesnapshotSizesTblCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.livesnapshot_sizes_tbl_cnt = livesnapshotSizesTblCntFinal;

        // Consolidate picture size
        // Find max picture dimension for main camera
        cam_dimension_t maxPicDimMain;
        maxPicDimMain.width  = 0;
        maxPicDimMain.height = 0;

        for(uint32_t i = 0; i < (capsMainCam->picture_sizes_tbl_cnt - 1); ++i) {
            if ((maxPicDimMain.width * maxPicDimMain.height) <
                    (capsMainCam->picture_sizes_tbl[i].width *
                            capsMainCam->picture_sizes_tbl[i].height)) {
                maxPicDimMain.width  = capsMainCam->picture_sizes_tbl[i].width;
                maxPicDimMain.height = capsMainCam->picture_sizes_tbl[i].height;
            }
        }

        // Find max picture dimension for aux camera
        cam_dimension_t maxPicDimAux;
        maxPicDimAux.width  = 0;
        maxPicDimAux.height = 0;

        for(uint32_t i = 0; i < (capsAuxCam->picture_sizes_tbl_cnt - 1); ++i) {
            if ((maxPicDimAux.width * maxPicDimAux.height) <
                    (capsAuxCam->picture_sizes_tbl[i].width *
                            capsAuxCam->picture_sizes_tbl[i].height)) {
                maxPicDimAux.width  = capsAuxCam->picture_sizes_tbl[i].width;
                maxPicDimAux.height = capsAuxCam->picture_sizes_tbl[i].height;
            }
        }

        LOGH("MAIN Max picture wxh %dx%d", maxPicDimMain.width, maxPicDimMain.height);
        LOGH("AUX Max picture wxh %dx%d", maxPicDimAux.width, maxPicDimAux.height);

        // Choose the larger of the two max picture dimensions
        if ((maxPicDimAux.width * maxPicDimAux.height) >
                (maxPicDimMain.width * maxPicDimMain.height)) {
            capsConsolidated.picture_sizes_tbl_cnt = capsAuxCam->picture_sizes_tbl_cnt;
            memcpy(capsConsolidated.picture_sizes_tbl, capsAuxCam->picture_sizes_tbl,
                    (capsAuxCam->picture_sizes_tbl_cnt * sizeof(cam_dimension_t)));
        }
        LOGH("Consolidated Max picture wxh %dx%d", capsConsolidated.picture_sizes_tbl[0].width,
                capsConsolidated.picture_sizes_tbl[0].height);

        // Consolidate supported preview formats
        uint32_t supportedPreviewFmtCntMain  = capsMainCam->supported_preview_fmt_cnt;
        uint32_t supportedPreviewFmtCntAux   = capsAuxCam->supported_preview_fmt_cnt;
        uint32_t supportedPreviewFmtCntFinal = 0;
        for (uint32_t i = 0; i < supportedPreviewFmtCntMain; ++i) {
            for (uint32_t j = 0; j < supportedPreviewFmtCntAux; ++j) {
                if (capsMainCam->supported_preview_fmts[i] ==
                        capsAuxCam->supported_preview_fmts[j]) {
                    if (supportedPreviewFmtCntFinal != i) {
                        capsConsolidated.supported_preview_fmts[supportedPreviewFmtCntFinal] =
                            capsAuxCam->supported_preview_fmts[j];
                    }
                    ++supportedPreviewFmtCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.supported_preview_fmt_cnt = supportedPreviewFmtCntFinal;

        // Consolidate supported picture formats
        uint32_t supportedPictureFmtCntMain  = capsMainCam->supported_picture_fmt_cnt;
        uint32_t supportedPictureFmtCntAux   = capsAuxCam->supported_picture_fmt_cnt;
        uint32_t supportedPictureFmtCntFinal = 0;
        for (uint32_t i = 0; i < supportedPictureFmtCntMain; ++i) {
            for (uint32_t j = 0; j < supportedPictureFmtCntAux; ++j) {
                if (capsMainCam->supported_picture_fmts[i] ==
                        capsAuxCam->supported_picture_fmts[j]) {
                    if (supportedPictureFmtCntFinal != i) {
                        capsConsolidated.supported_picture_fmts[supportedPictureFmtCntFinal] =
                            capsAuxCam->supported_picture_fmts[j];
                    }
                    ++supportedPictureFmtCntFinal;
                    break;
                }
            }
        }
        capsConsolidated.supported_picture_fmt_cnt = supportedPictureFmtCntFinal;

        //consolidate focus modes, to handle FF + AF case
        if (capsMainCam->supported_focus_modes_cnt <= 1) {
            capsConsolidated.supported_focus_modes_cnt = capsAuxCam->supported_focus_modes_cnt;
            memcpy(capsConsolidated.supported_focus_modes, capsAuxCam->supported_focus_modes,
                    (capsAuxCam->supported_focus_modes_cnt * sizeof(cam_focus_mode_type)));
        }

        if (mZoomTranslator) {
            // Copy the opaque calibration data pointer and size
            mFovControlData.zoomTransInitData.calibData =
                    capsConsolidated.related_cam_calibration.dc_otp_params;
            mFovControlData.zoomTransInitData.calibDataSize =
                    capsConsolidated.related_cam_calibration.dc_otp_size;
        }
    }
    return capsConsolidated;
}


/*===========================================================================
 * FUNCTION    : resetVars
 *
 * DESCRIPTION : Reset the variables used in FOV-control.
 *
 * PARAMETERS  : None
 *
 * RETURN      : None
 *
 *==========================================================================*/
void QCameraFOVControl::resetVars()
{
    // Copy the FOV-control settings for camera/camcorder from QCameraFOVControlSettings.h
    if (mFovControlData.camcorderMode) {
        // Disable snapshot post-processing if two cameras have no sync mechanism.
        mFovControlConfig.snapshotPPConfig.enablePostProcess =
                (DUALCAM_SYNC_MECHANISM == CAM_SYNC_NO_SYNC) ?
                    0 : FOVC_CAMCORDER_SNAPSHOT_PP_ENABLE;
    } else {
        // Disable snapshot post-processing if two cameras have no sync mechanism.
        mFovControlConfig.snapshotPPConfig.enablePostProcess =
                (DUALCAM_SYNC_MECHANISM == CAM_SYNC_NO_SYNC) ? 0 : FOVC_CAM_SNAPSHOT_PP_ENABLE;
        mFovControlConfig.snapshotPPConfig.zoomMin           = FOVC_CAM_SNAPSHOT_PP_ZOOM_MIN;
        mFovControlConfig.snapshotPPConfig.zoomMax           = FOVC_CAM_SNAPSHOT_PP_ZOOM_MAX;
        mFovControlConfig.snapshotPPConfig.LuxIdxMax         = FOVC_CAM_SNAPSHOT_PP_LUX_IDX_MAX;
        mFovControlConfig.snapshotPPConfig.focusDistanceMin  =
                FOVC_CAM_SNAPSHOT_PP_FOCUS_DIST_CM_MIN;
    }
    mFovControlConfig.auxSwitchLuxIdxMax      = FOVC_AUXCAM_SWITCH_LUX_IDX_MAX;
    mFovControlConfig.auxSwitchFocusDistCmMin = FOVC_AUXCAM_SWITCH_FOCUS_DIST_CM_MIN;

    mFovControlData.fallbackEnabled = FOVC_MAIN_CAM_FALLBACK_MECHANISM;

    mFovControlConfig.fallbackTimeout  = FOVC_LOWLIGHT_MACROSCENE_FALLBACK_TIMEOUT_MS;
    mFovControlConfig.constZoomTimeout = FOVC_CONSTZOOM_TIMEOUT_MS;
    mFovControlConfig.constZoomTimeoutSnapshotPPRange = FOVC_CONSTZOOM_SNAPSHOT_PP_RANGE_TIMEOUT_MS;

    // Reset variables
    mFovControlData.zoomDirection  = ZOOM_STABLE;
    mFovControlData.fallbackToWide = false;

    mFovControlData.afStatusMain = CAM_AF_STATE_INACTIVE;
    mFovControlData.afStatusAux  = CAM_AF_STATE_INACTIVE;

    mFovControlData.wideCamStreaming = false;
    mFovControlData.teleCamStreaming = false;
    mFovControlData.frameCountWide = 0;
    mFovControlData.frameCountTele = 0;

    mFovControlData.spatialAlignResult.readyStatus = 0;
    mFovControlData.spatialAlignResult.activeCameras = 0;
    mFovControlData.spatialAlignResult.camMasterHint = 0;
    mFovControlData.spatialAlignResult.shiftWide.shiftHorz = 0;
    mFovControlData.spatialAlignResult.shiftWide.shiftVert = 0;
    mFovControlData.spatialAlignResult.shiftTele.shiftHorz = 0;
    mFovControlData.spatialAlignResult.shiftTele.shiftVert = 0;

    mFovControlData.updateResultState = true;

    // WA for now until the QTI solution is in place writing the spatial alignment ready status
    mFovControlData.spatialAlignResult.readyStatus = 1;

    // Inactivate the timers
    inactivateTimer(&mFovControlData.timerLowLitMacroScene);
    inactivateTimer(&mFovControlData.timerWellLitNonMacroScene);
    inactivateTimer(&mFovControlData.timerConstZoom);
 }

/*===========================================================================
 * FUNCTION    : updateConfigSettings
 *
 * DESCRIPTION : Update the config settings such as margins and preview size
 *               and recalculate the transition parameters.
 *
 * PARAMETERS  :
 * @capsMainCam: Capabilities for the main camera
 * @capsAuxCam : Capabilities for the aux camera
 *
 * RETURN :
 * NO_ERROR           : Success
 * INVALID_OPERATION  : Failure
 *
 *==========================================================================*/
int32_t QCameraFOVControl::updateConfigSettings(
        parm_buffer_t* paramsMainCam,
        parm_buffer_t* paramsAuxCam)
{
    int32_t rc = INVALID_OPERATION;

    if (paramsMainCam &&
        paramsAuxCam  &&
        paramsMainCam->is_valid[CAM_INTF_META_STREAM_INFO] &&
        paramsAuxCam->is_valid[CAM_INTF_META_STREAM_INFO]) {

        cam_stream_size_info_t camMainStreamInfo;
        READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_META_STREAM_INFO, camMainStreamInfo);
        mFovControlData.camcorderMode = false;

        mFovControlData.camStreamInfo = camMainStreamInfo;

        // Identify if in camera or camcorder mode
        for (int i = 0; i < MAX_NUM_STREAMS; ++i) {
            if (camMainStreamInfo.type[i] == CAM_STREAM_TYPE_VIDEO) {
                mFovControlData.camcorderMode = true;
            }
        }

        // Get the margins for the main camera. If video stream is present, the margins correspond
        // to video stream. Otherwise, margins are copied from preview stream.
        for (int i = 0; i < MAX_NUM_STREAMS; ++i) {
            if (camMainStreamInfo.type[i] == CAM_STREAM_TYPE_VIDEO) {
                mFovControlData.camMainWidthMargin  = camMainStreamInfo.margins[i].widthMargins;
                mFovControlData.camMainHeightMargin = camMainStreamInfo.margins[i].heightMargins;
            }
            if (camMainStreamInfo.type[i] == CAM_STREAM_TYPE_PREVIEW) {
                // Update the preview dimension and ISP output size
                mFovControlData.previewSize = camMainStreamInfo.stream_sizes[i];
                mFovControlData.ispOutSize  = camMainStreamInfo.stream_sz_plus_margin[i];
                if (!mFovControlData.camcorderMode) {
                    mFovControlData.camMainWidthMargin  =
                            camMainStreamInfo.margins[i].widthMargins;
                    mFovControlData.camMainHeightMargin =
                            camMainStreamInfo.margins[i].heightMargins;
                    break;
                }
            }
        }

        // Get the margins for the aux camera. If video stream is present, the margins correspond
        // to the video stream. Otherwise, margins are copied from preview stream.
        cam_stream_size_info_t camAuxStreamInfo;
        READ_PARAM_ENTRY(paramsAuxCam, CAM_INTF_META_STREAM_INFO, camAuxStreamInfo);
        for (int i = 0; i < MAX_NUM_STREAMS; ++i) {
            if (camAuxStreamInfo.type[i] == CAM_STREAM_TYPE_VIDEO) {
                mFovControlData.camAuxWidthMargin  = camAuxStreamInfo.margins[i].widthMargins;
                mFovControlData.camAuxHeightMargin = camAuxStreamInfo.margins[i].heightMargins;
            }
            if (camAuxStreamInfo.type[i] == CAM_STREAM_TYPE_PREVIEW) {
                // Update the preview dimension
                mFovControlData.previewSize = camAuxStreamInfo.stream_sizes[i];
                if (!mFovControlData.camcorderMode) {
                    mFovControlData.camAuxWidthMargin  = camAuxStreamInfo.margins[i].widthMargins;
                    mFovControlData.camAuxHeightMargin = camAuxStreamInfo.margins[i].heightMargins;
                    break;
                }
            }
        }

        // Get the sensor out dimensions
        cam_dimension_t sensorDimMain = {0,0};
        cam_dimension_t sensorDimAux  = {0,0};
        if (paramsMainCam->is_valid[CAM_INTF_PARM_RAW_DIMENSION]) {
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_PARM_RAW_DIMENSION, sensorDimMain);
            mDualCamParams.paramsMain.sensorStreamWidth  = sensorDimMain.width;
            mDualCamParams.paramsMain.sensorStreamHeight = sensorDimMain.height;
        }
        if (paramsAuxCam->is_valid[CAM_INTF_PARM_RAW_DIMENSION]) {
            READ_PARAM_ENTRY(paramsAuxCam, CAM_INTF_PARM_RAW_DIMENSION, sensorDimAux);
            mDualCamParams.paramsAux.sensorStreamWidth  = sensorDimAux.width;
            mDualCamParams.paramsAux.sensorStreamHeight = sensorDimAux.height;
        }

        // Reset the internal variables
        resetVars();

        if (isBayerMono()) {
            // Bayer is primary camera
            mFovControlResult.isValid = true;
            mFovControlResult.camMasterPreview  = CAM_TYPE_MAIN;
            mFovControlResult.camMaster3A  = CAM_TYPE_MAIN;
            mFovControlResult.activeCameras = MM_CAMERA_DUAL_CAM;
            mFovControlData.configCompleted = true;
            mFovControlResult.snapshotPostProcess = true;
            memcpy(&mFovControlResultCachedCopy, &mFovControlResult,
                    sizeof(fov_control_result_t));
            LOGH("Configure FOVC for Bayer+Mono");
            return NO_ERROR;
        }

        // Recalculate the transition parameters
        if (calculateBasicFovRatio() && combineFovAdjustment()) {

            calculateDualCamTransitionParams();

            if (mHalPPType == CAM_HAL_PP_TYPE_BOKEH) {
                // Tele is primary camera
                mFovControlResult.isValid = true;
                mFovControlResult.camMasterPreview = mFovControlData.camTele;
                mFovControlResult.camMaster3A = mFovControlData.camTele;
                mFovControlResult.activeCameras =
                        (uint32_t)(mFovControlData.camTele | mFovControlData.camWide);
                mFovControlResult.snapshotPostProcess = true;
                mFovControlResult.oisMode = OIS_MODE_HOLD;
                mFovControlData.configCompleted = true;
                memcpy(&mFovControlResultCachedCopy, &mFovControlResult,
                        sizeof(fov_control_result_t));
                LOGH("Bokeh : start camera state for Bokeh Mode: TELE");
                return NO_ERROR;
            }

            // Initialize the result OIS mode as HOLD
            mFovControlResult.oisMode = OIS_MODE_HOLD;

            // Update OIS setting / scheme
            if (mFovControlData.camcorderMode) {
                mFovControlData.oisSetting = DUALCAM_OIS_MODE_CAMCORDER;
            } else {
                mFovControlData.oisSetting = DUALCAM_OIS_MODE_CAM;
            }

            // Set initial camera state
            float zoom = findZoomRatio(mFovControlData.zoomUser) /
                    (float)mFovControlData.zoomRatioTable[0];
            if (zoom > mFovControlData.transitionParams.cutOverWideToTele) {
                mFovControlResult.camMasterPreview  = mFovControlData.camTele;
                mFovControlResult.camMaster3A       = mFovControlData.camTele;
                mFovControlResult.activeCameras     = (uint32_t)mFovControlData.camTele;
                mFovControlData.camState            = STATE_TELE;
                LOGH("start camera state: TELE");
            } else {
                mFovControlResult.camMasterPreview  = mFovControlData.camWide;
                mFovControlResult.camMaster3A       = mFovControlData.camWide;
                mFovControlResult.activeCameras     = (uint32_t)mFovControlData.camWide;
                mFovControlData.camState            = STATE_WIDE;
                LOGH("start camera state: WIDE");
            }
            mFovControlResult.snapshotPostProcess = false;

            // Deinit zoom translation lib if needed
            if (mZoomTranslator && mZoomTranslator->isInitialized()) {
                if (mZoomTranslator->deInit() != NO_ERROR) {
                    ALOGW("deinit failed for zoom translation lib");
                }
            }

            // Initialize the zoom translation lib
            if (mZoomTranslator) {
                // Set the initialization data
                mFovControlData.zoomTransInitData.previewDimension.width =
                        mFovControlData.previewSize.width;
                mFovControlData.zoomTransInitData.previewDimension.height =
                        mFovControlData.previewSize.height;
                mFovControlData.zoomTransInitData.ispOutDimension.width =
                        mFovControlData.ispOutSize.width;
                mFovControlData.zoomTransInitData.ispOutDimension.height =
                        mFovControlData.ispOutSize.height;
                mFovControlData.zoomTransInitData.sensorOutDimensionMain.width =
                        sensorDimMain.width;
                mFovControlData.zoomTransInitData.sensorOutDimensionMain.height =
                        sensorDimMain.height;
                mFovControlData.zoomTransInitData.sensorOutDimensionAux.width =
                        sensorDimAux.width;
                mFovControlData.zoomTransInitData.sensorOutDimensionAux.height =
                        sensorDimAux.height;
                mFovControlData.zoomTransInitData.zoomRatioTable =
                        mFovControlData.zoomRatioTable;
                mFovControlData.zoomTransInitData.zoomRatioTableCount =
                        mFovControlData.zoomRatioTableCount;
                mFovControlData.zoomTransInitData.mode = mFovControlData.camcorderMode ?
                        MODE_CAMCORDER : MODE_CAMERA;

                if(mZoomTranslator->init(mFovControlData.zoomTransInitData) != NO_ERROR) {
                    LOGE("init failed for zoom translation lib");

                    // deinitialize the zoom translator and set to NULL
                    mZoomTranslator->deInit();
                    mZoomTranslator = NULL;
                }
            }

            // FOV-control config is complete for the current use case
            mFovControlData.configCompleted = true;
            rc = NO_ERROR;

            memcpy(&mFovControlResultCachedCopy, &mFovControlResult, sizeof(fov_control_result_t));

            rc = translateInputParams(paramsMainCam, paramsAuxCam);
            if (rc != NO_ERROR) {
                LOGE("FOV-control: Failed to translate");
                return rc;
            }
        }
    }

    return rc;
}


/*===========================================================================
 * FUNCTION   : translateInputParams
 *
 * DESCRIPTION: Translate a subset of input parameters from main camera. As main
 *              and aux cameras have different properties/params, this translation
 *              is needed before the input parameters are sent to the aux camera.
 *
 * PARAMETERS :
 * @paramsMainCam : Input parameters for main camera
 * @paramsAuxCam  : Input parameters for aux camera
 *
 * RETURN :
 * NO_ERROR           : Success
 * INVALID_OPERATION  : Failure
 *
 *==========================================================================*/
int32_t QCameraFOVControl::translateInputParams(
        parm_buffer_t* paramsMainCam,
        parm_buffer_t* paramsAuxCam)
{
    int32_t rc = INVALID_OPERATION;
    if (paramsMainCam && paramsAuxCam) {
        bool useCropRegion = false;
        cam_crop_region_t scalerCropRegion;
        // First copy all the parameters from main to aux and then translate the subset
        memcpy(paramsAuxCam, paramsMainCam, sizeof(parm_buffer_t));

        if (isBayerMono()) {
            //skip translation for B+M
            return NO_ERROR;
        }

        //Translate zoom in HAL3
        if (paramsMainCam->is_valid[CAM_INTF_META_SCALER_CROP_REGION]) {
            READ_PARAM_ENTRY(paramsMainCam,
                    CAM_INTF_META_SCALER_CROP_REGION, scalerCropRegion);
            float ratio = (float)mDualCamParams.paramsMain.sensorStreamWidth / scalerCropRegion.width;
            uint32_t userZoomRatio = ratio * 100;
            uint32_t userZoom = findZoomValue(userZoomRatio);
            mFovControlParm.zoom_valid = 1;
            mFovControlParm.zoom_value = userZoom;
            useCropRegion = true;
            LOGD("user zoomRatio: %f", ratio);
        }

        // Translate zoom in HAL1
        if (paramsMainCam->is_valid[CAM_INTF_PARM_USERZOOM]) {
            //Cache user zoom
            cam_zoom_info_t zoomInfo;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_PARM_USERZOOM, zoomInfo);
            mFovControlParm.zoom_valid = 1;
            mFovControlParm.zoom_value = zoomInfo.user_zoom;
        }

        // Translate zoom
        if (mFovControlParm.zoom_valid) {
            uint32_t userZoom = mFovControlParm.zoom_value;
            convertUserZoomToWideAndTele(userZoom);

            // Update zoom values in the param buffers
            cam_zoom_info_t zoomInfo;
            memset(&zoomInfo, 0, sizeof(cam_zoom_info_t));
            zoomInfo.num_streams = mFovControlData.camStreamInfo.num_streams;
            if (zoomInfo.num_streams) {
                zoomInfo.is_stream_zoom_info_valid = 1;
            }
            zoomInfo.user_zoom = userZoom;
            LOGD("user zoom: %d", userZoom);

            // Read the snapshot postprocess flag
            mMutex.lock();
            bool snapshotPostProcess = mFovControlResult.snapshotPostProcess;
            mMutex.unlock();

            // Update zoom value for main camera param buffer
            uint32_t zoomTotal = isMainCamFovWider() ?
                    mFovControlData.zoomWide : mFovControlData.zoomTele;
            uint32_t zoomIsp   = isMainCamFovWider() ?
                    mFovControlData.zoomWideIsp : mFovControlData.zoomTeleIsp;
            uint32_t zoomStep = isMainCamFovWider() ?
                    mFovControlData.zoomRatioWide : mFovControlData.zoomRatioTele;
            if (useCropRegion) {
                setCropParam(CAM_TYPE_MAIN, zoomStep, paramsMainCam);
            }
            setZoomParam(CAM_TYPE_MAIN, zoomInfo, zoomTotal, zoomIsp,
                        snapshotPostProcess, paramsMainCam, useCropRegion);

            // Update zoom value for aux camera param buffer
            zoomTotal = isMainCamFovWider() ?
                    mFovControlData.zoomTele : mFovControlData.zoomWide;
            zoomIsp = isMainCamFovWider() ?
                    mFovControlData.zoomTeleIsp : mFovControlData.zoomWideIsp;
            zoomStep = isMainCamFovWider() ?
                    mFovControlData.zoomRatioTele : mFovControlData.zoomRatioWide;
            if (useCropRegion) {
                setCropParam(CAM_TYPE_AUX, zoomStep, paramsAuxCam);
            }
            setZoomParam(CAM_TYPE_AUX, zoomInfo, zoomTotal, zoomIsp,
                        snapshotPostProcess, paramsAuxCam, useCropRegion);

            // Generate FOV-control result
            generateFovControlResult();
            if (mFovControlData.configCompleted) {
                mFovControlParm.zoom_valid = 0;
            }
        }

        // Translate focus areas
        if (paramsMainCam->is_valid[CAM_INTF_PARM_AF_ROI]) {
            cam_roi_info_t roiAfMain;
            cam_roi_info_t roiAfAux;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_PARM_AF_ROI, roiAfMain);
            if (roiAfMain.num_roi > 0) {
                roiAfAux = translateFocusAreas(roiAfMain, CAM_TYPE_AUX);
                roiAfMain = translateFocusAreas(roiAfMain, CAM_TYPE_MAIN);
                ADD_SET_PARAM_ENTRY_TO_BATCH(paramsAuxCam, CAM_INTF_PARM_AF_ROI, roiAfAux);
                ADD_SET_PARAM_ENTRY_TO_BATCH(paramsMainCam, CAM_INTF_PARM_AF_ROI, roiAfMain);
            }
        } else if (paramsMainCam->is_valid[CAM_INTF_META_AF_ROI]) {
            cam_area_t roiAfMain;
            cam_area_t roiAfAux;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_META_AF_ROI, roiAfMain);
            roiAfAux = translateRoi(roiAfMain, CAM_TYPE_AUX);
            roiAfMain = translateRoi(roiAfMain, CAM_TYPE_MAIN);
            ADD_SET_PARAM_ENTRY_TO_BATCH(paramsAuxCam, CAM_INTF_META_AF_ROI, roiAfAux);
            ADD_SET_PARAM_ENTRY_TO_BATCH(paramsMainCam, CAM_INTF_META_AF_ROI, roiAfMain);
        }

        // Translate metering areas
        if (paramsMainCam->is_valid[CAM_INTF_PARM_AEC_ROI]) {
            cam_set_aec_roi_t roiAecMain;
            cam_set_aec_roi_t roiAecAux;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_PARM_AEC_ROI, roiAecMain);
            if (roiAecMain.aec_roi_enable == CAM_AEC_ROI_ON) {
                roiAecAux = translateMeteringAreas(roiAecMain, CAM_TYPE_AUX);
                roiAecMain = translateMeteringAreas(roiAecMain, CAM_TYPE_MAIN);
                ADD_SET_PARAM_ENTRY_TO_BATCH(paramsAuxCam, CAM_INTF_PARM_AEC_ROI, roiAecAux);
                ADD_SET_PARAM_ENTRY_TO_BATCH(paramsMainCam, CAM_INTF_PARM_AEC_ROI, roiAecMain);
            }
        } else if (paramsMainCam->is_valid[CAM_INTF_META_AEC_ROI]) {
            cam_area_t roiAecMain;
            cam_area_t roiAecAux;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_META_AEC_ROI, roiAecMain);
            roiAecAux = translateRoi(roiAecMain, CAM_TYPE_AUX);
            roiAecMain = translateRoi(roiAecMain, CAM_TYPE_MAIN);
            ADD_SET_PARAM_ENTRY_TO_BATCH(paramsAuxCam, CAM_INTF_META_AEC_ROI, roiAecAux);
            ADD_SET_PARAM_ENTRY_TO_BATCH(paramsMainCam, CAM_INTF_META_AEC_ROI, roiAecMain);
        }

        /*Set torch param only in the active session.With present LPM logic in MCT, if the session
          which has been put to LPM wakes up, MCT will start applying all the previous settings at
          one go. This can lead to a sudden unwanted flash sometimes. To avoid this, added WA
          in HAL to set torch mode only in the active session.*/
        if (paramsMainCam->is_valid[CAM_INTF_PARM_LED_MODE]) {
            int32_t flashMode = CAM_FLASH_MODE_OFF;
            READ_PARAM_ENTRY(paramsMainCam, CAM_INTF_PARM_LED_MODE, flashMode);
            if (CAM_FLASH_MODE_TORCH == flashMode) {
                if (isMaster(mFovControlData.camWide)) {
                    paramsAuxCam->is_valid[CAM_INTF_PARM_LED_MODE] = 0;
                } else {
                    paramsMainCam->is_valid[CAM_INTF_PARM_LED_MODE] = 0;
                }
            }
        }
        rc = NO_ERROR;
    }
    return rc;
}


/*===========================================================================
 * FUNCTION   : processResultMetadata
 *
 * DESCRIPTION: Process the metadata from main and aux cameras to generate the
 *              result metadata. The result metadata should be the metadata
 *              coming from the master camera. If aux camera is master, the
 *              subset of the metadata needs to be translated to main as that's
 *              the only camera seen by the application.
 *
 * PARAMETERS :
 * @metaMain  : metadata for main camera
 * @metaAux   : metadata for aux camera
 *
 * RETURN :
 * Result metadata for the logical camera. After successfully processing main
 * and aux metadata, the result metadata points to either main or aux metadata
 * based on which one was the master. In case of failure, it returns NULL.
 *==========================================================================*/
metadata_buffer_t* QCameraFOVControl::processResultMetadata(
        metadata_buffer_t*  metaMain,
        metadata_buffer_t*  metaAux)
{
    metadata_buffer_t* metaResult = NULL;

    if (metaMain || metaAux) {

        mMutex.lock();

        metadata_buffer_t *meta   = metaMain ? metaMain : metaAux;
        cam_sync_type_t masterCam = mFovControlData.updateResultState ?
                mFovControlResult.camMasterPreview : mFovControlResultCachedCopy.camMasterPreview;

        // Book-keep the needed metadata from main camera and aux camera
        IF_META_AVAILABLE(cam_sac_output_info_t, spatialAlignOutput,
                CAM_INTF_META_DC_SAC_OUTPUT_INFO, meta) {

            // Get master camera hint
            if (spatialAlignOutput->is_master_hint_valid) {
                uint8_t master = spatialAlignOutput->master_hint;
                mFovControlData.spatialAlignResult.fallbackComplete = 0;
                if (master == CAM_ROLE_WIDE) {
                    mFovControlData.spatialAlignResult.camMasterHint = mFovControlData.camWide;
                } else if (master == CAM_ROLE_TELE) {
                    mFovControlData.spatialAlignResult.camMasterHint = mFovControlData.camTele;
                } else if (master == CAM_ROLE_WIDE_FALLBACK) {
                    // Confirmation on fallback complete
                    mFovControlData.spatialAlignResult.camMasterHint = 0;
                    mFovControlData.spatialAlignResult.fallbackComplete = 1;
                }
            }

            // Get master camera used for the preview in the frame corresponding to this metadata
            if (spatialAlignOutput->is_master_preview_valid) {
                uint8_t master = spatialAlignOutput->master_preview;
                if (master == CAM_ROLE_WIDE) {
                    masterCam = mFovControlData.camWide;
                    mFovControlData.spatialAlignResult.camMasterPreview = masterCam;
                } else if (master == CAM_ROLE_TELE) {
                    masterCam = mFovControlData.camTele;
                    mFovControlData.spatialAlignResult.camMasterPreview = masterCam;
                }
            }

            // Get master camera used for 3A in the frame corresponding to this metadata
            if (spatialAlignOutput->is_master_3A_valid) {
                uint8_t master = spatialAlignOutput->master_3A;
                if (master == CAM_ROLE_WIDE) {
                    mFovControlData.spatialAlignResult.camMaster3A = mFovControlData.camWide;
                } else if (master == CAM_ROLE_TELE) {
                    mFovControlData.spatialAlignResult.camMaster3A = mFovControlData.camTele;
                }
            }

            // Get spatial alignment ready status
            if (spatialAlignOutput->is_ready_status_valid) {
                mFovControlData.spatialAlignResult.readyStatus = spatialAlignOutput->ready_status;
            }

            LOGD("master_hint_valid %d masterCam %d FBcomplete %d camMaster3A %d readyStatus %d",
                    mFovControlData.spatialAlignResult.camMasterHint,
                    mFovControlData.spatialAlignResult.camMasterPreview,
                    mFovControlData.spatialAlignResult.fallbackComplete,
                    mFovControlData.spatialAlignResult.camMaster3A,
                    mFovControlData.spatialAlignResult.readyStatus);
        }

        metadata_buffer_t *metaWide = isMainCamFovWider() ? metaMain : metaAux;
        metadata_buffer_t *metaTele = isMainCamFovWider() ? metaAux : metaMain;

        // Get spatial alignment output info for wide camera
        if (metaWide) {
            IF_META_AVAILABLE(cam_sac_output_info_t, spatialAlignOutput,
                CAM_INTF_META_DC_SAC_OUTPUT_INFO, metaWide) {

                // Get spatial alignment output shift for wide camera
                if (spatialAlignOutput->is_output_shift_valid) {
                    // Calculate the spatial alignment shift for the current stream dimensions based
                    // on the reference resolution used for the output shift.
                    float horzShiftFactor = (float)mFovControlData.previewSize.width /
                            spatialAlignOutput->reference_res_for_output_shift.width;
                    float vertShiftFactor = (float)mFovControlData.previewSize.height /
                            spatialAlignOutput->reference_res_for_output_shift.height;

                    mFovControlData.spatialAlignResult.shiftWide.shiftHorz =
                            spatialAlignOutput->output_shift.shift_horz * horzShiftFactor;
                    mFovControlData.spatialAlignResult.shiftWide.shiftVert =
                            spatialAlignOutput->output_shift.shift_vert * vertShiftFactor;

                    LOGD("SAC output shift for Wide: x:%d, y:%d",
                            mFovControlData.spatialAlignResult.shiftWide.shiftHorz,
                            mFovControlData.spatialAlignResult.shiftWide.shiftVert);
                }

                // Get the AF roi shift for wide camera
                if (spatialAlignOutput->is_focus_roi_shift_valid) {
                    // Calculate the spatial alignment shift for the current stream dimensions based
                    // on the reference resolution used for the output shift.
                    float horzShiftFactor = (float)mFovControlData.previewSize.width /
                            spatialAlignOutput->reference_res_for_focus_roi_shift.width;
                    float vertShiftFactor = (float)mFovControlData.previewSize.height /
                            spatialAlignOutput->reference_res_for_focus_roi_shift.height;

                    mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftHorz =
                            spatialAlignOutput->focus_roi_shift.shift_horz * horzShiftFactor;
                    mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftVert =
                            spatialAlignOutput->focus_roi_shift.shift_vert * vertShiftFactor;

                    LOGD("SAC AF ROI shift for Wide: x:%d, y:%d",
                            mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftHorz,
                            mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftVert);
                }
            }
        }

        // Get spatial alignment output info for tele camera
        if (metaTele) {
            IF_META_AVAILABLE(cam_sac_output_info_t, spatialAlignOutput,
                CAM_INTF_META_DC_SAC_OUTPUT_INFO, metaTele) {

                // Get spatial alignment output shift for tele camera
                if (spatialAlignOutput->is_output_shift_valid) {
                    // Calculate the spatial alignment shift for the current stream dimensions based
                    // on the reference resolution used for the output shift.
                    float horzShiftFactor = (float)mFovControlData.previewSize.width /
                            spatialAlignOutput->reference_res_for_output_shift.width;
                    float vertShiftFactor = (float)mFovControlData.previewSize.height /
                            spatialAlignOutput->reference_res_for_output_shift.height;

                    mFovControlData.spatialAlignResult.shiftTele.shiftHorz =
                            spatialAlignOutput->output_shift.shift_horz * horzShiftFactor;
                    mFovControlData.spatialAlignResult.shiftTele.shiftVert =
                            spatialAlignOutput->output_shift.shift_vert * vertShiftFactor;

                    LOGD("SAC output shift for Tele: x:%d, y:%d",
                            mFovControlData.spatialAlignResult.shiftTele.shiftHorz,
                            mFovControlData.spatialAlignResult.shiftTele.shiftVert);
                }

                // Get the AF roi shift for tele camera
                if (spatialAlignOutput->is_focus_roi_shift_valid) {
                    // Calculate the spatial alignment shift for the current stream dimensions based
                    // on the reference resolution used for the output shift.
                    float horzShiftFactor = (float)mFovControlData.previewSize.width /
                            spatialAlignOutput->reference_res_for_focus_roi_shift.width;
                    float vertShiftFactor = (float)mFovControlData.previewSize.height /
                            spatialAlignOutput->reference_res_for_focus_roi_shift.height;

                    mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftHorz =
                            spatialAlignOutput->focus_roi_shift.shift_horz * horzShiftFactor;
                    mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftVert =
                            spatialAlignOutput->focus_roi_shift.shift_vert * vertShiftFactor;

                    LOGD("SAC AF ROI shift for Tele: x:%d, y:%d",
                            mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftHorz,
                            mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftVert);
                }
            }
        }

        // Get the requested LPM
        if (meta) {
            IF_META_AVAILABLE(cam_sac_output_info_t, spatialAlignOutput,
                CAM_INTF_META_DC_SAC_OUTPUT_INFO, meta) {
                if (spatialAlignOutput->is_lpm_info_valid) {
                    // Update active cameras requested by spatial alignment
                    for (int i = 0; i < DUALCAM_CAMERA_CNT; ++i) {
                        if(spatialAlignOutput->lpm_info[i].camera_role == CAM_ROLE_WIDE) {
                            if (spatialAlignOutput->lpm_info[i].lpm_enable) {
                                mFovControlData.spatialAlignResult.activeCameras &=
                                        ~mFovControlData.camWide;
                            } else {
                                mFovControlData.spatialAlignResult.activeCameras |=
                                        mFovControlData.camWide;
                            }
                        }
                        if(spatialAlignOutput->lpm_info[i].camera_role == CAM_ROLE_TELE) {
                            if (spatialAlignOutput->lpm_info[i].lpm_enable) {
                                mFovControlData.spatialAlignResult.activeCameras &=
                                        ~mFovControlData.camTele;
                            } else {
                                mFovControlData.spatialAlignResult.activeCameras |=
                                        mFovControlData.camTele;
                            }
                        }
                        LOGD("SAC LPM info: cam role: %d, lpm: %d",
                                spatialAlignOutput->lpm_info[i].camera_role,
                                spatialAlignOutput->lpm_info[i].lpm_enable);
                    }
                }
            }
        }

        // Update the camera streaming status
        if (metaWide) {
            mFovControlData.wideCamStreaming = true;
            ++mFovControlData.frameCountWide;
            IF_META_AVAILABLE(uint8_t, enableLPM, CAM_INTF_META_DC_LOW_POWER_ENABLE, metaWide) {
                if (*enableLPM) {
                    // If LPM enabled is 1, this is probably the last metadata returned
                    // before going into LPM state
                    mFovControlData.wideCamStreaming = false;
                    mFovControlData.frameCountWide = 0;
                }
            }
        }

        if (metaTele) {
            mFovControlData.teleCamStreaming = true;
            ++mFovControlData.frameCountTele;
            IF_META_AVAILABLE(uint8_t, enableLPM, CAM_INTF_META_DC_LOW_POWER_ENABLE, metaTele) {
                if (*enableLPM) {
                    // If LPM enabled is 1, this is probably the last metadata returned
                    // before going into LPM state
                    mFovControlData.teleCamStreaming = false;
                    mFovControlData.frameCountTele = 0;
                }
            }
        }

        LOGD("active cameras: SAC requested %d, current wide streaming %d, tele streaming %d",
                mFovControlData.spatialAlignResult.activeCameras, mFovControlData.wideCamStreaming,
                mFovControlData.teleCamStreaming);

        // Get AF status
        if (metaMain) {
            IF_META_AVAILABLE(uint32_t, afState, CAM_INTF_META_AF_STATE, metaMain) {
                mFovControlData.afStatusMain = *afState;
                LOGD("AF state: Main cam: %d", mFovControlData.afStatusMain);
            }

            IF_META_AVAILABLE(float, luxIndex, CAM_INTF_META_AEC_LUX_INDEX, metaMain) {
                mFovControlData.status3A.main.ae.luxIndex = *luxIndex;
                LOGD("Lux Index: Main cam: %f", mFovControlData.status3A.main.ae.luxIndex);
            }

            IF_META_AVAILABLE(int32_t, objDist, CAM_INTF_META_AF_OBJ_DIST_CM, metaMain) {
                mFovControlData.status3A.main.af.focusDistCm = (*objDist < 0) ? 0 : *objDist;
                LOGD("Obj Dist: Main cam: %d", mFovControlData.status3A.main.af.focusDistCm);
            }
        }
        if (metaAux) {
            IF_META_AVAILABLE(uint32_t, afState, CAM_INTF_META_AF_STATE, metaAux) {
                mFovControlData.afStatusAux = *afState;
                LOGD("AF state: Aux cam: %d", mFovControlData.afStatusAux);
            }

            IF_META_AVAILABLE(float, luxIndex, CAM_INTF_META_AEC_LUX_INDEX, metaAux) {
                mFovControlData.status3A.aux.ae.luxIndex = *luxIndex;
                LOGD("Lux Index: Aux cam: %f", mFovControlData.status3A.aux.ae.luxIndex);
            }

            IF_META_AVAILABLE(int32_t, objDist, CAM_INTF_META_AF_OBJ_DIST_CM, metaAux) {
                mFovControlData.status3A.aux.af.focusDistCm = (*objDist < 0) ? 0 : *objDist;
                LOGD("Obj Dist: Aux cam: %d", mFovControlData.status3A.aux.af.focusDistCm);
            }
        }

        if ((masterCam == CAM_TYPE_AUX) && metaAux) {
            // Translate face detection ROI from aux camera
            IF_META_AVAILABLE(cam_face_detection_data_t, metaFD,
                    CAM_INTF_META_FACE_DETECTION, metaAux) {
                cam_face_detection_data_t metaFDTranslated;
                metaFDTranslated = translateRoiFD(*metaFD, CAM_TYPE_AUX);
                ADD_SET_PARAM_ENTRY_TO_BATCH(metaAux, CAM_INTF_META_FACE_DETECTION,
                        metaFDTranslated);
            }
            metaResult = metaAux;
        }
        else if ((masterCam == CAM_TYPE_MAIN) && metaMain) {
            // Translate face detection ROI from main camera
            IF_META_AVAILABLE(cam_face_detection_data_t, metaFD,
                    CAM_INTF_META_FACE_DETECTION, metaMain) {
                cam_face_detection_data_t metaFDTranslated;
                metaFDTranslated = translateRoiFD(*metaFD, CAM_TYPE_MAIN);
                ADD_SET_PARAM_ENTRY_TO_BATCH(metaMain, CAM_INTF_META_FACE_DETECTION,
                        metaFDTranslated);
            }
            metaResult = metaMain;
        } else {
            // Metadata for the master camera was dropped
            metaResult = NULL;
            LOGD("Metadata Result is NULL")
        }

        // If snapshot postprocess is enabled, consolidate the AF status to be sent to the app
        // when in the transition state.
        // Only return focused if both are focused.
        bool snapshotPostProcess = mFovControlData.updateResultState ?
                mFovControlResult.snapshotPostProcess :
                mFovControlResultCachedCopy.snapshotPostProcess;
        uint32_t camState = mFovControlData.updateResultState ?
                mFovControlResult.activeCameras : mFovControlResultCachedCopy.activeCameras;
        if (snapshotPostProcess &&
                (camState == (CAM_TYPE_MAIN | CAM_TYPE_AUX)) &&
                metaResult) {
            if (!mDualCamParams.paramsMain.isAFSupported)
                mFovControlData.afStatusMain = mFovControlData.afStatusAux;
            else if (!mDualCamParams.paramsAux.isAFSupported)
                mFovControlData.afStatusAux = mFovControlData.afStatusMain;
            if (((mFovControlData.afStatusMain == CAM_AF_STATE_FOCUSED_LOCKED) ||
                    (mFovControlData.afStatusMain == CAM_AF_STATE_NOT_FOCUSED_LOCKED)) &&
                    ((mFovControlData.afStatusAux == CAM_AF_STATE_FOCUSED_LOCKED) ||
                    (mFovControlData.afStatusAux == CAM_AF_STATE_NOT_FOCUSED_LOCKED))) {
                // If both indicate focused, return focused.
                // If either one indicates 'not focused', return 'not focused'.
                if ((mFovControlData.afStatusMain == CAM_AF_STATE_FOCUSED_LOCKED) &&
                        (mFovControlData.afStatusAux  == CAM_AF_STATE_FOCUSED_LOCKED)) {
                    ADD_SET_PARAM_ENTRY_TO_BATCH(metaResult, CAM_INTF_META_AF_STATE,
                            CAM_AF_STATE_FOCUSED_LOCKED);
                } else {
                    ADD_SET_PARAM_ENTRY_TO_BATCH(metaResult, CAM_INTF_META_AF_STATE,
                            CAM_AF_STATE_NOT_FOCUSED_LOCKED);
                }
            } else {
                // If either one indicates passive state or active scan, return that state
                if ((mFovControlData.afStatusMain != CAM_AF_STATE_FOCUSED_LOCKED) &&
                        (mFovControlData.afStatusMain != CAM_AF_STATE_NOT_FOCUSED_LOCKED)) {
                    ADD_SET_PARAM_ENTRY_TO_BATCH(metaResult, CAM_INTF_META_AF_STATE,
                            mFovControlData.afStatusMain);
                } else {
                    ADD_SET_PARAM_ENTRY_TO_BATCH(metaResult, CAM_INTF_META_AF_STATE,
                            mFovControlData.afStatusAux);
                }
            }
            IF_META_AVAILABLE(uint32_t, afState, CAM_INTF_META_AF_STATE, metaResult) {
                LOGD("Result AF state: %d", *afState);
            }
        }

        mMutex.unlock();

        // Generate FOV-control result only if the result meta is valid
        if (metaResult) {
            generateFovControlResult();
        }
    }
    return metaResult;
}


/*===========================================================================
 * FUNCTION   : generateFovControlResult
 *
 * DESCRIPTION: Generate FOV control result
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *
 *==========================================================================*/
void QCameraFOVControl::generateFovControlResult()
{
    Mutex::Autolock lock(mMutex);

    if (isBayerMono()) {
        // Bayer is primary camera
        mFovControlResult.camMasterPreview  = CAM_TYPE_MAIN;
        mFovControlResult.camMaster3A  = CAM_TYPE_MAIN;
        mFovControlResult.activeCameras = MM_CAMERA_DUAL_CAM;
        mFovControlData.configCompleted = true;
        if(mHalPPType != CAM_HAL_PP_TYPE_NONE)
        {
            mFovControlResult.snapshotPostProcess = true;
        } else {
            mFovControlResult.snapshotPostProcess = false;
        }
        mFovControlResult.isValid = true;
        return;
     } else if (mHalPPType == CAM_HAL_PP_TYPE_BOKEH) {
        // Tele is primary camera
        mFovControlResult.camMasterPreview  = mFovControlData.camTele;
        mFovControlResult.camMaster3A  = mFovControlData.camTele;
        mFovControlResult.activeCameras =
                (uint32_t)(mFovControlData.camTele | mFovControlData.camWide);
        mFovControlData.configCompleted = true;
        mFovControlResult.snapshotPostProcess = true;
        mFovControlResult.oisMode = OIS_MODE_HOLD;
        mFovControlResult.isValid = true;
        return;
    }

    float zoom = findZoomRatio(mFovControlData.zoomUser) / (float)mFovControlData.zoomRatioTable[0];
    uint32_t zoomCur  = mFovControlData.zoomUser;
    uint32_t zoomPrev = mFovControlData.zoomUserPrev;

    if (mFovControlData.configCompleted == false) {
        // Return as invalid result if the FOV-control configuration is not yet complete
        mFovControlResult.isValid = false;
        return;
    }

    // Update previous zoom value
    mFovControlData.zoomUserPrev = mFovControlData.zoomUser;

    uint16_t  currentLuxIdx     = 0;
    uint16_t  currentFocusDist  = 0;

    if (isMaster(CAM_TYPE_MAIN)) {
        currentLuxIdx     = mFovControlData.status3A.main.ae.luxIndex;
        currentFocusDist  = mFovControlData.status3A.main.af.focusDistCm;
    } else {
        currentLuxIdx     = mFovControlData.status3A.aux.ae.luxIndex;
        currentFocusDist  = mFovControlData.status3A.aux.af.focusDistCm;
    }

    cam_sync_type_t camWide = mFovControlData.camWide;
    cam_sync_type_t camTele = mFovControlData.camTele;

    uint16_t thresholdLuxIdx     = mFovControlConfig.auxSwitchLuxIdxMax;
    uint16_t thresholdFocusDist  = mFovControlConfig.auxSwitchFocusDistCmMin;

    LOGD("current LuxIdx: %d, threshold LuxIdx: %d", currentLuxIdx, thresholdLuxIdx);
    LOGD("current focus dist: %d, threshold focus dist: %d", currentFocusDist, thresholdFocusDist);

    dual_cam_state prevCamState = mFovControlData.camState;

    if (zoomCur == zoomPrev) {
        mFovControlData.zoomDirection = ZOOM_STABLE;
    } else if (zoomCur > zoomPrev) {
        mFovControlData.zoomDirection = ZOOM_IN;
    } else {
        mFovControlData.zoomDirection = ZOOM_OUT;
    }

    // Update snapshot post-process flags
    if (mFovControlConfig.snapshotPPConfig.enablePostProcess &&
        (zoom >= mFovControlConfig.snapshotPPConfig.zoomMin) &&
        (zoom <= mFovControlConfig.snapshotPPConfig.zoomMax)) {
        mFovControlResult.snapshotPostProcessZoomRange = true;
    } else {
        mFovControlResult.snapshotPostProcessZoomRange = false;
    }

    if (mFovControlResult.snapshotPostProcessZoomRange &&
        (currentLuxIdx <= mFovControlConfig.snapshotPPConfig.LuxIdxMax) &&
        (currentFocusDist >= mFovControlConfig.snapshotPPConfig.focusDistanceMin)) {
        mFovControlResult.snapshotPostProcess = true;
    } else {
        mFovControlResult.snapshotPostProcess = false;
    }

    // Default the fallback to No fallback
    mFovControlResult.fallback = CAM_NO_FALLBACK;

    switch (mFovControlData.camState) {
        case STATE_WIDE:
            // Start timerWellLitNonMacroScene timer if the scene is bright and non-macro,
            // otherwise inactivate.
            if (mFovControlData.fallbackEnabled) {
                if ((currentLuxIdx <= thresholdLuxIdx) &&
                        (currentFocusDist >= thresholdFocusDist)) {
                    if (!mFovControlData.timerWellLitNonMacroScene.active) {
                        startTimer(&mFovControlData.timerWellLitNonMacroScene,
                                mFovControlConfig.fallbackTimeout);
                        LOGD("state: wide: start timer for well lit, non-macro scene");
                    }
                } else {
                    inactivateTimer(&mFovControlData.timerWellLitNonMacroScene);
                }
            }

            // Reset fallback to main flag if zoom is less than cutover point
            if (mFovControlData.fallbackEnabled &&
                    mFovControlData.fallbackToWide &&
                    !needDualZone()) {
                // Reset fallback to wide flag
                mFovControlData.fallbackToWide = false;
                LOGD("state: wide: fallback complete. zoom less than tele-to-wide threshold");
            }

            // Check if the scene is good for aux (bright and far focused) if fallback is enabled
            if (!mFovControlData.fallbackEnabled ||
                    ((currentLuxIdx <= thresholdLuxIdx) &&
                    (currentFocusDist >= thresholdFocusDist))) {
                /* Switch to transition state if -
                 1. Scene is bright and far focused
                 2. Force cameras to wakeup (Bundled snapshot / autofocus)
                 3. Zoom is above threshold with user zooming in
                 4. No low light / macro scene fallback triggered
                 5. Spatial alignment disabling LPM on both if FOVControl fallback is disabled
                 Lower constraint if zooming in
                 This path is taken for the normal behavior - there was no fallback to wide state
                 due to low light, macro-scene and user zooms in with zoom hitting the threshold */
                if ((mFovControlData.forceCameraWakeup) ||
                    (!mFovControlData.fallbackEnabled &&
                    ((mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_OEM) &&
                    (mFovControlData.spatialAlignResult.activeCameras == (camWide | camTele)))) ||
                    (needDualZone() &&
                    ((mFovControlData.zoomDirection == ZOOM_IN) ||
                    (mFovControlData.fallbackEnabled &&
                    (zoom >= mFovControlData.transitionParams.cutOverTeleToWide))) &&
                    (mFovControlData.fallbackToWide == false))) {
                    mFovControlData.camState = STATE_TRANSITION;
                    mFovControlResult.activeCameras = (camWide | camTele);

                    if (mFovControlData.forceCameraWakeup) {
                        LOGD("state: wide: Forced camera wakeup (bundled snapshot/autofocus)."
                                " Switching to Transition state");
                    }
                }
                /* 6. Low light / macro scene fallback was triggered and completed
                 Higher constraint if not zooming in.
                 This path is taken if the state had transitioned to wide due to low light or
                 macro scene - check that using mFovControlData.fallbackToWide flag.
                 timerWellLitNonMacroScene will timeout if the scene continues to be bright and
                 non-macro for the duration specified as FOVC_FALLBACK_TIMEOUT_MS.
                 This indicates that the scene is bright and non-macro, as a result it's safe to
                 move back to transition state */
                else if (mFovControlData.fallbackEnabled &&
                            mFovControlData.fallbackToWide &&
                            isTimedOut(mFovControlData.timerWellLitNonMacroScene)) {
                    // Enter the transition state
                    mFovControlData.camState = STATE_TRANSITION;
                    mFovControlResult.activeCameras = (camWide | camTele);

                    // Reset fallback to wide flag
                    mFovControlData.fallbackToWide = false;
                    LOGD("state: wide: fallback complete. Switching to Transition state");
                }
            } else {
                LOGD("state: wide: low light / macro scene. Remain in Wide state.");
                // Set the fallback to Wide
                mFovControlResult.fallback = CAM_WIDE_FALLBACK;
            }
            break;

        case STATE_TRANSITION:
            // Start const-zoom timer if needed
            if (mFovControlData.zoomDirection == ZOOM_STABLE) {
                if (!mFovControlData.timerConstZoom.active ||
                        mFovControlData.forceCameraWakeup) {
                    /* Provide the timeout value based on whether zoom is in snapshot
                     postprocess range. */
                    startTimer(&mFovControlData.timerConstZoom,
                            (mFovControlResult.snapshotPostProcessZoomRange ?
                            mFovControlConfig.constZoomTimeoutSnapshotPPRange :
                            mFovControlConfig.constZoomTimeout));
                    LOGD("state: transition: start const zoom timer");
                }
            } else {
                inactivateTimer(&mFovControlData.timerConstZoom);
            }

            if (mFovControlData.fallbackEnabled) {
                if ((currentLuxIdx > thresholdLuxIdx) ||
                        (currentFocusDist < thresholdFocusDist)) {
                    /* Update timerLowLitMacroScene / fallback timer if the scene continues
                     to be dark or macro */
                    if (!mFovControlData.timerLowLitMacroScene.active) {
                        startTimer(&mFovControlData.timerLowLitMacroScene,
                                mFovControlConfig.fallbackTimeout);
                        inactivateTimer(&mFovControlData.timerWellLitNonMacroScene);
                        LOGD("state: transition: start fallback timer");
                    }

                    // If the fallback timer times out, set the fallback flag to true
                    if (isTimedOut(mFovControlData.timerLowLitMacroScene)) {
                        mFovControlData.fallbackToWide = true;
                        mFovControlData.fallbackInitedInTransition = true;
                        LOGD("state: transition: fallback timer timed out");
                    }
                } else {
                     /* Update timerWellLitNonMacroScene timer to check if the scene is
                      bright and non-macro */
                    if (!mFovControlData.timerWellLitNonMacroScene.active) {
                        startTimer(&mFovControlData.timerWellLitNonMacroScene,
                                mFovControlConfig.fallbackTimeout);
                        inactivateTimer(&mFovControlData.timerLowLitMacroScene);
                        LOGD("state: transition: start timer for well lit, non-macro scene");
                    }

                    // If the fallback timer times out, set the fallback flag to true
                    if (isTimedOut(mFovControlData.timerWellLitNonMacroScene)) {
                        mFovControlData.fallbackToWide = false;
                        mFovControlData.fallbackInitedInTransition = false;
                        LOGD("state: transition: well lit, non-macro scene timer timed out");
                    }
                }
            }

            // Set the master info
            // Switch to wide
            if (isMaster(camTele) &&
                    canSwitchMasterTo(CAM_ROLE_WIDE)) {
                mFovControlResult.camMasterPreview = camWide;
                mFovControlResult.camMaster3A      = camWide;
                LOGD("state: transition: Switching master to Wide");
            }
            // switch to tele
            else if (isMaster(camWide) &&
                    canSwitchMasterTo(CAM_ROLE_TELE)) {
                mFovControlResult.camMasterPreview = camTele;
                mFovControlResult.camMaster3A      = camTele;
                LOGD("state: transition: Switching master to Tele");
            }

            /* Change the transition state if necessary.
             Switch to wide state if -
             1. The low light / macro scene detected */
            if (mFovControlData.fallbackEnabled && mFovControlData.fallbackToWide) {
                if (isMaster(camWide) &&
                        (!mFovControlData.availableSpatialAlignSolns ||
                                (mFovControlData.spatialAlignResult.camMasterPreview == camWide)) &&
                        (!mFovControlData.fallbackInitedInTransition ||
                                FOVC_TELE_LPM_IN_TRANSITION_WITH_FALLBACK)) {
                    // Put Tele to LPM only when Spatial alignment starts outputing Wide
                    mFovControlData.camState = STATE_WIDE;
                    mFovControlResult.activeCameras = camWide;

                    LOGD("state: transition: low light / Macro scene fallback triggered."
                        " Switching to Wide state");
                }
            // 2. Zoom is below threshold
            } else if (!needDualZone() && isMaster(camWide)) {
                mFovControlData.camState        = STATE_WIDE;
                mFovControlResult.activeCameras = camWide;
            /* Switch to tele state if -
             1. Zoom is above threshold */
            } else if (!needDualZone() && isMaster(camTele)) {
                mFovControlData.camState        = STATE_TELE;
                mFovControlResult.activeCameras = camTele;
            } else if (isTimedOut(mFovControlData.timerConstZoom) &&
                        (mFovControlData.fallbackEnabled ||
                        ((mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_OEM) &&
                            (mFovControlData.spatialAlignResult.activeCameras !=
                            (camWide | camTele))))) {
                /* timerConstZoom will timeout if the zoom does not change for the duration
                 specified as FOVC_CONST_ZOOM_TIMEOUT_MS
                 If the zoom is stable put the non-master camera to LPM for power optimization */
                if (isMaster(camWide)) {
                    mFovControlData.camState        = STATE_WIDE;
                    mFovControlResult.activeCameras = camWide;
                    LOGD("state: transition: const zoom timer timed out. Switching to Wide state");
                } else {
                    mFovControlData.camState        = STATE_TELE;
                    mFovControlResult.activeCameras = camTele;
                    LOGD("state: transition: const zoom timer timed out. Switching to Tele state");
                }
            }
            break;

        case STATE_TELE:
            /* Update the fallback timer if the scene continues to be dark or macro,
             reset otherwise */
            if (mFovControlData.fallbackEnabled) {
                if ((currentLuxIdx > thresholdLuxIdx) ||
                        (currentFocusDist < thresholdFocusDist)) {
                    if (!mFovControlData.timerLowLitMacroScene.active) {
                        startTimer(&mFovControlData.timerLowLitMacroScene,
                                mFovControlConfig.fallbackTimeout);
                        LOGD("state: tele: start fallback timer");
                    }

                    /* timerLowLitMacroScene will timeout if the scene continues to be dark or macro
                     for the duration specified as FOVC_FALLBACK_TIMEOUT_MS
                     If the fallback timer times out, set the fallback flag to true */
                    if (isTimedOut(mFovControlData.timerLowLitMacroScene)) {
                        mFovControlData.fallbackToWide = true;
                        mFovControlData.fallbackInitedInTransition = false;
                        LOGD("state: tele: fallback timer timed out");
                    }
                } else {
                    inactivateTimer(&mFovControlData.timerLowLitMacroScene);
                }
            }

            /* Switch to transition state if -
             1. Start fallback to wide if the low light / macro scene detected
             2. Force cameras to wakeup (Bundled snapshot / autofocus) */
            if ((mFovControlData.fallbackEnabled && mFovControlData.fallbackToWide) ||
                    (mFovControlData.forceCameraWakeup)) {
                mFovControlData.camState = STATE_TRANSITION;
                mFovControlResult.activeCameras = (camWide | camTele);

                if (mFovControlData.forceCameraWakeup) {
                    LOGD("state: tele: Forced camera wakeup (bundled snapshot/autofocus)."
                        " Switching to Transition state");
                } else if (mFovControlData.fallbackEnabled && mFovControlData.fallbackToWide) {
                    LOGD("state: tele: low light / Macro scene fallback triggered."
                        " Switching to Transition state");
                }
            }
            /* 3. Zooming out and if the zoom value is less than the threshold
               4. Spatial alignment disabling LPM on both if FOVControl fallback is disabled */
            else if ((needDualZone() && (mFovControlData.zoomDirection == ZOOM_OUT)) ||
                        (!mFovControlData.fallbackEnabled &&
                        ((mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_OEM) &&
                        (mFovControlData.spatialAlignResult.activeCameras ==
                            (camWide | camTele))))) {
                mFovControlData.camState = STATE_TRANSITION;
                mFovControlResult.activeCameras = (camWide | camTele);
            }

            break;
    }

    // Inactivate the timers if the camera state changed
    if (prevCamState != mFovControlData.camState) {
        inactivateTimer(&mFovControlData.timerLowLitMacroScene);
        inactivateTimer(&mFovControlData.timerWellLitNonMacroScene);
        inactivateTimer(&mFovControlData.timerConstZoom);
        LOGD("Active camera state changed. Reset timers");
    }

    // Only one active camera in case of thermal throttle and disable snapshot post-processing
    if (mFovControlData.thermalThrottle) {
        mFovControlResult.activeCameras = isMaster(camWide) ? camWide : camTele;
        mFovControlResult.snapshotPostProcess = false;
    }

    // Update OIS mode in the result
    if (mFovControlData.oisSetting == OIS_ACTIVE_IN_LPM) {
        mFovControlResult.oisMode = (mFovControlData.camState == STATE_TRANSITION) ?
                OIS_MODE_HOLD : OIS_MODE_ACTIVE;
    } else if (mFovControlData.oisSetting == OIS_HOLD) {
        mFovControlResult.oisMode = OIS_MODE_HOLD;
    }

    // Update fallback result if needed
    if (mFovControlData.fallbackEnabled && mFovControlData.fallbackToWide) {
        mFovControlResult.fallback = CAM_WIDE_FALLBACK;
    }

    mFovControlResult.isValid = true;
    // Debug print for the FOV-control result
    LOGD("Effective zoom: %f", zoom);
    LOGD("zoom direction: %s", ((mFovControlData.zoomDirection == ZOOM_STABLE) ? "STABLE" :
            ((mFovControlData.zoomDirection == ZOOM_IN) ? "IN" : "OUT")));
    LOGD("zoomWide: %d, zoomTele: %d", mFovControlData.zoomWide, mFovControlData.zoomTele);
    LOGD("Snapshot postprocess: %d", mFovControlResult.snapshotPostProcess);
    LOGD("Master camera            : %s", (mFovControlResult.camMasterPreview == CAM_TYPE_MAIN) ?
            "CAM_TYPE_MAIN" : "CAM_TYPE_AUX");
    LOGD("Master camera for preview: %s",
            (mFovControlResult.camMasterPreview == camWide ) ? "Wide" : "Tele");
    LOGD("Master camera for 3A     : %s",
            (mFovControlResult.camMaster3A == camWide ) ? "Wide" : "Tele");
    LOGD("Wide camera status : %s",
            (mFovControlResult.activeCameras & camWide) ? "Active" : "LPM");
    LOGD("Tele camera status : %s",
            (mFovControlResult.activeCameras & camTele) ? "Active" : "LPM");
    LOGD("transition state: %s", ((mFovControlData.camState == STATE_WIDE) ? "STATE_WIDE" :
            ((mFovControlData.camState == STATE_TELE) ? "STATE_TELE" : "STATE_TRANSITION" )));
    LOGD("OIS mode: %s", ((mFovControlResult.oisMode == OIS_MODE_ACTIVE) ? "ACTIVE" :
            ((mFovControlResult.oisMode == OIS_MODE_HOLD) ? "HOLD" : "INACTIVE")));
    LOGD("fallback : %d, thermal throttle: %d", mFovControlResult.fallback,
            mFovControlData.thermalThrottle);
}


/*===========================================================================
 * FUNCTION   : getFovControlResult
 *
 * DESCRIPTION: Return FOV-control result
 *
 * PARAMETERS : None
 *
 * RETURN     : FOV-control result
 *
 *==========================================================================*/
 fov_control_result_t QCameraFOVControl::getFovControlResult()
{
    Mutex::Autolock lock(mMutex);
    /* Return either the latest result or the cached result based on the update flag
      set by HAL */
    fov_control_result_t fovControlResult = mFovControlData.updateResultState ?
            mFovControlResult : mFovControlResultCachedCopy;
    return fovControlResult;
}


/*===========================================================================
 * FUNCTION    : isMainCamFovWider
 *
 * DESCRIPTION : Check if the main camera FOV is wider than aux
 *
 * PARAMETERS  : None
 *
 * RETURN      :
 * true        : If main cam FOV is wider than tele
 * false       : If main cam FOV is narrower than tele
 *
 *==========================================================================*/
bool QCameraFOVControl::isMainCamFovWider()
{
    if (mDualCamParams.paramsMain.focalLengthMm <
            mDualCamParams.paramsAux.focalLengthMm) {
        return true;
    } else {
        return false;
    }
}


/*===========================================================================
 * FUNCTION    : needDualZone
 *
 * DESCRIPTION : Check if both the cameras need to be active.
 *
 * PARAMETERS  : None
 *
 * RETURN      :
 * true        : If dual zone is needed
 * false       : If dual zone is not needed
 *
 *==========================================================================*/
bool QCameraFOVControl::needDualZone()
{
    bool ret = false;
    cam_sync_type_t camWide = mFovControlData.camWide;
    cam_sync_type_t camTele = mFovControlData.camTele;
    float zoom = findZoomRatio(mFovControlData.zoomUser) / (float)mFovControlData.zoomRatioTable[0];
    float transitionLow  = mFovControlData.transitionParams.transitionLow;
    float transitionHigh = mFovControlData.transitionParams.transitionHigh;
    float cutoverWideToTele = mFovControlData.transitionParams.cutOverWideToTele;
    float cutoverTeleToWide = mFovControlData.transitionParams.cutOverTeleToWide;

    if (mFovControlResult.snapshotPostProcessZoomRange) {
        ret = true;
    }
    // Return true if Spatial alignment block requested both the cameras to be active or
    // if zoom level dictates so
    else if (((mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_OEM) &&
            (mFovControlData.spatialAlignResult.activeCameras == (camWide | camTele))) ||
            ((zoom >= transitionLow) && (zoom <= transitionHigh)) ||
            (mFovControlData.fallbackEnabled &&
            (((zoom >= cutoverWideToTele) && isMaster(camWide)) ||
            ((zoom <= cutoverTeleToWide) && isMaster(camTele))))) {
        ret = true;
    }

    return ret;
}


/*===========================================================================
 * FUNCTION    : canSwitchMasterTo
 *
 * DESCRIPTION : Check if the master can be switched to the camera- cam.
 *
 * PARAMETERS  :
 * @cam        : cam type
 *
 * RETURN      :
 * true        : If master can be switched
 * false       : If master cannot be switched
 *
 *==========================================================================*/
bool QCameraFOVControl::canSwitchMasterTo(
        uint32_t cam)
{
    bool ret = false;
    float zoom = findZoomRatio(mFovControlData.zoomUser) / (float)mFovControlData.zoomRatioTable[0];
    float cutOverWideToTele = mFovControlData.transitionParams.cutOverWideToTele;
    float cutOverTeleToWide = mFovControlData.transitionParams.cutOverTeleToWide;

    uint16_t thresholdLuxIdx = mFovControlConfig.auxSwitchLuxIdxMax;
    uint16_t thresholdFocusDist = mFovControlConfig.auxSwitchFocusDistCmMin;
    uint16_t currentLuxIdxTele = 0;
    uint16_t currentFocusDistTele = 0;

    if (mFovControlData.camTele == CAM_TYPE_AUX) {
        currentLuxIdxTele     = mFovControlData.status3A.aux.ae.luxIndex;
        currentFocusDistTele  = mFovControlData.status3A.aux.af.focusDistCm;
    } else {
        currentLuxIdxTele     = mFovControlData.status3A.main.ae.luxIndex;
        currentFocusDistTele  = mFovControlData.status3A.main.af.focusDistCm;
    }

    LOGD("Tele: current LuxIdx: %d, threshold LuxIdx: %d", currentLuxIdxTele, thresholdLuxIdx);
    LOGD("Tele: current focus dist: %d, threshold focus dist: %d",
            currentFocusDistTele, thresholdFocusDist);
    LOGD("frameCountWide: %d, frameCountTele: %d",
            mFovControlData.frameCountWide, mFovControlData.frameCountTele);

    if (cam == CAM_ROLE_WIDE) {
        // In case of thermal throttle, only check zoom value for master switch.
        if (mFovControlData.thermalThrottle) {
            if (zoom < cutOverTeleToWide) {
                ret = true;
            }
        } else if (mFovControlData.wideCamStreaming &&
                        (mFovControlData.frameCountWide > FOVC_MIN_FRAME_WAIT_FOR_MASTER_SWITCH)) {
            if (mFovControlData.fallbackEnabled && mFovControlData.fallbackToWide) {
                /* If the fallback is initiated, only switch the master when the spatial alignment
                 confirms the completion of the fallback. */
                if (!(mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_OEM) ||
                        ((mFovControlData.spatialAlignResult.camMasterHint == 0) &&
                        mFovControlData.spatialAlignResult.fallbackComplete)) {
                    ret = true;
                }
            } else if (mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_OEM) {
                // In case of OEM Spatial alignment solution, check the spatial alignment ready
                if (isSpatialAlignmentReady()) {
                    ret = true;
                }
            } else if (zoom < cutOverTeleToWide) {
                // In case of QTI Spatial alignment solution and no spatial alignment solution,
                // check the fallback flag or if the zoom level has crossed the threhold.
                ret = true;
            }
        }
    } else if (cam == CAM_ROLE_TELE) {
        if (mFovControlData.fallbackEnabled && mFovControlData.fallbackToWide) {
            // If the fallback to wide is initiated, don't switch the master to tele
            ret = false;
        } else if (mFovControlData.thermalThrottle) {
            // In case of thermal throttle, only check zoom value for master switch.
            if (zoom > cutOverWideToTele) {
                ret = true;
            }
        } else if (mFovControlData.teleCamStreaming &&
                        (mFovControlData.frameCountTele > FOVC_MIN_FRAME_WAIT_FOR_MASTER_SWITCH)) {
            bool teleWellLitNonMacroScene = !mFovControlData.fallbackEnabled ||
                                                (currentLuxIdxTele <= thresholdLuxIdx);
            if (mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_OEM) {
                // In case of OEM Spatial alignment solution, check the spatial alignment ready
                if (isSpatialAlignmentReady() &&
                        teleWellLitNonMacroScene) {
                    ret = true;
                }
            } else if (mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_QTI) {
                // In case of QTI Spatial alignment solution check the spatial alignment ready flag
                // and if the zoom level has crossed the threhold.
                if (isSpatialAlignmentReady() &&
                        (zoom > cutOverWideToTele) &&
                        teleWellLitNonMacroScene) {
                    ret = true;
                }
            } else {
                // In case of no spatial alignment solution check if
                // the zoom level has crossed the threhold.
                if ((zoom > cutOverWideToTele) &&
                        teleWellLitNonMacroScene) {
                    ret = true;
                }
            }
        }
    } else {
        LOGE("Request to switch to invalid cam type");
    }
    return ret;
}

/*===========================================================================
 * FUNCTION    : isSpatialAlignmentReady
 *
 * DESCRIPTION : Check if the spatial alignment is ready.
 *               For QTI solution, check ready_status flag
 *               For OEM solution, check camMasterHint
 *               If the spatial alignment solution is not needed, return true
 *
 * PARAMETERS  : None
 *
 * RETURN      :
 * true        : If spatial alignment is ready
 * false       : If spatial alignment is not yet ready
 *
 *==========================================================================*/
bool QCameraFOVControl::isSpatialAlignmentReady()
{
    bool ret = true;
    cam_sync_type_t camWide = mFovControlData.camWide;
    cam_sync_type_t camTele = mFovControlData.camTele;

    if (mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_OEM) {
        uint8_t currentMaster = (uint8_t)mFovControlResult.camMasterPreview;
        uint8_t camMasterHint = mFovControlData.spatialAlignResult.camMasterHint;
        LOGD("current master: %d, recommended master: %d", currentMaster, camMasterHint);

        if (((currentMaster == camWide) && (camMasterHint == camTele)) ||
                ((currentMaster == camTele) && (camMasterHint == camWide))){
            ret = true;
        } else {
            ret = false;
        }
    } else if (mFovControlData.availableSpatialAlignSolns & CAM_SPATIAL_ALIGN_QTI) {
        if (mFovControlData.spatialAlignResult.readyStatus) {
            ret = true;
        } else {
            ret = false;
        }
    }

    return ret;
}


/*===========================================================================
 * FUNCTION    : isMaster
 *
 * DESCRIPTION : Check if the given camera is master.
 *
 * PARAMETERS  :
 *  @cam       : input camera for master check
 *
 * RETURN      : Boolean indicating if the given camera is master
 *
 *==========================================================================*/
inline bool QCameraFOVControl::isMaster(
        cam_sync_type_t cam)
{
    if ((mFovControlResult.camMasterPreview == cam) &&
            (mFovControlResult.camMaster3A == cam)) {
        return true;
    } else {
        return false;
    }
}


/*===========================================================================
 * FUNCTION    : isTimedOut
 *
 * DESCRIPTION : Check if the timer is timed out.
 *
 * PARAMETERS  :
 *  @timer     : Timer to check the timeout.
 *
 * RETURN      : Boolean indicating if timed out
 *
 *==========================================================================*/
bool QCameraFOVControl::isTimedOut(
        timer_t timer)
{
    if (timer.active && (systemTime() > timer.timeout)) {
        return true;
    }
    return false;
}


/*===========================================================================
 * FUNCTION    : startTimer
 *
 * DESCRIPTION : Start the timer for the duration specified.
 *
 * PARAMETERS  :
 *  @timer     : Timer to start.
 *  @time      : Time duration in ms.
 *
 * RETURN      : void
 *
 *==========================================================================*/
void QCameraFOVControl::startTimer(
        timer_t  *timer,
        uint32_t  time)
{
    if (time > 0) {
        timer->timeout = systemTime() + ms2ns(time);
        timer->active = true;
    }
}


/*===========================================================================
 * FUNCTION    : inactivateTimer
 *
 * DESCRIPTION : Stop the timer and set the 'active' flag to false.
 *
 * PARAMETERS  :
 *  @timer     : Timer to stop.
 *
 * RETURN      : void
 *
 *==========================================================================*/
void QCameraFOVControl::inactivateTimer(
        timer_t  *timer)
{
    timer->active = false;
}

/*===========================================================================
 * FUNCTION    : validateAndExtractParameters
 *
 * DESCRIPTION : Validates a subset of parameters from capabilities and
 *               saves those parameters for decision making.
 *
 * PARAMETERS  :
 *  @capsMain  : The capabilities for the main camera
 *  @capsAux   : The capabilities for the aux camera
 *
 * RETURN      :
 * true        : Success
 * false       : Failure
 *
 *==========================================================================*/
bool QCameraFOVControl::validateAndExtractParameters(
        cam_capability_t  *capsMainCam,
        cam_capability_t  *capsAuxCam)
{
    bool rc = false;
    if (capsMainCam && capsAuxCam) {
        char propSAC[PROPERTY_VALUE_MAX];
        char propSAT[PROPERTY_VALUE_MAX];

        property_get("persist.vendor.camera.sac.enable", propSAC, "0");
        property_get("persist.vendor.camera.sat.enable", propSAT, "0");

        mFovControlConfig.percentMarginHysterisis  = 5;
        mFovControlConfig.percentMarginMain        = 25;
        mFovControlConfig.percentMarginAux         = 15;

        mDualCamParams.paramsMain.focalLengthMm = capsMainCam->focal_length;
        mDualCamParams.paramsAux.focalLengthMm  = capsAuxCam->focal_length;

        mDualCamParams.paramsMain.pixelPitchUm = capsMainCam->pixel_pitch_um;
        mDualCamParams.paramsAux.pixelPitchUm  = capsAuxCam->pixel_pitch_um;

        if (capsMainCam->supported_focus_modes_cnt > 1)
            mDualCamParams.paramsMain.isAFSupported = true;
        if (capsAuxCam->supported_focus_modes_cnt > 1)
            mDualCamParams.paramsAux.isAFSupported = true;

        if (((capsMainCam->avail_spatial_align_solns & CAM_SPATIAL_ALIGN_QTI) && atoi(propSAC)) ||
                ((capsMainCam->avail_spatial_align_solns & CAM_SPATIAL_ALIGN_OEM) &&
                        atoi(propSAT))) {
            mFovControlData.availableSpatialAlignSolns =
                    capsMainCam->avail_spatial_align_solns;
        } else {
            LOGW("Spatial alignment not supported");
        }

        if (capsMainCam->zoom_supported > 0) {
            mFovControlData.zoomRatioTable      = capsMainCam->zoom_ratio_tbl;
            mFovControlData.zoomRatioTableCount = capsMainCam->zoom_ratio_tbl_cnt;
        } else {
            LOGE("zoom feature not supported");
            return false;
        }
        rc = true;
    }

    return rc;
}

/*===========================================================================
 * FUNCTION   : calculateBasicFovRatio
 *
 * DESCRIPTION: Calculate the FOV ratio between main and aux cameras
 *
 * PARAMETERS : None
 *
 * RETURN     :
 * true       : Success
 * false      : Failure
 *
 *==========================================================================*/
bool QCameraFOVControl::calculateBasicFovRatio()
{
    float fovWide = 0.0f;
    float fovTele = 0.0f;
    bool rc = false;

    if ((mDualCamParams.paramsMain.focalLengthMm > 0.0f) &&
         (mDualCamParams.paramsAux.focalLengthMm > 0.0f)) {
        if (mDualCamParams.paramsMain.focalLengthMm <
            mDualCamParams.paramsAux.focalLengthMm) {
            fovWide = (mDualCamParams.paramsMain.sensorStreamWidth *
                        mDualCamParams.paramsMain.pixelPitchUm) /
                        mDualCamParams.paramsMain.focalLengthMm;

            fovTele = (mDualCamParams.paramsAux.sensorStreamWidth *
                        mDualCamParams.paramsAux.pixelPitchUm) /
                        mDualCamParams.paramsAux.focalLengthMm;
        } else {
            fovWide = (mDualCamParams.paramsAux.sensorStreamWidth *
                        mDualCamParams.paramsAux.pixelPitchUm) /
                        mDualCamParams.paramsAux.focalLengthMm;

            fovTele = (mDualCamParams.paramsMain.sensorStreamWidth *
                        mDualCamParams.paramsMain.pixelPitchUm) /
                        mDualCamParams.paramsMain.focalLengthMm;
        }
        if (fovTele > 0.0f) {
            mFovControlData.basicFovRatio = (fovWide / fovTele);
            rc = true;
        }
    }

    LOGD("Main cam focalLengthMm : %f", mDualCamParams.paramsMain.focalLengthMm);
    LOGD("Aux  cam focalLengthMm : %f", mDualCamParams.paramsAux.focalLengthMm);
    LOGD("Main cam sensorStreamWidth : %u", mDualCamParams.paramsMain.sensorStreamWidth);
    LOGD("Main cam sensorStreamHeight: %u", mDualCamParams.paramsMain.sensorStreamHeight);
    LOGD("Aux cam sensorStreamWidth  : %u", mDualCamParams.paramsAux.sensorStreamWidth);
    LOGD("Aux cam sensorStreamHeight : %u", mDualCamParams.paramsAux.sensorStreamHeight);
    LOGD("Main cam pixelPitchUm      : %f", mDualCamParams.paramsMain.pixelPitchUm);
    LOGD("Aux cam pixelPitchUm       : %f", mDualCamParams.paramsAux.pixelPitchUm);
    LOGD("fov wide : %f", fovWide);
    LOGD("fov tele : %f", fovTele);
    LOGD("BasicFovRatio : %f", mFovControlData.basicFovRatio);

    return rc;
}


/*===========================================================================
 * FUNCTION   : combineFovAdjustment
 *
 * DESCRIPTION: Calculate the final FOV adjustment by combining basic FOV ratio
 *              with the margin info
 *
 * PARAMETERS : None
 *
 * RETURN     :
 * true       : Success
 * false      : Failure
 *
 *==========================================================================*/
bool QCameraFOVControl::combineFovAdjustment()
{
    float ratioMarginWidth;
    float ratioMarginHeight;
    float adjustedRatio;
    bool rc = false;

    ratioMarginWidth = (1.0 + (mFovControlData.camMainWidthMargin)) /
            (1.0 + (mFovControlData.camAuxWidthMargin));
    ratioMarginHeight = (1.0 + (mFovControlData.camMainHeightMargin)) /
            (1.0 + (mFovControlData.camAuxHeightMargin));

    adjustedRatio = (ratioMarginHeight < ratioMarginWidth) ? ratioMarginHeight : ratioMarginWidth;

    if (adjustedRatio > 0.0f) {
        mFovControlData.transitionParams.cutOverFactor =
                (mFovControlData.basicFovRatio / adjustedRatio);
        rc = true;
    }

    LOGD("Main cam margin for width  : %f", mFovControlData.camMainWidthMargin);
    LOGD("Main cam margin for height : %f", mFovControlData.camMainHeightMargin);
    LOGD("Aux  cam margin for width  : %f", mFovControlData.camAuxWidthMargin);
    LOGD("Aux  cam margin for height : %f", mFovControlData.camAuxHeightMargin);
    LOGD("Width  margin ratio : %f", ratioMarginWidth);
    LOGD("Height margin ratio : %f", ratioMarginHeight);

    return rc;
}


/*===========================================================================
 * FUNCTION   : calculateDualCamTransitionParams
 *
 * DESCRIPTION: Calculate the transition parameters needed to switch the camera
 *              between main and aux
 *
 * PARAMETERS :
 * @fovAdjustBasic       : basic FOV ratio
 * @zoomTranslationFactor: translation factor for main, aux zoom
 *
 * RETURN     : none
 *
 *==========================================================================*/
void QCameraFOVControl::calculateDualCamTransitionParams()
{
    float percentMarginWide;
    float percentMarginTele;

    if (isMainCamFovWider()) {
        percentMarginWide = mFovControlConfig.percentMarginMain;
        percentMarginTele = mFovControlConfig.percentMarginAux;
    } else {
        percentMarginWide = mFovControlConfig.percentMarginAux;
        percentMarginTele = mFovControlConfig.percentMarginMain;
    }

    mFovControlData.transitionParams.cropRatio = mFovControlData.basicFovRatio;

    mFovControlData.transitionParams.cutOverWideToTele =
            mFovControlData.transitionParams.cutOverFactor +
            (mFovControlConfig.percentMarginHysterisis / 100.0) * mFovControlData.basicFovRatio;

    mFovControlData.transitionParams.cutOverTeleToWide =
            mFovControlData.transitionParams.cutOverFactor;

    mFovControlData.transitionParams.transitionHigh =
            mFovControlData.transitionParams.cutOverWideToTele +
            (percentMarginWide / 100.0) * mFovControlData.basicFovRatio;

    mFovControlData.transitionParams.transitionLow =
            mFovControlData.transitionParams.cutOverTeleToWide -
            (percentMarginTele / 100.0) * mFovControlData.basicFovRatio;

    if (mFovControlConfig.snapshotPPConfig.enablePostProcess) {
        // Expand the transition zone if necessary to account for
        // the snapshot post-process settings
        if (mFovControlConfig.snapshotPPConfig.zoomMax >
                mFovControlData.transitionParams.transitionHigh) {
            mFovControlData.transitionParams.transitionHigh =
                mFovControlConfig.snapshotPPConfig.zoomMax;
        }
        if (mFovControlConfig.snapshotPPConfig.zoomMin <
                mFovControlData.transitionParams.transitionLow) {
            mFovControlData.transitionParams.transitionLow =
                mFovControlConfig.snapshotPPConfig.zoomMin;
        }
    }

    LOGD("transition param: TransitionLow  %f", mFovControlData.transitionParams.transitionLow);
    LOGD("transition param: TeleToWide     %f", mFovControlData.transitionParams.cutOverTeleToWide);
    LOGD("transition param: WideToTele     %f", mFovControlData.transitionParams.cutOverWideToTele);
    LOGD("transition param: TransitionHigh %f", mFovControlData.transitionParams.transitionHigh);
}


/*===========================================================================
 * FUNCTION   : findZoomValue
 *
 * DESCRIPTION: For the input zoom ratio, find the zoom value.
 *              Zoom table contains zoom ratios where the indices
 *              in the zoom table indicate the corresponding zoom values.
 * PARAMETERS :
 * @zoomRatio : Zoom ratio
 *
 * RETURN     : Zoom value
 *
 *==========================================================================*/
uint32_t QCameraFOVControl::findZoomValue(
        uint32_t zoomRatio)
{
    uint32_t zoom = 0;
    for (uint32_t i = 0; i < mFovControlData.zoomRatioTableCount; ++i) {
        if (zoomRatio <= mFovControlData.zoomRatioTable[i]) {
            zoom = i;
            break;
        }
    }
    return zoom;
}


/*===========================================================================
 * FUNCTION   : findZoomRatio
 *
 * DESCRIPTION: For the input zoom value, find the zoom ratio.
 *              Zoom table contains zoom ratios where the indices
 *              in the zoom table indicate the corresponding zoom values.
 * PARAMETERS :
 * @zoom      : zoom value
 *
 * RETURN     : zoom ratio
 *
 *==========================================================================*/
uint32_t QCameraFOVControl::findZoomRatio(
        uint32_t zoom)
{
    return mFovControlData.zoomRatioTable[zoom];
}


/*===========================================================================
 * FUNCTION   : readjustZoomForTele
 *
 * DESCRIPTION: Calculate the zoom value for the tele camera based on zoom value
 *              for the wide camera
 *
 * PARAMETERS :
 * @zoomWide  : Zoom value for wide camera
 *
 * RETURN     : Zoom value for tele camera
 *
 *==========================================================================*/
uint32_t QCameraFOVControl::readjustZoomForTele(
        uint32_t zoomWide)
{
    uint32_t zoomRatioWide;
    uint32_t zoomRatioTele;

    zoomRatioWide = findZoomRatio(zoomWide);
    zoomRatioTele  = zoomRatioWide / mFovControlData.transitionParams.cutOverFactor;

    mFovControlData.zoomRatioWide = zoomRatioWide;
    mFovControlData.zoomRatioTele = zoomRatioTele;

    return(findZoomValue(zoomRatioTele));
}


/*===========================================================================
 * FUNCTION   : readjustZoomForWide
 *
 * DESCRIPTION: Calculate the zoom value for the wide camera based on zoom value
 *              for the tele camera
 *
 * PARAMETERS :
 * @zoomTele  : Zoom value for tele camera
 *
 * RETURN     : Zoom value for wide camera
 *
 *==========================================================================*/
uint32_t QCameraFOVControl::readjustZoomForWide(
        uint32_t zoomTele)
{
    uint32_t zoomRatioWide;
    uint32_t zoomRatioTele;

    zoomRatioTele = findZoomRatio(zoomTele);
    zoomRatioWide = zoomRatioTele * mFovControlData.transitionParams.cutOverFactor;

    mFovControlData.zoomRatioWide = zoomRatioWide;
    mFovControlData.zoomRatioTele = zoomRatioTele;

    return(findZoomValue(zoomRatioWide));
}


/*===========================================================================
 * FUNCTION   : convertUserZoomToWideAndTele
 *
 * DESCRIPTION: Calculate the zoom value for the wide and tele cameras
 *              based on the input user zoom value
 *
 * PARAMETERS :
 * @zoom      : User zoom value
 *
 * RETURN     : none
 *
 *==========================================================================*/
void QCameraFOVControl::convertUserZoomToWideAndTele(
        uint32_t zoom)
{
    Mutex::Autolock lock(mMutex);

    mFovControlData.zoomUser = zoom;

    // If the zoom translation library is present and initialized,
    // use it to get wide and tele zoom values
    if (mZoomTranslator && mZoomTranslator->isInitialized()) {
        zoom_data zoomData;
        if (mZoomTranslator->getZoomValues(zoom, &zoomData) != NO_ERROR) {
            LOGE("getZoomValues failed from zoom translation lib");
            // Use zoom translation logic from FOV-control
            mFovControlData.zoomWide = zoom;
            mFovControlData.zoomTele = readjustZoomForTele(mFovControlData.zoomWide);
        } else {
            // Use the zoom values provided by zoom translation lib
            mFovControlData.zoomWideIsp = zoomData.zoomWideIsp;
            mFovControlData.zoomTeleIsp = zoomData.zoomTeleIsp;
            mFovControlData.zoomWide    = zoomData.zoomWideTotal;
            mFovControlData.zoomTele    = zoomData.zoomTeleTotal;
        }
    } else {
        if (mHalPPType == CAM_HAL_PP_TYPE_BOKEH) {
            mFovControlData.zoomTele = zoom;
            mFovControlData.zoomWide = readjustZoomForWide(mFovControlData.zoomTele);
        } else {
            mFovControlData.zoomWide = zoom;
            mFovControlData.zoomTele = readjustZoomForTele(mFovControlData.zoomWide);
        }
        mFovControlData.zoomWideIsp = mFovControlData.zoomWide;
        mFovControlData.zoomTeleIsp = mFovControlData.zoomTele;
    }
}


/*===========================================================================
 * FUNCTION   : translateFocusAreas
 *
 * DESCRIPTION: Translate the focus areas from main to aux camera.
 *
 * PARAMETERS :
 * @roiAfMain : Focus area ROI for main camera
 * @cam       : Cam type
 *
 * RETURN     : Translated focus area ROI for aux camera
 *
 *==========================================================================*/
cam_roi_info_t QCameraFOVControl::translateFocusAreas(
        cam_roi_info_t  roiAfMain,
        cam_sync_type_t cam)
{
    float fovRatio;
    float zoomWide;
    float zoomTele;
    float AuxRoiLeft;
    float AuxRoiTop;
    cam_roi_info_t roiAfTrans = roiAfMain;
    int32_t shiftHorzAdjusted;
    int32_t shiftVertAdjusted;

    zoomWide = findZoomRatio(mFovControlData.zoomWide) / (float)mFovControlData.zoomRatioTable[0];
    zoomTele = findZoomRatio(mFovControlData.zoomTele) / (float)mFovControlData.zoomRatioTable[0];

    if (cam == mFovControlData.camWide) {
        fovRatio = (mHalPPType == CAM_HAL_PP_TYPE_BOKEH) ?
                        (1.0f / mFovControlData.transitionParams.cropRatio) : 1.0f;
    } else {
        fovRatio = (zoomTele / zoomWide) * mFovControlData.transitionParams.cropRatio;
    }

    // Acquire the mutex in order to read the spatial alignment result which is written
    // by another thread
    mMutex.lock();
    if (cam == mFovControlData.camWide) {
        shiftHorzAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftHorz * zoomWide;
        shiftVertAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftVert * zoomWide;
    } else {
        shiftHorzAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftHorz * zoomTele;
        shiftVertAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftVert * zoomTele;
    }
    mMutex.unlock();

    for (int i = 0; i < roiAfMain.num_roi; ++i) {
        roiAfTrans.roi[i].width  = roiAfMain.roi[i].width * fovRatio;
        roiAfTrans.roi[i].height = roiAfMain.roi[i].height * fovRatio;

        AuxRoiTop = ((roiAfMain.roi[i].top - mFovControlData.previewSize.height / 2.0f) * fovRatio)
                        + (mFovControlData.previewSize.height / 2.0f);
        roiAfTrans.roi[i].top =  AuxRoiTop - shiftVertAdjusted;
        AuxRoiLeft = ((roiAfMain.roi[i].left - mFovControlData.previewSize.width / 2.0f) * fovRatio)
                        + (mFovControlData.previewSize.width / 2.0f);
        roiAfTrans.roi[i].left = AuxRoiLeft - shiftHorzAdjusted;

        // Check the ROI bounds and correct if necessory
        if ((roiAfTrans.roi[i].width >= mFovControlData.previewSize.width) ||
            (roiAfTrans.roi[i].height >= mFovControlData.previewSize.height)) {
            roiAfTrans = roiAfMain;
            LOGW("AF ROI translation failed, reverting to the pre-translation ROI");
        } else {
            bool error = false;
            if (roiAfTrans.roi[i].left < 0) {
                roiAfTrans.roi[i].left = 0;
                error = true;
            }
            if (roiAfTrans.roi[i].top < 0) {
                roiAfTrans.roi[i].top = 0;
                error = true;
            }
            if ((roiAfTrans.roi[i].left >= mFovControlData.previewSize.width) ||
                ((roiAfTrans.roi[i].left + roiAfTrans.roi[i].width) >
                        mFovControlData.previewSize.width)) {
                roiAfTrans.roi[i].left = mFovControlData.previewSize.width -
                                                roiAfTrans.roi[i].width;
                error = true;
            }
            if ((roiAfTrans.roi[i].top >= mFovControlData.previewSize.height) ||
                ((roiAfTrans.roi[i].top + roiAfTrans.roi[i].height) >
                        mFovControlData.previewSize.height)) {
                roiAfTrans.roi[i].top = mFovControlData.previewSize.height -
                                                roiAfTrans.roi[i].height;
                error = true;
            }
            if (error) {
                LOGW("Translated ROI - out of bounds. Clamping to the nearest valid ROI");
            }
        }

        LOGD("Translated AF ROI-%d %s: L:%d, T:%d, W:%d, H:%d", i,
                (cam == CAM_TYPE_MAIN) ? "main cam" : "aux  cam", roiAfTrans.roi[i].left,
                roiAfTrans.roi[i].top, roiAfTrans.roi[i].width, roiAfTrans.roi[i].height);
    }
    return roiAfTrans;
}


/*===========================================================================
 * FUNCTION   : translateMeteringAreas
 *
 * DESCRIPTION: Translate the AEC metering areas from main to aux camera.
 *
 * PARAMETERS :
 * @roiAfMain : AEC ROI for main camera
 * @cam       : Cam type
 *
 * RETURN     : Translated AEC ROI for aux camera
 *
 *==========================================================================*/
cam_set_aec_roi_t QCameraFOVControl::translateMeteringAreas(
        cam_set_aec_roi_t roiAecMain,
        cam_sync_type_t cam)
{
    float fovRatio;
    float zoomWide;
    float zoomTele;
    float AuxDiffRoiX;
    float AuxDiffRoiY;
    float AuxRoiX;
    float AuxRoiY;
    cam_set_aec_roi_t roiAecTrans = roiAecMain;
    int32_t shiftHorzAdjusted;
    int32_t shiftVertAdjusted;

    zoomWide = findZoomRatio(mFovControlData.zoomWide) / (float)mFovControlData.zoomRatioTable[0];
    zoomTele = findZoomRatio(mFovControlData.zoomTele) / (float)mFovControlData.zoomRatioTable[0];

    if (cam == mFovControlData.camWide) {
        fovRatio = (mHalPPType == CAM_HAL_PP_TYPE_BOKEH) ?
                        (1.0f / mFovControlData.transitionParams.cropRatio) : 1.0f;
    } else {
        fovRatio = (zoomTele / zoomWide) * mFovControlData.transitionParams.cropRatio;
    }

    // Acquire the mutex in order to read the spatial alignment result which is written
    // by another thread
    mMutex.lock();
    if (cam == mFovControlData.camWide) {
        shiftHorzAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftHorz * zoomWide;
        shiftVertAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftVert * zoomWide;
    } else {
        shiftHorzAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftHorz * zoomTele;
        shiftVertAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftVert * zoomTele;
    }
    mMutex.unlock();

    for (int i = 0; i < roiAecMain.num_roi; ++i) {
        AuxDiffRoiX = fovRatio * ((float)roiAecMain.cam_aec_roi_position.coordinate[i].x -
                          (mFovControlData.previewSize.width / 2));
        AuxRoiX = (mFovControlData.previewSize.width / 2) + AuxDiffRoiX;

        AuxDiffRoiY = fovRatio * ((float)roiAecMain.cam_aec_roi_position.coordinate[i].y -
                          (mFovControlData.previewSize.height / 2));
        AuxRoiY = (mFovControlData.previewSize.height / 2) + AuxDiffRoiY;

        roiAecTrans.cam_aec_roi_position.coordinate[i].x = AuxRoiX - shiftHorzAdjusted;
        roiAecTrans.cam_aec_roi_position.coordinate[i].y = AuxRoiY - shiftVertAdjusted;

        // Check the ROI bounds and correct if necessory
        if ((AuxRoiX < 0) ||
            (AuxRoiY < 0)) {
            roiAecTrans.cam_aec_roi_position.coordinate[i].x = 0;
            roiAecTrans.cam_aec_roi_position.coordinate[i].y = 0;
            LOGW("AEC ROI translation failed");
        } else if ((AuxRoiX >= mFovControlData.previewSize.width) ||
            (AuxRoiY >= mFovControlData.previewSize.height)) {
            // Clamp the Aux AEC ROI co-ordinates to max possible value
            if (AuxRoiX >= mFovControlData.previewSize.width) {
                roiAecTrans.cam_aec_roi_position.coordinate[i].x =
                        mFovControlData.previewSize.width - 1;
            }
            if (AuxRoiY >= mFovControlData.previewSize.height) {
                roiAecTrans.cam_aec_roi_position.coordinate[i].y =
                        mFovControlData.previewSize.height - 1;
            }
            LOGW("AEC ROI translation failed");
        }

        LOGD("Translated AEC ROI-%d %s: x:%d, y:%d", i,
                (cam == CAM_TYPE_MAIN) ? "main cam" : "aux  cam",
                roiAecTrans.cam_aec_roi_position.coordinate[i].x,
                roiAecTrans.cam_aec_roi_position.coordinate[i].y);
    }
    return roiAecTrans;
}


/*===========================================================================
 * FUNCTION   : translateRoiFD
 *
 * DESCRIPTION: Translate face detection ROI from aux metadata to main
 *
 * PARAMETERS :
 * @faceDetectionInfo  : face detection data from aux metadata. This face
 *                       detection data is overwritten with the translated
 *                       FD ROI.
 * @cam                : Cam type
 *
 * RETURN     : none
 *
 *==========================================================================*/
cam_face_detection_data_t QCameraFOVControl::translateRoiFD(
        cam_face_detection_data_t metaFD,
        cam_sync_type_t cam)
{

    if (mbIsHAL3) {
        return translateHAL3FDRoi(metaFD, cam);
    }

    cam_face_detection_data_t metaFDTranslated = metaFD;
    int32_t shiftHorz = 0;
    int32_t shiftVert = 0;

    float zoomWide = findZoomRatio(mFovControlData.zoomWide) /
                        (float)mFovControlData.zoomRatioTable[0];
    float zoomTele = findZoomRatio(mFovControlData.zoomTele) /
                        (float)mFovControlData.zoomRatioTable[0];

    if (cam == mFovControlData.camWide) {
        shiftHorz = mFovControlData.spatialAlignResult.shiftWide.shiftHorz * zoomWide;
        shiftVert = mFovControlData.spatialAlignResult.shiftWide.shiftVert * zoomWide;
    } else {
        shiftHorz = mFovControlData.spatialAlignResult.shiftTele.shiftHorz * zoomTele;
        shiftVert = mFovControlData.spatialAlignResult.shiftTele.shiftVert * zoomTele;
    }

    for (int i = 0; i < metaFDTranslated.num_faces_detected; ++i) {
        metaFDTranslated.faces[i].face_boundary.left += shiftHorz;
        metaFDTranslated.faces[i].face_boundary.top  += shiftVert;
    }

    // If ROI is out of bounds, remove that FD ROI from the list
    for (int i = 0; i < metaFDTranslated.num_faces_detected; ++i) {
        if ((metaFDTranslated.faces[i].face_boundary.left < 0) ||
            (metaFDTranslated.faces[i].face_boundary.left >= mFovControlData.previewSize.width) ||
            (metaFDTranslated.faces[i].face_boundary.top < 0) ||
            (metaFDTranslated.faces[i].face_boundary.top >= mFovControlData.previewSize.height) ||
            ((metaFDTranslated.faces[i].face_boundary.left +
                    metaFDTranslated.faces[i].face_boundary.width) >=
                    mFovControlData.previewSize.width) ||
            ((metaFDTranslated.faces[i].face_boundary.top +
                    metaFDTranslated.faces[i].face_boundary.height) >=
                    mFovControlData.previewSize.height)) {
            // Invalid FD ROI detected
            LOGW("Failed translating FD ROI %s: L:%d, T:%d, W:%d, H:%d",
                    (cam == CAM_TYPE_MAIN) ? "main cam" : "aux  cam",
                    metaFDTranslated.faces[i].face_boundary.left,
                    metaFDTranslated.faces[i].face_boundary.top,
                    metaFDTranslated.faces[i].face_boundary.width,
                    metaFDTranslated.faces[i].face_boundary.height);

            // Remove it by copying the last FD ROI at this index
            if (i < (metaFDTranslated.num_faces_detected - 1)) {
                metaFDTranslated.faces[i] =
                        metaFDTranslated.faces[metaFDTranslated.num_faces_detected - 1];
                // Decrement the current index to process the newly copied FD ROI.
                --i;
            }
            --metaFDTranslated.num_faces_detected;
        }
        else {
            LOGD("Translated FD ROI-%d %s: L:%d, T:%d, W:%d, H:%d", i,
                    (cam == CAM_TYPE_MAIN) ? "main cam" : "aux  cam",
                    metaFDTranslated.faces[i].face_boundary.left,
                    metaFDTranslated.faces[i].face_boundary.top,
                    metaFDTranslated.faces[i].face_boundary.width,
                    metaFDTranslated.faces[i].face_boundary.height);
        }
    }
    return metaFDTranslated;
}

/*===========================================================================
 * FUNCTION      : getFrameMargins
 *
 * DESCRIPTION   : Return frame margin data for the requested camera
 *
 * PARAMETERS    :
 * @masterCamera : Master camera id
 *
 * RETURN        : Frame margins
 *
 *==========================================================================*/
cam_frame_margins_t QCameraFOVControl::getFrameMargins(
        int8_t masterCamera)
{
    cam_frame_margins_t frameMargins;
    memset(&frameMargins, 0, sizeof(cam_frame_margins_t));

    if (masterCamera == CAM_TYPE_MAIN) {
        frameMargins.widthMargins  = mFovControlData.camMainWidthMargin;
        frameMargins.heightMargins = mFovControlData.camMainHeightMargin;
    } else if (masterCamera == CAM_TYPE_AUX) {
        frameMargins.widthMargins  = mFovControlData.camAuxWidthMargin;
        frameMargins.heightMargins = mFovControlData.camAuxHeightMargin;
    }

    return frameMargins;
}


/*===========================================================================
 * FUNCTION      : setHalPPType
 *
 * DESCRIPTION   : set the mode in which dual camera is performing
 *
 * PARAMETERS    :
 * @halPPType    : HAL post processing type for current dual camera mode
 *
 * RETURN        : NONE
 *
 *==========================================================================*/
void QCameraFOVControl::setHalPPType(cam_hal_pp_type_t halPPType)
{
    mHalPPType = halPPType;
    LOGH("halPPType: %d", halPPType);
    return;
}

/*===========================================================================
 * FUNCTION      : UpdateFlag
 *
 * DESCRIPTION   : Update FOV-control flag and run the state machine to reflect
 *                 the effect of the flag
 *
 * PARAMETERS    :
 * @flag         : Flag to update
 * @value        : value to update
 *
 * RETURN        : void
 *
 *==========================================================================*/
void QCameraFOVControl::UpdateFlag(
        fov_control_flag  flag,
        void             *value)
{
    if ((flag >= 0) && (flag < FOVCONTROL_FLAG_COUNT)) {
        mMutex.lock();
        switch (flag) {
            case FOVCONTROL_FLAG_FORCE_CAMERA_WAKEUP: {
                bool forceCameraWakeup = *(bool*)value;
                if (forceCameraWakeup) {
                    mFovControlData.forceCameraWakeup = true;
                } else {
                    mFovControlData.forceCameraWakeup = false;
                }
                break;
            }
            case FOVCONTROL_FLAG_THERMAL_THROTTLE:
                mFovControlData.thermalThrottle = *(bool*)value;
                break;
            case FOVCONTROL_FLAG_UPDATE_RESULT_STATE:
                if (mFovControlData.updateResultState == *(bool*)value) {
                    mMutex.unlock();
                    return;
                } else {
                    mFovControlData.updateResultState = *(bool*)value;
                    // Cache the latest result if the update flag is set to false
                    if (mFovControlData.updateResultState == false) {
                        memcpy(&mFovControlResultCachedCopy, &mFovControlResult,
                                sizeof(fov_control_result_t));
                    }
                }
                break;
            default:
                LOGE("Invalid event to process");
                break;
        }
        mMutex.unlock();
        LOGH("FOV-Control received flag %d with value %d", flag, *(bool*)value);
        generateFovControlResult();
    }
}

/*===========================================================================
* FUNCTION   : setDualCameraConfig
*
* DESCRIPTION: set dual camera configuration whether B+M/W+T
*
* PARAMETERS : input of type dual_cam_type
*
* RETURN    : none
*==========================================================================*/
void QCameraFOVControl::setDualCameraConfig(uint8_t type)
{
    mDualCamType = type;
}

void QCameraFOVControl::setZoomParam(uint8_t cam_type, cam_zoom_info_t zoomInfo, uint32_t zoomTotal,
        uint32_t zoomIsp, bool snapshotPostProcess, parm_buffer_t* params, bool isHAL3)
{
    for (uint32_t i = 0; i < zoomInfo.num_streams; ++i) {
        zoomInfo.stream_zoom_info[i].stream_type = mFovControlData.camStreamInfo.type[i];
        zoomInfo.stream_zoom_info[i].stream_zoom = zoomTotal;
        zoomInfo.stream_zoom_info[i].isp_zoom    = zoomIsp;

        // If the snapshot post-processing is enabled, disable isp zoom
        if (snapshotPostProcess &&
                (zoomInfo.stream_zoom_info[i].stream_type == CAM_STREAM_TYPE_SNAPSHOT)) {
            zoomInfo.stream_zoom_info[i].isp_zoom = 0;
        }

        LOGD("cam[%d]: stream_type: %d, stream_zoom: %d, isp_zoom: %d",
            cam_type, zoomInfo.stream_zoom_info[i].stream_type,
            zoomInfo.stream_zoom_info[i].stream_zoom,
            zoomInfo.stream_zoom_info[i].isp_zoom);
    }
    if (isHAL3) {
        ADD_SET_PARAM_ENTRY_TO_BATCH(params, CAM_INTF_META_USERZOOM, zoomInfo);
    } else {
        ADD_SET_PARAM_ENTRY_TO_BATCH(params, CAM_INTF_PARM_USERZOOM, zoomInfo);
    }
}

void QCameraFOVControl::setCropParam(uint8_t cam_type, uint32_t zoomStep, parm_buffer_t* params)
{
    uint32_t sensorW, sensorH;
    cam_crop_region_t scalerCropRegion;
    float zoomRatio = zoomStep/100.0;
    if (zoomRatio < 1.0) zoomRatio = 1.0;
    // Adjust crop region from main sensor output coordinate system to aux
    // array coordinate system.

    if (cam_type == CAM_TYPE_MAIN) {
        sensorW = mDualCamParams.paramsMain.sensorStreamWidth;
        sensorH = mDualCamParams.paramsMain.sensorStreamHeight;
    } else {
        sensorW = mDualCamParams.paramsAux.sensorStreamWidth;
        sensorH = mDualCamParams.paramsAux.sensorStreamHeight;
    }

    uint32_t xCenter = sensorW / 2;
    uint32_t yCenter = sensorH / 2;
    uint32_t xDelta = (uint32_t) (sensorW / (2 * zoomRatio));
    uint32_t yDelta = (uint32_t) (sensorH / (2 * zoomRatio));

    scalerCropRegion.left = xCenter - xDelta;
    scalerCropRegion.top = yCenter - yDelta;
    scalerCropRegion.width = (uint32_t)sensorW/zoomRatio;
    scalerCropRegion.height = (uint32_t)sensorH/zoomRatio;

    LOGD("Cam type %d : zoomRatio %f Crop [%d, %d, %d %d]",
        cam_type, zoomRatio,
        scalerCropRegion.left,
        scalerCropRegion.top,
        scalerCropRegion.width,
        scalerCropRegion.height);

    ADD_SET_PARAM_ENTRY_TO_BATCH(params, CAM_INTF_META_SCALER_CROP_REGION,
        scalerCropRegion);
}
/*===========================================================================
 * FUNCTION   : translateRoi
 *
 * DESCRIPTION: Translate the focus areas from main to aux camera.
 *
 * PARAMETERS :
 * @roiAfMain : Focus area ROI for main camera
 * @cam       : Cam type
 *
 * RETURN     : Translated focus area ROI for aux camera
 *
 *==========================================================================*/
cam_area_t QCameraFOVControl::translateRoi(
        cam_area_t  roiMain,
        cam_sync_type_t cam)
{
    float fovRatio;
    float zoomWide;
    float zoomTele;
    float AuxRoiLeft;
    float AuxRoiTop;
    cam_area_t roiTrans = roiMain;
    int32_t shiftHorzAdjusted;
    int32_t shiftVertAdjusted;
    uint32_t maxW, maxH;

    zoomWide = findZoomRatio(mFovControlData.zoomWide) / (float)mFovControlData.zoomRatioTable[0];
    zoomTele = findZoomRatio(mFovControlData.zoomTele) / (float)mFovControlData.zoomRatioTable[0];

    if (cam == mFovControlData.camWide) {
        fovRatio = (mHalPPType == CAM_HAL_PP_TYPE_BOKEH) ?
                        (1.0f / mFovControlData.transitionParams.cropRatio) : 1.0f;
    } else {
        fovRatio = (zoomTele / zoomWide) * mFovControlData.transitionParams.cropRatio;
    }

    // Acquire the mutex in order to read the spatial alignment result which is written
    // by another thread
    mMutex.lock();
    if (cam == mFovControlData.camWide) {
        shiftHorzAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftHorz * zoomWide;
        shiftVertAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiWide.shiftVert * zoomWide;
    } else {
        shiftHorzAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftHorz * zoomTele;
        shiftVertAdjusted = mFovControlData.spatialAlignResult.shiftAfRoiTele.shiftVert * zoomTele;
    }
    mMutex.unlock();

    roiTrans.rect.width  = roiMain.rect.width * fovRatio;
    roiTrans.rect.height = roiMain.rect.height * fovRatio;

    AuxRoiTop = ((roiMain.rect.top - mDualCamParams.paramsMain.sensorStreamHeight / 2.0f) * fovRatio)
                    + (mDualCamParams.paramsMain.sensorStreamHeight / 2.0f);
    roiTrans.rect.top =  AuxRoiTop - shiftVertAdjusted;
    AuxRoiLeft = ((roiMain.rect.left - mDualCamParams.paramsMain.sensorStreamWidth / 2.0f) * fovRatio)
                    + (mDualCamParams.paramsMain.sensorStreamWidth / 2.0f);
    roiTrans.rect.left = AuxRoiLeft - shiftHorzAdjusted;

    if (CAM_TYPE_MAIN == cam) {
        maxW = mDualCamParams.paramsMain.sensorStreamWidth;
        maxH = mDualCamParams.paramsMain.sensorStreamHeight;
    } else {
        maxW = mDualCamParams.paramsAux.sensorStreamWidth;
        maxH = mDualCamParams.paramsAux.sensorStreamHeight;
    }


    // Check the ROI bounds and correct if necessory
    if ((roiTrans.rect.width >= (int32_t)maxW) ||
        (roiTrans.rect.height >= (int32_t)maxH)) {
        roiTrans = roiMain;
        LOGW("ROI translation failed, reverting to the pre-translation ROI");
    } else {
        bool error = false;
        if (roiTrans.rect.left < 0) {
            roiTrans.rect.left = 0;
            error = true;
        }
        if (roiTrans.rect.top < 0) {
            roiTrans.rect.top = 0;
            error = true;
        }
        if ((roiTrans.rect.left >= (int32_t)maxW) ||
            ((roiTrans.rect.left + roiTrans.rect.width) > (int32_t) maxW)) {
            roiTrans.rect.left = maxW - roiTrans.rect.width;
            error = true;
        }
        if ((roiTrans.rect.top >= (int32_t)maxH) ||
            ((roiTrans.rect.top + roiTrans.rect.height) > (int32_t)maxH)) {
            roiTrans.rect.top = maxH - roiTrans.rect.height;
            error = true;
        }
        if (error) {
            LOGW("Translated ROI - out of bounds. Clamping to the nearest valid ROI");
        }
    }

    LOGD("Translated ROI - %s: L:%d, T:%d, W:%d, H:%d",
            (cam == CAM_TYPE_MAIN) ? "main cam" : "aux  cam", roiTrans.rect.left,
            roiTrans.rect.top, roiTrans.rect.width, roiTrans.rect.height);
    return roiTrans;
}

/*===========================================================================
 * FUNCTION   : translateHAL3FDRoi
 *
 * DESCRIPTION: Translate face detection ROI from aux metadata to main
 *
 * PARAMETERS :
 * @faceDetectionInfo  : face detection data from aux metadata. This face
 *                       detection data is overwritten with the translated
 *                       FD ROI.
 * @cam                : Cam type
 *
 * RETURN     : none
 *
 *==========================================================================*/
cam_face_detection_data_t QCameraFOVControl::translateHAL3FDRoi(
        cam_face_detection_data_t metaFD,
        cam_sync_type_t cam)
{
    cam_face_detection_data_t metaFDTranslated = metaFD;
    int32_t shiftHorz = 0;
    int32_t shiftVert = 0;
    uint32_t maxW, maxH;

    float zoomWide = findZoomRatio(mFovControlData.zoomWide) /
                        (float)mFovControlData.zoomRatioTable[0];
    float zoomTele = findZoomRatio(mFovControlData.zoomTele) /
                        (float)mFovControlData.zoomRatioTable[0];

   if (cam == mFovControlData.camWide) {
        shiftHorz = mFovControlData.spatialAlignResult.shiftWide.shiftHorz * zoomWide;
        shiftVert = mFovControlData.spatialAlignResult.shiftWide.shiftVert * zoomWide;
   } else {
        shiftHorz = mFovControlData.spatialAlignResult.shiftTele.shiftHorz * zoomTele;
        shiftVert = mFovControlData.spatialAlignResult.shiftTele.shiftVert * zoomTele;
   }

       maxW = mDualCamParams.paramsMain.sensorStreamWidth;
       maxH = mDualCamParams.paramsMain.sensorStreamHeight;

   for (int i = 0; i < metaFDTranslated.num_faces_detected; ++i) {
       metaFDTranslated.faces[i].face_boundary.left += shiftHorz;
       metaFDTranslated.faces[i].face_boundary.top  += shiftVert;
       //maping AUX fd values to MAIN.
       if(CAM_TYPE_AUX == cam && (mHalPPType != CAM_HAL_PP_TYPE_BOKEH))
       {
           float widthFactor = (float)(mDualCamParams.paramsMain.sensorStreamWidth/
                               (float)mDualCamParams.paramsAux.sensorStreamWidth);
           float heightFactor = (float)(mDualCamParams.paramsMain.sensorStreamHeight/
                               (float)mDualCamParams.paramsAux.sensorStreamHeight);
           float zoomFactor = (float)(zoomTele / (float)zoomWide);
           cam_rect_t *face = &metaFDTranslated.faces[i].face_boundary;
           face->left =
               (((face->left - mDualCamParams.paramsAux.sensorStreamWidth/2.0f) * zoomFactor)
               + (mDualCamParams.paramsAux.sensorStreamWidth / 2.0f)) * widthFactor;
           face->top =
               (((face->top - mDualCamParams.paramsAux.sensorStreamHeight/2.0f) * zoomFactor)
               + (mDualCamParams.paramsAux.sensorStreamHeight/2.0f)) * heightFactor;
           face->width *= zoomFactor;
           face->height *= zoomFactor;
       }
   }

    // If ROI is out of bounds, remove that FD ROI from the list
    for (int i = 0; i < metaFDTranslated.num_faces_detected; ++i) {
        if ((metaFDTranslated.faces[i].face_boundary.left < 0) ||
            (metaFDTranslated.faces[i].face_boundary.left >= (int32_t)maxW) ||
            (metaFDTranslated.faces[i].face_boundary.top < 0) ||
            (metaFDTranslated.faces[i].face_boundary.top >= (int32_t)maxH) ||
            ((metaFDTranslated.faces[i].face_boundary.left +
                    metaFDTranslated.faces[i].face_boundary.width) >= (int32_t)maxW) ||
            ((metaFDTranslated.faces[i].face_boundary.top +
                    metaFDTranslated.faces[i].face_boundary.height) >= (int32_t)maxH)) {
            // Invalid FD ROI detected
            LOGW("Failed translating FD ROI %s: L:%d, T:%d, W:%d, H:%d",
                    (cam == CAM_TYPE_MAIN) ? "main cam" : "aux  cam",
                    metaFDTranslated.faces[i].face_boundary.left,
                    metaFDTranslated.faces[i].face_boundary.top,
                    metaFDTranslated.faces[i].face_boundary.width,
                    metaFDTranslated.faces[i].face_boundary.height);

            // Remove it by copying the last FD ROI at this index
            if (i < (metaFDTranslated.num_faces_detected - 1)) {
                metaFDTranslated.faces[i] =
                        metaFDTranslated.faces[metaFDTranslated.num_faces_detected - 1];
                // Decrement the current index to process the newly copied FD ROI.
                --i;
            }
            --metaFDTranslated.num_faces_detected;
        }
        else {
            LOGD("Translated FD ROI-%d %s: L:%d, T:%d, W:%d, H:%d", i,
                    (cam == CAM_TYPE_MAIN) ? "main cam" : "aux  cam",
                    metaFDTranslated.faces[i].face_boundary.left,
                    metaFDTranslated.faces[i].face_boundary.top,
                    metaFDTranslated.faces[i].face_boundary.width,
                    metaFDTranslated.faces[i].face_boundary.height);
        }
    }
    return metaFDTranslated;
}

}; // namespace qcamera
