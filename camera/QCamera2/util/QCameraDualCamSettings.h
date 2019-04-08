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

#ifndef __QCAMERADUALCAMSETTINGS_H__
#define __QCAMERADUALCAMSETTINGS_H__

#include <cam_intf.h>
#ifdef ENABLE_QC_BOKEH
#include "dualcameraddm_wrapper.h"
#endif //ENABLE_QC_BOKEH

typedef enum {
    OIS_HOLD,
    OIS_ACTIVE_IN_LPM
} dual_cam_ois_setting;
/* OIS_HOLD: This is partially active OIS with servo control enabled and gyro control disabled.
 In this mode, the lens moves back to its hold position indicated in the OTP calibration data.
 OIS_ACTIVE_IN_LPM: This setting dynamically chooses between ACTIVE and HOLD modes based on
 active camera state. It activates OIS when one of the cameras goes into LPM. When both
 the cameras stream, HOLD mode gets selected.*/


/**********************************************************************************************
 Dual camera settings
**********************************************************************************************/

/* This setting enables/disables Low Power Mode(LPM). The system is in power optimized state when
 in LPM(1) where only one camera- the master camera is streaming. The other camera doesn't stream.
 At the time of master role switch, both the cameras stream to switch the master and after
 successful role switch, the previous master can be put into LPM.
 When disabled(0), both the cameras will stream all the time. */
#define DUALCAM_LPM_ENABLE                      (1)

/* This setting indicates low power modes for two cameras
 Various low power modes:
 CAM_PERF_NONE             : No/Invalid mode
 CAM_PERF_SENSOR_SUSPEND   : Sensor sleep/suspend.
 CAM_PERF_ISPIF_FRAME_DROP : Sensor streams but ISPIF drops the frames.
 CAM_PERF_ISPIF_FRAME_SKIP : Sensor streams and ISPIF skips the frame to control frame rate. (This
 mode is currently not supported)
 CAM_PERF_STATS_FPS_CONTROL : Sensor and ISPIF stream but stats module controls the frame rate. */
#define DUALCAM_LPM_MAIN                   (CAM_PERF_ISPIF_FRAME_DROP)
#define DUALCAM_LPM_AUX                    (CAM_PERF_SENSOR_SUSPEND)

/* This setting indicates the OIS modes for camera and camcorder modes. Possible settings are
 listed under dual_cam_ois_setting. */
#define DUALCAM_OIS_MODE_CAM               (OIS_ACTIVE_IN_LPM)
#define DUALCAM_OIS_MODE_CAMCORDER         (OIS_HOLD)

/* Dual camera sync mechanism
 This setting indicates the mechanism used to sync the two cameras.
 Various sync mechanisms:
 CAM_SYNC_NO_SYNC    : Absence of any sync mechanism between the two cameras.
 CAM_SYNC_HW_SYNC    : Sensor turns on hw-sync.
 CAM_SYNC_SW_SYNC    : Sensor ensures the phase difference is kept close to zero. (This
 mode is currently not supported)
 CAM_SYNC_HYBRID_SYNC: Sensor turns on hw-sync and also injects phase as needed. */

#define DUALCAM_SYNC_MECHANISM             (CAM_SYNC_HW_SYNC)


/**********************************************************************************************
 FOV-control settings
**********************************************************************************************/

/* Main camera fallback mechanism for low light and macro scene
 If set to 1, low light and macro scene will force the transition from narrow FOV to
 wide FOV camera
 If set to 0, these conditions are ignored and the forced transition from narrow FOV to
 wide FOV camera will not take place */
#define FOVC_MAIN_CAM_FALLBACK_MECHANISM        (1)

/* Use external zoom translator. Setting this to 1 allows opening an external lib containing
 implementation for custom zoom translation for user zoom to wide zoom and tele zoom. This
 would override the default zoom translation logic present in the FOV-control. */
#define FOVC_USE_EXTERNAL_ZOOM_TRANSLATOR       (0)

/* This setting indicates the minimum number of frames the master switch cannot take place after
 the camera starts streaming. */
#define FOVC_MIN_FRAME_WAIT_FOR_MASTER_SWITCH  (0)


/**********************************************************************************************
 Camera mode settings
**********************************************************************************************/

/* This setting will enable the snapshot postprocessing.
 If set to 1, it will enable capturing snapshots from both the cameras and feeding those
 to snapshot postprocessing algorithm.
 If set to 0, a snapshot is captured only from the master camera session at any time. */
#define FOVC_CAM_SNAPSHOT_PP_ENABLE             (1)

/* This setting indicates the minimum zoom value for the snapshot postprocessing.
 Snapshot postprocessing is only enabled for zoom equal to and higher than this value. */
#define FOVC_CAM_SNAPSHOT_PP_ZOOM_MIN           (1.50)

/* This setting indicates the maximum zoom value for the snapshot postprocessing.
 Snapshot postprocessing is only enabled for zoom equal to and lower than this value. */
#define FOVC_CAM_SNAPSHOT_PP_ZOOM_MAX           (1.90)

/* This setting indicates the maximum lux index value for snapshot postprocessing.
 If the current lux index is higher than this value, snapshot postprocessing will be disabled.
 The LUX index value will be higher for dark lit scene than bright scene. */
#define FOVC_CAM_SNAPSHOT_PP_LUX_IDX_MAX        (400)

/* This setting indicates the minimum focus distance in cm for snapshot postprocessing.
 If the current current focus distance is lower than this value, snapshot postprocessing
 will be disabled */
#define FOVC_CAM_SNAPSHOT_PP_FOCUS_DIST_CM_MIN  (45)


/**********************************************************************************************
 Camcorder mode settings
**********************************************************************************************/

/* This setting will enable the snapshot postprocessing.
 If set to 1, it will enable capturing snapshots from both the cameras and feeding those
 to snapshot postprocessing algorithm.
 If set to 0, a snapshot is captured only from the master camera session at any time.
 Currently only 0 is supported for this setting */
#define FOVC_CAMCORDER_SNAPSHOT_PP_ENABLE       (0)


/**********************************************************************************************
 Main and Aux camera switch settings
**********************************************************************************************/

/* These values indicates the thresholds for the max lux index amd min focus distance in cm
 to switch the preview from main to aux camera. If the current lux index is higher than the
 threshold or if the focus distance is lower than the threshold, camera preview will not switch
 from main to aux.
 The LUX index value will be higher for dark lit scene than bright scene. */
#define FOVC_AUXCAM_SWITCH_LUX_IDX_MAX          (500)
#define FOVC_AUXCAM_SWITCH_FOCUS_DIST_CM_MIN    (40)

/* This setting indicates timeout in ms for the constant zoom optimization.
 This is a power optimization setting. When in the transition zone, if the zoom doesn't
 change for this duration, the non-master camera is put in LPM. */
#define FOVC_CONSTZOOM_TIMEOUT_MS                    (1000)

/* This setting indicates the constant zoom optimization in the snapshot postprocess range.
 Having this separate setting allows both the cameras to be streaming for the higher timeout
 before non-master is put in LPM. If user takes picture inside this range with non-master in LPM,
 it will first be woken up and then snapshots from both the cameras will be captured. */
#define FOVC_CONSTZOOM_SNAPSHOT_PP_RANGE_TIMEOUT_MS  (5000)

/* This setting indicates timeout in ms as the threshold for focus distance stable count and
 brightness stable count. This threshold is used in handling the macro scene and
 low light scenarios. */
#define FOVC_LOWLIGHT_MACROSCENE_FALLBACK_TIMEOUT_MS (3000)

/* This setting indicates if tele should be put in LPM if low light / macro scene fallback is
 initiated in the transition zone. */
#define FOVC_TELE_LPM_IN_TRANSITION_WITH_FALLBACK    (0)

/* This setting indicates the dual camera sensor configuration used whether W+T or
 Bayer + Wide Bayer, etc. Value to be set based on the SensorConfiguration enum
 in dualcameraddm_wrapper.h */
#define DUAL_CAM_CONFIG qrcp::SYMMETRIC_BAYER_MONO

#endif /* __QCAMERADUALCAM_H__ */
