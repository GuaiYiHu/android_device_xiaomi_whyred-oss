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

#define LOG_TAG "QCameraCommon"

#include <cutils/properties.h>

// System dependencies
#include <utils/Errors.h>
#include <stdlib.h>
#include <string.h>
#include <utils/Log.h>
#include <math.h>
#include <fcntl.h>

// Camera dependencies
#include "QCameraCommon.h"

extern "C" {
#include "mm_camera_dbg.h"
#include "mm_camera_interface.h"
}

using namespace android;

namespace qcamera {

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define ASPECT_RATIO_TOLERANCE 0.01

/*===========================================================================
 * FUNCTION   : QCameraCommon
 *
 * DESCRIPTION: default constructor of QCameraCommon
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraCommon::QCameraCommon() :
    m_pCapability(NULL)
{
}

/*===========================================================================
 * FUNCTION   : ~QCameraCommon
 *
 * DESCRIPTION: destructor of QCameraCommon
 *
 * PARAMETERS : None
 *
 * RETURN     : None
 *==========================================================================*/
QCameraCommon::~QCameraCommon()
{
}

/*===========================================================================
 * FUNCTION   : init
 *
 * DESCRIPTION: Init function for QCameraCommon
 *
 * PARAMETERS :
 *   @pCapability : Capabilities
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraCommon::init(cam_capability_t *pCapability)
{
    m_pCapability = pCapability;

    return NO_ERROR;
}

/*===========================================================================
 * FUNCTION   : calculateLCM
 *
 * DESCRIPTION: Get the LCM of 2 numbers
 *
 * PARAMETERS :
 *   @num1   : First number
 *   @num2   : second number
 *
 * RETURN     : int32_t type (LCM)
 *
 *==========================================================================*/
uint32_t QCameraCommon::calculateLCM(int32_t num1, int32_t num2)
{
   uint32_t lcm = 0;
   uint32_t temp = 0;

   if ((num1 < 1) && (num2 < 1)) {
       return 0;
   } else if (num1 < 1) {
       return num2;
   } else if (num2 < 1) {
       return num1;
   }

   if (num1 > num2) {
       lcm = num1;
   } else {
       lcm = num2;
   }
   temp = lcm;

   while (1) {
       if (((lcm % num1) == 0) && ((lcm % num2) == 0)) {
           break;
       }
       lcm += temp;
   }
   return lcm;
}

/*===========================================================================
 * FUNCTION   : getAnalysisInfo
 *
 * DESCRIPTION: Get the Analysis information based on
 *     current mode and feature mask
 *
 * PARAMETERS :
 *   @fdVideoEnabled : Whether fdVideo enabled currently
 *   @hal3           : Whether hal3 or hal1
 *   @featureMask    : Feature mask
 *   @pAnalysis_info : Analysis info to be filled
 *
 * RETURN     : int32_t type of status
 *              NO_ERROR  -- success
 *              none-zero failure code
 *==========================================================================*/
int32_t QCameraCommon::getAnalysisInfo(
        bool fdVideoEnabled,
        cam_feature_mask_t featureMask,
        cam_analysis_info_t *pAnalysisInfo)
{
    if (!pAnalysisInfo) {
        return BAD_VALUE;
    }

    pAnalysisInfo->valid = 0;

    if ((fdVideoEnabled == TRUE) &&
            (m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_VIDEO].hw_analysis_supported) &&
            (m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_VIDEO].valid)) {
        *pAnalysisInfo =
                m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_VIDEO];
    } else if (m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_STILL].valid) {
        *pAnalysisInfo =
                m_pCapability->analysis_info[CAM_ANALYSIS_INFO_FD_STILL];
    }

    if ((featureMask & CAM_QCOM_FEATURE_PAAF) &&
      (m_pCapability->analysis_info[CAM_ANALYSIS_INFO_PAAF].valid)) {
        cam_analysis_info_t *pPaafInfo =
          &m_pCapability->analysis_info[CAM_ANALYSIS_INFO_PAAF];

        if (!pAnalysisInfo->valid) {
            *pAnalysisInfo = *pPaafInfo;
        } else {
            pAnalysisInfo->analysis_max_res.width =
                MAX(pAnalysisInfo->analysis_max_res.width,
                pPaafInfo->analysis_max_res.width);
            pAnalysisInfo->analysis_max_res.height =
                MAX(pAnalysisInfo->analysis_max_res.height,
                pPaafInfo->analysis_max_res.height);
            pAnalysisInfo->analysis_recommended_res.width =
                MAX(pAnalysisInfo->analysis_recommended_res.width,
                pPaafInfo->analysis_recommended_res.width);
            pAnalysisInfo->analysis_recommended_res.height =
                MAX(pAnalysisInfo->analysis_recommended_res.height,
                pPaafInfo->analysis_recommended_res.height);
            pAnalysisInfo->analysis_padding_info.height_padding =
                calculateLCM(pAnalysisInfo->analysis_padding_info.height_padding,
                pPaafInfo->analysis_padding_info.height_padding);
            pAnalysisInfo->analysis_padding_info.width_padding =
                calculateLCM(pAnalysisInfo->analysis_padding_info.width_padding,
                pPaafInfo->analysis_padding_info.width_padding);
            pAnalysisInfo->analysis_padding_info.plane_padding =
                calculateLCM(pAnalysisInfo->analysis_padding_info.plane_padding,
                pPaafInfo->analysis_padding_info.plane_padding);
            pAnalysisInfo->analysis_padding_info.min_stride =
                MAX(pAnalysisInfo->analysis_padding_info.min_stride,
                pPaafInfo->analysis_padding_info.min_stride);
            pAnalysisInfo->analysis_padding_info.min_stride =
                ALIGN(pAnalysisInfo->analysis_padding_info.min_stride,
                pAnalysisInfo->analysis_padding_info.width_padding);

            pAnalysisInfo->analysis_padding_info.min_scanline =
                MAX(pAnalysisInfo->analysis_padding_info.min_scanline,
                pPaafInfo->analysis_padding_info.min_scanline);
            pAnalysisInfo->analysis_padding_info.min_scanline =
                ALIGN(pAnalysisInfo->analysis_padding_info.min_scanline,
                pAnalysisInfo->analysis_padding_info.height_padding);

            pAnalysisInfo->hw_analysis_supported |=
                pPaafInfo->hw_analysis_supported;
        }
    }
    return pAnalysisInfo->valid ? NO_ERROR : BAD_VALUE;
}

/*===========================================================================
 * FUNCTION   : getMatchingDimension
 *
 * DESCRIPTION: Get dimension closest to the current, but with matching aspect ratio
 *
 * PARAMETERS :
 *   @exp_dim : The dimension corresponding to desired aspect ratio
 *   @cur_dim : The dimension which has to be modified
 *
 * RETURN     : cam_dimension_t new dimensions as per desired aspect ratio
 *==========================================================================*/
cam_dimension_t QCameraCommon::getMatchingDimension(
        cam_dimension_t exp_dim,
        cam_dimension_t cur_dim)
{
    cam_dimension_t expected_dim = cur_dim;
    if ((exp_dim.width != 0) && (exp_dim.height != 0)) {
        double cur_ratio, expected_ratio;

        cur_ratio = (double)cur_dim.width / (double)cur_dim.height;
        expected_ratio = (double)exp_dim.width / (double)exp_dim.height;
        if (fabs(cur_ratio - expected_ratio) > ASPECT_RATIO_TOLERANCE) {
            if (cur_ratio < expected_ratio) {
                expected_dim.height = (int32_t)((double)cur_dim.width / expected_ratio);
            } else {
                expected_dim.width = (int32_t)((double)cur_dim.height * expected_ratio);
            }
            expected_dim.width &= ~0x1;
            expected_dim.height &= ~0x1;
        }
        LOGD("exp ratio: %f, cur ratio: %f, new dim: %d x %d",
                expected_ratio, cur_ratio, exp_dim.width, exp_dim.height);
    }
    return expected_dim;
}



/*===========================================================================
 * FUNCTION   : isVideoUBWCEnabled
 *
 * DESCRIPTION: Function to get UBWC hardware support for video.
 *
 * PARAMETERS : None
 *
 * RETURN     : TRUE -- UBWC format supported
 *              FALSE -- UBWC is not supported.
 *==========================================================================*/

bool QCameraCommon::isVideoUBWCEnabled()
{
#ifdef UBWC_PRESENT
    char prop[PROPERTY_VALUE_MAX];
    memset(prop, 0, sizeof(prop));
    /* Checking the property set by video
     * to disable/enable UBWC. And, Android P
     * onwards we use vendor prefix*/
#ifdef USE_VENDOR_PROP
    if (property_get("vendor.video.disable.ubwc", prop, "") > 0) {
        return (atoi(prop) == 0);
    }
#else
    if (property_get("video.disable.ubwc", prop, "") > 0){
        return (atoi(prop) == 0);
    }
#endif
    return TRUE;
#else
    return FALSE;
#endif
}

/*===========================================================================
 * FUNCTION   : is_target_SDM450
 *
 * DESCRIPTION: Function to check whether target is sdm630 or not.
 *
 * PARAMETERS : None
 *
 * RETURN     : TRUE -- SDM450 target.
 *              FALSE -- Some other target.
 *==========================================================================*/

bool QCameraCommon::is_target_SDM450()
{
    return (parseHWID() == 338 || parseHWID() == 351);
}


/*===========================================================================
 * FUNCTION   : is_target_SDM630
 *
 * DESCRIPTION: Function to check whether target is sdm630 or not.
 *
 * PARAMETERS : None
 *
 * RETURN     : TRUE -- SDM630 target.
 *              FALSE -- Some other target.
 *==========================================================================*/

bool QCameraCommon::is_target_SDM630()
{
    return  (parseHWID() == 318 || parseHWID() == 327);
}


bool QCameraCommon::skipAnalysisBundling()
{
    //Enabling analysis stream dynamically at ISP requires removing
    //it from bundled list of streams. This has the advantage of power
    //savings. But as of now, this feature is enabled only via setprop.
    //So, by default analysis stream gets bundled.
    char prop[PROPERTY_VALUE_MAX];
    bool needBundling = true;
    memset(prop, 0, sizeof(prop));
    property_get("persist.vendor.camera.isp.analysis_en", prop, "1");
    needBundling = atoi(prop);

    return !needBundling;
}

/*===========================================================================
 * FUNCTION   : needAnalysisStream
 *
 * DESCRIPTION: Function to check whether analysis stream is needed or not
 *
 * PARAMETERS : None
 *
 * RETURN     : TRUE /FALSE
 *==========================================================================*/
bool QCameraCommon::needAnalysisStream()
{
    bool needAnalysisStream = true;
    cam_capability_t *caps = (m_pCapability->aux_cam_cap != NULL) ?
            m_pCapability->aux_cam_cap : m_pCapability;
    if ((caps->color_arrangement == CAM_FILTER_ARRANGEMENT_Y) &&
            caps->is_mono_stats_suport) {
        needAnalysisStream = false;
    }

    return needAnalysisStream;
}

/*===========================================================================
* FUNCTION   : isBayer
*
* DESCRIPTION: check whether sensor is bayer type or not
*
* PARAMETERS : cam_capability_t
*
* RETURN    : true or false
*==========================================================================*/
bool QCameraCommon::isBayer(cam_capability_t *caps)
{
    return (caps && (caps->color_arrangement == CAM_FILTER_ARRANGEMENT_RGGB ||
            caps->color_arrangement == CAM_FILTER_ARRANGEMENT_GRBG ||
            caps->color_arrangement == CAM_FILTER_ARRANGEMENT_GBRG ||
            caps->color_arrangement == CAM_FILTER_ARRANGEMENT_BGGR));
}

/*===========================================================================
* FUNCTION   : isMono
*
* DESCRIPTION: check whether sensor is mono or not
*
* PARAMETERS : cam_capability_t
*
* RETURN    : true or false
*==========================================================================*/
bool QCameraCommon::isMono(cam_capability_t *caps)
{
    return (caps && (caps->color_arrangement == CAM_FILTER_ARRANGEMENT_Y));
}

/*===========================================================================
* FUNCTION   : getDualCameraConfig
*
* DESCRIPTION: get dual camera configuration whether B+M/W+T
*
* PARAMETERS : capabilities of main and aux cams
*
* RETURN    : dual_cam_type
*==========================================================================*/
dual_cam_type QCameraCommon::getDualCameraConfig(cam_capability_t *capsMainCam,
        cam_capability_t *capsAuxCam)
{
    dual_cam_type type = DUAL_CAM_WIDE_TELE;
    if (isBayer(capsMainCam) && isMono(capsAuxCam)) {
        type = DUAL_CAM_BAYER_MONO;
    }
    return type;
}

/*===========================================================================
* FUNCTION   : parseHWID
*
* DESCRIPTION: get SOC id of current platform
*
* PARAMETERS : None
*
* RETURN     : Return Soc Id if successfull else -1
*==========================================================================*/
int QCameraCommon::parseHWID()
{
    static int nHW_ID = -1;
    if (nHW_ID == -1)
    {
#ifdef ANDROID
        int result = -1;
        char buffer[PATH_MAX];
        FILE *device = NULL;
        device = fopen("/sys/devices/soc0/soc_id", "r");
        if(device)
        {
          /* 4 = 3 (MAX_SOC_ID_LENGTH) + 1 */
          result = fread(buffer, 1, 4, device);
          fclose(device);
        }
        else
        {
          device = fopen("/sys/devices/system/soc/soc0/id", "r");
          if(device)
          {
             result = fread(buffer, 1, 4, device);
             fclose(device);
          }
        }
        if(result > 0)
        {
           nHW_ID = atoi(buffer);
        }
        ALOGE("%s: Got HW_ID = %d",__func__, nHW_ID);
#endif
    }
    return nHW_ID;
}

bool QCameraCommon::isAutoFocusSupported(uint32_t cam_type)
{
    bool bAFSupported = false;
    bool bMainCamAFSupported = (m_pCapability->main_cam_cap->supported_focus_modes_cnt > 1);
    bool bAuxCamAFSupported = (m_pCapability->aux_cam_cap->supported_focus_modes_cnt > 1);
    if (cam_type == MM_CAMERA_DUAL_CAM) {
        bAFSupported =  (bMainCamAFSupported || bAuxCamAFSupported) ;
    } else if (cam_type == CAM_TYPE_AUX) {
        bAFSupported =  bAuxCamAFSupported;
    } else {
        bAFSupported =  bMainCamAFSupported;
    }
    LOGH("bAFSupported: %d cam_type: %d", bAFSupported, cam_type);
    return bAFSupported;
}

}; // namespace qcamera
