/* Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
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

#ifndef __QCAMERA3HARDWAREINTERFACE_H__
#define __QCAMERA3HARDWAREINTERFACE_H__

// System dependencies
#include <pthread.h>
#include <utils/KeyedVector.h>
#include <utils/List.h>
#include <map>
#include "CameraMetadata.h"

// Camera dependencies
#include "hardware/camera3.h"
#include "QCamera3Channel.h"
#include "QCamera3CropRegionMapper.h"
#include "QCamera3HALHeader.h"
#include "QCamera3Mem.h"
#include "QCameraPerf.h"
#include "QCameraCommon.h"
#include "QCamera3VendorTags.h"
#include "QCameraDualCamSettings.h"
#include "QCameraFOVControl.h"


extern "C" {
#include "mm_camera_interface.h"
#include "mm_jpeg_interface.h"
}

using namespace android;


namespace qcamera {

using ::android::hardware::camera::common::V1_0::helper::CameraMetadata;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* Time related macros */
typedef int64_t nsecs_t;
#define NSEC_PER_SEC 1000000000LLU
#define NSEC_PER_USEC 1000LLU
#define NSEC_PER_33MSEC 33000000LLU

/*Orchestrate Macros */
#define EV_COMP_SETTLE_DELAY   2
#define GB_HDR_HALF_STEP_EV -6
#define GB_HDR_2X_STEP_EV 6

#define FRAME_REGISTER_LRU_SIZE 256
#define INTERNAL_FRAME_STARTING_NUMBER 800
#define EMPTY_FRAMEWORK_FRAME_NUMBER 0xFFFFFFFF

//blur range
#define MIN_BLUR 0
#define MAX_BLUR 100
#define BLUR_STEP 1


typedef enum {
    SET_ENABLE,
    SET_CONTROLENABLE,
    SET_RELOAD_CHROMATIX,
    SET_STATUS,
} optype_t;

typedef enum {
    QCFA_INACTIVE,
    QCFA_RAW_OUTPUT,
    QCFA_RAW_REPROCESS
} quadra_cfa_state_t;


#define MODULE_ALL 0

extern volatile uint32_t gCamHal3LogLevel;

class QCamera3MetadataChannel;
class QCamera3PicChannel;
class QCamera3HeapMemory;
class QCamera3Exif;

typedef struct {
    camera3_stream_t *stream;
    camera3_stream_buffer_set_t buffer_set;
    stream_status_t status;
    int registered;
    QCamera3ProcessingChannel *channel;
} stream_info_t;

typedef struct {
    // Stream handle
    camera3_stream_t *stream;
    // Buffer handle
    buffer_handle_t *buffer;
    // Buffer status
    camera3_buffer_status_t bufStatus = CAMERA3_BUFFER_STATUS_OK;
} PendingBufferInfo;

typedef struct {
    // Frame number corresponding to request
    uint32_t frame_number;
    // Time when request queued into system
    nsecs_t timestamp;
    List<PendingBufferInfo> mPendingBufferList;
} PendingBuffersInRequest;

class PendingBuffersMap {
public:
    // Number of outstanding buffers at flush
    uint32_t numPendingBufsAtFlush;
    // List of pending buffers per request
    List<PendingBuffersInRequest> mPendingBuffersInRequest;
    uint32_t get_num_overall_buffers();
    void removeBuf(buffer_handle_t *buffer);
    int32_t getBufErrStatus(buffer_handle_t *buffer);
};

class FrameNumberRegistry {
public:

    FrameNumberRegistry();
    ~FrameNumberRegistry();
    int32_t allocStoreInternalFrameNumber(uint32_t frameworkFrameNumber,
            uint32_t &internalFrameNumber);
    int32_t generateStoreInternalFrameNumber(uint32_t &internalFrameNumber);
    int32_t freeInternalFrameNumber(uint32_t internalFrameNumber);
    int32_t getFrameworkFrameNumber(uint32_t internalFrameNumber, uint32_t &frameworkFrameNumber);
    void purgeOldEntriesLocked();

private:
    std::map<uint32_t, uint32_t> _register;
    uint32_t _nextFreeInternalNumber;
    Mutex mRegistryLock;
};

class QCamera3HardwareInterface {
public:
    /* static variable and functions accessed by camera service */
    static camera3_device_ops_t mCameraOps;
    //Id of each session in bundle/link
    static uint32_t sessionId[MM_CAMERA_MAX_NUM_SENSORS];
    static int initialize(const struct camera3_device *,
                const camera3_callback_ops_t *callback_ops);
    static int configure_streams(const struct camera3_device *,
                camera3_stream_configuration_t *stream_list);
    static const camera_metadata_t* construct_default_request_settings(
                                const struct camera3_device *, int type);
    static int process_capture_request(const struct camera3_device *,
                                camera3_capture_request_t *request);

    static void dump(const struct camera3_device *, int fd);
    static int flush(const struct camera3_device *);
    static int close_camera_device(struct hw_device_t* device);

public:
    QCamera3HardwareInterface(uint32_t cameraId,
            const camera_module_callbacks_t *callbacks);
    virtual ~QCamera3HardwareInterface();
    static void camEvtHandle(uint32_t camera_handle, mm_camera_event_t *evt,
                                          void *user_data);
    int openCamera(struct hw_device_t **hw_device);
    camera_metadata_t* translateCapabilityToMetadata(int type);

    typedef struct {
        camera3_stream_t *stream;
        bool need_metadata;
        bool meteringOnly;
    } InternalRequest;

    static int getCamInfo(uint32_t cameraId, struct camera_info *info);
    static cam_capability_t *getCapabilities(mm_camera_ops_t *ops,
            uint32_t cam_handle);
    static int initCapabilities(uint32_t cameraId);
    static int initStaticMetadata(uint32_t cameraId);
    static void makeTable(cam_dimension_t *dimTable, size_t size,
            size_t max_size, int32_t *sizeTable);
    static void makeFPSTable(cam_fps_range_t *fpsTable, size_t size,
            size_t max_size, int32_t *fpsRangesTable);
    static void makeOverridesList(cam_scene_mode_overrides_t *overridesTable,
            size_t size, size_t max_size, uint8_t *overridesList,
            uint8_t *supported_indexes, uint32_t camera_id);
    static size_t filterJpegSizes(int32_t *jpegSizes, int32_t *processedSizes,
            size_t processedSizesCnt, size_t maxCount, cam_rect_t active_array_size,
            uint8_t downscale_factor);
    static void convertToRegions(cam_rect_t rect, int32_t* region, int weight);
    static void convertFromRegions(cam_area_t &roi, const camera_metadata_t *settings,
                                   uint32_t tag);
    static bool resetIfNeededROI(cam_area_t* roi, const cam_crop_region_t* scalerCropRegion);
    static int32_t getSensorSensitivity(int32_t iso_mode);
    static uint8_t getIsoMode(int32_t sensitivity);

    double computeNoiseModelEntryS(int32_t sensitivity);
    double computeNoiseModelEntryO(int32_t sensitivity);

    static void captureResultCb(mm_camera_super_buf_t *metadata,
                camera3_stream_buffer_t *buffer, uint32_t frame_number,
                bool isInputBuffer, void *userdata);

    static void internalMetaCb(mm_camera_super_buf_t *metadata,
                camera3_stream_buffer_t *buffer, uint32_t frame_number,
                bool isInputBuffer, void *userdata);

    int initialize(const camera3_callback_ops_t *callback_ops);
    int configureStreams(camera3_stream_configuration_t *stream_list);
    int configureStreamsPerfLocked(camera3_stream_configuration_t *stream_list);
    int processCaptureRequest(camera3_capture_request_t *request,
                              List<InternalRequest> &internalReqs);
    int orchestrateRequest(camera3_capture_request_t *request);
    void orchestrateResult(camera3_capture_result_t *result);
    void orchestrateNotify(camera3_notify_msg_t *notify_msg);

    void dump(int fd);
    int flushPerf();

    int setFrameParameters(camera3_capture_request_t *request,
            cam_stream_ID_t streamID, int blob_request, uint32_t snapshotStreamId);
    int32_t setReprocParameters(camera3_capture_request_t *request,
            metadata_buffer_t *reprocParam, uint32_t snapshotStreamId);
    int8_t getReprocChannelCnt() {return m_ppChannelCnt;};
    int translateToHalMetadata(const camera3_capture_request_t *request,
            metadata_buffer_t *parm, uint32_t snapshotStreamId);
    camera_metadata_t* translateCbUrgentMetadataToResultMetadata (
                             metadata_buffer_t *metadata);
    camera_metadata_t* translateFromHalMetadata(metadata_buffer_t *metadata,
                            nsecs_t timestamp, int32_t request_id,
                            const CameraMetadata& jpegMetadata, uint8_t pipeline_depth,
                            uint8_t capture_intent, bool pprocDone, uint8_t fwk_cacMode,
                            bool firstMetadataInBatch);
    camera_metadata_t* saveRequestSettings(const CameraMetadata& jpegMetadata,
                            camera3_capture_request_t *request);
    int initParameters();
    void deinitParameters();
    QCamera3ReprocessChannel *addOfflineReprocChannel(const reprocess_config_t &config,
            QCamera3ProcessingChannel *inputChHandle, int8_t pp_channel_idx = 0);
    bool needRotationReprocess();
    bool needJpegExifRotation();
    bool useExifRotation();
    bool needReprocess(cam_feature_mask_t postprocess_mask);
    bool needJpegRotation();
    cam_denoise_process_type_t getWaveletDenoiseProcessPlate();
    cam_denoise_process_type_t getTemporalDenoiseProcessPlate();

    void captureResultCb(mm_camera_super_buf_t *metadata,
                camera3_stream_buffer_t *buffer, uint32_t frame_number,
                bool isInputBuffer);
    void internalMetaCb(mm_camera_super_buf_t *metadata);
    cam_dimension_t getQuadraCfaDim();
    cam_dimension_t calcMaxJpegDim();
    bool needOnlineRotation();
    uint32_t getJpegQuality();
    QCamera3Exif *getExifData();
    mm_jpeg_exif_params_t get3AExifParams();
    uint8_t getMobicatMask();
    static void getFlashInfo(const int cameraId,
            bool& hasFlash,
            char (&flashNode)[QCAMERA_MAX_FILEPATH_LENGTH]);
    const char *getEepromVersionInfo();
    const uint32_t *getLdafCalib();
    void get3AVersion(cam_q3a_version_t &swVersion);
    static void setBufferErrorStatus(QCamera3Channel*, uint32_t frameNumber,
            camera3_buffer_status_t err, void *userdata);
    void setBufferErrorStatus(QCamera3Channel*, uint32_t frameNumber,
            camera3_buffer_status_t err);

    // Get dual camera related info
    bool isDeviceLinked() {return mIsDeviceLinked;}
    bool isMainCamera() {return mIsMainCamera;}
    uint32_t getSensorMountAngle();
    const cam_related_system_calibration_data_t *getRelatedCalibrationData();
    int getCameraId() {return mCameraId;}
    bool isQuadCfaSensor() {return m_bQuadraCfaSensor;}
    int32_t deleteQCFARawChannel();

    template <typename fwkType, typename halType> struct QCameraMap {
        fwkType fwk_name;
        halType hal_name;
    };

    typedef struct {
        const char *const desc;
        cam_cds_mode_type_t val;
    } QCameraPropMap;

    uint32_t getCameraID() {return mCameraId;}
    bool isDualCamera() { return mDualCamera; };
    int32_t bundleRelatedCameras(bool enable_sync);
    cam_hal_pp_type_t getHalPPType() {return m_halPPType;}
    bool isDimSupportedbyCamType(const cam_dimension_t &dim, const cam_sync_type_t &type);
    cam_dimension_t getOptimalSupportedDim(const cam_dimension_t &dim, const cam_sync_type_t &type);
    cam_dimension_t getMaxSingleIspRes();
    bool isAsymetricDim(const cam_dimension_t &dim);
    bool isPPUpscaleNeededForDim(const cam_dimension_t &dim);
    void rectifyStreamDimIfNeeded(
        cam_dimension_t &dim, const cam_sync_type_t &type, bool &needUpScale);
    uint32_t getBlurLevel() {return mBlurLevel;}
    cam_dual_camera_perf_mode_t getLowPowerMode(cam_sync_type_t cam);
    bool needHALPP() {return m_bNeedHalPP;}
    cam_capability_t *getCamHalCapabilities();
private:

    // State transition conditions:
    // "\" means not applicable
    // "x" means not valid
    // +------------+----------+----------+-------------+------------+---------+-------+--------+
    // |            |  CLOSED  |  OPENED  | INITIALIZED | CONFIGURED | STARTED | ERROR | DEINIT |
    // +------------+----------+----------+-------------+------------+---------+-------+--------+
    // |  CLOSED    |    \     |   open   |     x       |    x       |    x    |   x   |   x    |
    // +------------+----------+----------+-------------+------------+---------+-------+--------+
    // |  OPENED    |  close   |    \     | initialize  |    x       |    x    | error |   x    |
    // +------------+----------+----------+-------------+------------+---------+-------+--------+
    // |INITIALIZED |  close   |    x     |     \       | configure  |   x     | error |   x    |
    // +------------+----------+----------+-------------+------------+---------+-------+--------+
    // | CONFIGURED |  close   |    x     |     x       | configure  | request | error |   x    |
    // +------------+----------+----------+-------------+------------+---------+-------+--------+
    // |  STARTED   |  close   |    x     |     x       | configure  |    \    | error |   x    |
    // +------------+----------+----------+-------------+------------+---------+-------+--------+
    // |   ERROR    |  close   |    x     |     x       |     x      |    x    |   \   |  any   |
    // +------------+----------+----------+-------------+------------+---------+-------+--------+
    // |   DEINIT   |  close   |    x     |     x       |     x      |    x    |   x   |   \    |
    // +------------+----------+----------+-------------+------------+---------+-------+--------+

    typedef enum {
        CLOSED,
        OPENED,
        INITIALIZED,
        CONFIGURED,
        STARTED,
        ERROR,
        DEINIT
    } State;

    int openCamera();
    int closeCamera();
    int flush(bool restartChannels);
    static size_t calcMaxJpegSize(uint32_t camera_id);
    cam_dimension_t getMaxRawSize(uint32_t camera_id);
    static void addStreamConfig(Vector<int32_t> &available_stream_configs,
            int32_t scalar_format, const cam_dimension_t &dim,
            int32_t config_type);

    int validateCaptureRequest(camera3_capture_request_t *request,
                               List<InternalRequest> &internallyRequestedStreams);
    int validateStreamDimensions(camera3_stream_configuration_t *streamList);
    int validateStreamRotations(camera3_stream_configuration_t *streamList);
    void deriveMinFrameDuration();
    void handleBuffersDuringFlushLock(camera3_stream_buffer_t *buffer);
    int32_t handlePendingReprocResults(uint32_t frame_number);
    int64_t getMinFrameDuration(const camera3_capture_request_t *request);
    void handleMetadataWithLock(mm_camera_super_buf_t *metadata_buf,
            bool free_and_bufdone_meta_buf,
            bool firstMetadataInBatch,
            bool *p_is_metabuf_queued);
    void handleBatchMetadata(mm_camera_super_buf_t *metadata_buf,
            bool free_and_bufdone_meta_buf);
    void handleBufferWithLock(camera3_stream_buffer_t *buffer,
            uint32_t frame_number);
    void handleInputBufferWithLock(uint32_t frame_number);
    void unblockRequestIfNecessary();
    void dumpMetadataToFile(tuning_params_t &meta, uint32_t &dumpFrameCount,
            bool enabled, const char *type, uint32_t frameNumber);
    static void getLogLevel();

    void cleanAndSortStreamInfo();
    void extractJpegMetadata(CameraMetadata& jpegMetadata,
            const camera3_capture_request_t *request);

    bool isSupportChannelNeeded(camera3_stream_configuration_t *streamList,
            cam_stream_size_info_t stream_config_info);
    bool isHdrSnapshotRequest(camera3_capture_request *request);
    int32_t setMobicat();

    int32_t getSensorOutputSize(cam_dimension_t &sensor_dim, uint32_t cam_type = CAM_TYPE_MAIN);
    int32_t setHalFpsRange(const CameraMetadata &settings,
            metadata_buffer_t *hal_metadata);
    int32_t extractSceneMode(const CameraMetadata &frame_settings, uint8_t metaMode,
            metadata_buffer_t *hal_metadata);
    int32_t setVideoHdrMode(metadata_buffer_t *hal_metadata,
            cam_video_hdr_mode_t vhdr);
    int32_t numOfSizesOnEncoder(const camera3_stream_configuration_t *streamList,
            const cam_dimension_t &maxViewfinderSize);

    void addToPPFeatureMask(int stream_format, uint32_t stream_idx);
    void setDCFeature(cam_feature_mask_t& feature_mask,
            cam_stream_type_t stream_type);
    void updateFpsInPreviewBuffer(metadata_buffer_t *metadata, uint32_t frame_number);
    void updateTimeStampInPendingBuffers(uint32_t frameNumber, nsecs_t timestamp);

    void enablePowerHint();
    void disablePowerHint();
    int32_t dynamicUpdateMetaStreamInfo();
    int32_t startAllChannels();
    int32_t stopAllChannels();
    int32_t notifyErrorForPendingRequests();
    void notifyError(uint32_t frameNumber,
            camera3_error_msg_code_t errorCode);
    int32_t getReprocessibleOutputStreamId(uint32_t &id);
    int32_t handleCameraDeviceError();
    bool checkFrameInPendingList(const uint32_t frame_number);

    bool isOnEncoder(const cam_dimension_t max_viewfinder_size,
            uint32_t width, uint32_t height);
    camera_metadata_t* restoreHdrScene(uint8_t sceneMode, const camera_metadata_t *result);
    void hdrPlusPerfLock(mm_camera_super_buf_t *metadata_buf);

    static bool supportBurstCapture(uint32_t cameraId);
    int32_t setBundleInfo();
    int32_t setAuxBundleInfo();
    int32_t setInstantAEC(const CameraMetadata &meta);

    static void convertLandmarks(cam_face_landmarks_info_t face, int32_t* landmarks);
    static void setInvalidLandmarks(int32_t* landmarks);

    static void setPAAFSupport(cam_feature_mask_t& feature_mask,
            cam_stream_type_t stream_type,
            cam_color_filter_arrangement_t filter_arrangement);
    int32_t setSensorHDR(metadata_buffer_t *hal_metadata, bool enable,
            bool isVideoHdrEnable = false);
    int32_t captureQuadraCfaRawInternal(camera3_capture_request_t *request);
    int32_t switchStreamConfigInternal(uint32_t frame_number);
    uint8_t  mDualCamType;

    inline bool isBayerMono() { return (mDualCamType == DUAL_CAM_BAYER_MONO); };
    int32_t sendDualCamCmd(cam_dual_camera_cmd_type type,
            uint8_t num_cam, void *info);
    int32_t setDualCamBundleInfo(bool enable_sync,
            uint8_t bundle_cam_idx);
    int32_t configureHalPostProcess(bool bIsInput);
    void switchMaster(uint32_t masterCam);
    int32_t setDCMasterInfo(uint32_t camMaster);
    int32_t setDCControls(uint32_t camMaster, uint32_t state,
            bool bundleSnap, cam_fallback_mode_t fallbackMode);
    int32_t setDCLowPowerMode(uint32_t state);
    int32_t setDCFallbackMode(cam_fallback_mode_t fallback);
    int32_t setDCDeferCamera(cam_dual_camera_defer_cmd_t type);
    //dual camera api's
    void rectifyStreamSizesByCamType(
            cam_stream_size_info_t* streamsInfo, const cam_sync_type_t &type);
    void initDCSettings();

    camera3_device_t   mCameraDevice;
    uint32_t           mCameraId;
    uint32_t           mBlurLevel;
    cam_hal_pp_type_t m_halPPType;
    mm_camera_vtbl_t  *mCameraHandle;
    bool               mCameraInitialized;
    camera_metadata_t *mDefaultMetadata[CAMERA3_TEMPLATE_COUNT];
    const camera3_callback_ops_t *mCallbackOps;

    QCamera3MetadataChannel *mMetadataChannel;
    QCamera3PicChannel *mPictureChannel;
    QCamera3RawChannel *mRawChannel;
    QCamera3SupportChannel *mSupportChannel;
    QCamera3SupportChannel *mAnalysisChannel;
    QCamera3RawDumpChannel *mRawDumpChannel;
    QCamera3RegularChannel *mDummyBatchChannel;
    QCamera3DepthChannel *mDepthChannel;
    QCameraPerfLockMgr mPerfLockMgr;
    QCameraFOVControl *m_pFovControl;
    uint32_t mChannelHandle;

    void saveExifParams(metadata_buffer_t *metadata);
    mm_jpeg_exif_params_t mExifParams;

     //First request yet to be processed after configureStreams
    bool mFirstConfiguration;
    bool mFlush;
    bool mFlushPerf;
    bool mEnableRawDump;
    bool mForceHdrSnapshot;
    uint32_t mHdrFrameNum;
    bool mHdrSnapshotRunning;
    bool mShouldSetSensorHdr;
    QCamera3HeapMemory *mParamHeap;
    metadata_buffer_t* mParameters;
    metadata_buffer_t* mAuxParameters;
    metadata_buffer_t* mPrevParameters;
    CameraMetadata mCurJpegMeta;
    bool m_bIsVideo;
    bool m_bIs4KVideo;
    bool m_bEisSupportedSize;
    bool m_bEisEnable;
    bool m_bEis3PropertyEnabled;
    bool m_bEisSupported;
    bool m_bLPMEnabled;
    typedef struct {
        cam_dimension_t dim;
        int format;
        uint32_t usage;
    } InputStreamInfo;

    InputStreamInfo mInputStreamInfo;
    uint8_t m_MobicatMask;
    uint8_t m_bTnrEnabled;
    int8_t  mSupportedFaceDetectMode;
    uint8_t m_bTnrPreview;
    uint8_t m_bSwTnrPreview;
    uint8_t m_bTnrVideo;
    uint8_t m_debug_avtimer;
    uint8_t m_cacModeDisabled;

    /* Data structure to store pending request */
    typedef struct {
        camera3_stream_t *stream;
        camera3_stream_buffer_t *buffer;
        // metadata needs to be consumed by the corresponding stream
        // in order to generate the buffer.
        bool need_metadata;
    } RequestedBufferInfo;

    typedef struct {
        uint32_t frame_number;
        uint32_t num_buffers;
        int32_t request_id;
        List<RequestedBufferInfo> buffers;
        List<InternalRequest> internalRequestList;
        int blob_request;
        uint8_t bUrgentReceived;
        nsecs_t timestamp;
        camera3_stream_buffer_t *input_buffer;
        const camera_metadata_t *settings;
        CameraMetadata jpegMetadata;
        uint8_t pipeline_depth;
        uint32_t partial_result_cnt;
        uint8_t capture_intent;
        uint8_t fwkCacMode;
        bool shutter_notified;
        uint8_t scene_mode;
    } PendingRequestInfo;
    typedef struct {
        uint32_t frame_number;
        uint32_t stream_ID;
    } PendingFrameDropInfo;

    typedef struct {
        camera3_notify_msg_t notify_msg;
        camera3_stream_buffer_t buffer;
        uint32_t frame_number;
    } PendingReprocessResult;

    class FrameNumberRegistry _orchestrationDb;
    typedef KeyedVector<uint32_t, Vector<PendingBufferInfo> > FlushMap;
    typedef List<QCamera3HardwareInterface::PendingRequestInfo>::iterator
            pendingRequestIterator;
    typedef List<QCamera3HardwareInterface::RequestedBufferInfo>::iterator
            pendingBufferIterator;

    List<PendingReprocessResult> mPendingReprocessResultList;
    List<PendingRequestInfo> mPendingRequestsList;
    List<PendingFrameDropInfo> mPendingFrameDropList;
    /* Use last frame number of the batch as key and first frame number of the
     * batch as value for that key */
    KeyedVector<uint32_t, uint32_t> mPendingBatchMap;
    cam_stream_ID_t mBatchedStreamsArray;

    PendingBuffersMap mPendingBuffersMap;
    pthread_cond_t mRequestCond;
    pthread_cond_t mHdrRequestCond;
    uint32_t mPendingLiveRequest;
    bool mWokenUpByDaemon;
    int32_t mCurrentRequestId;
    cam_stream_size_info_t mStreamConfigInfo;
    cam_stream_size_info_t mAuxStreamConfigInfo;

    //mutex for serialized access to camera3_device_ops_t functions
    pthread_mutex_t mMutex;

    //condition used to signal flush after buffers have returned
    pthread_cond_t mBuffersCond;

    List<stream_info_t*> mStreamInfo;

    int64_t mMinProcessedFrameDuration;
    int64_t mMinJpegFrameDuration;
    int64_t mMinRawFrameDuration;

    uint32_t mMetaFrameCount;
    bool    mUpdateDebugLevel;
    const camera_module_callbacks_t *mCallbacks;

    uint8_t mCaptureIntent;
    uint8_t mCacMode;
    metadata_buffer_t mReprocMeta; //scratch meta buffer
    /* 0: Not batch, non-zero: Number of image buffers in a batch */
    uint8_t mBatchSize;
    // Used only in batch mode
    uint8_t mToBeQueuedVidBufs;
    // Fixed video fps
    float mHFRVideoFps;
public:
    uint32_t mOpMode;
    bool mStreamConfig;
    QCameraCommon   mCommon;
    cam_format_t mRdiModeFmt;
    QCamera3QCfaRawChannel *mQCFARawChannel;
    bool m_bQuadraCfaRequest;
private:
    uint32_t mFirstFrameNumberInBatch;
    camera3_stream_t mDummyBatchStream;
    bool mNeedSensorRestart;
    bool mPreviewStarted;
    uint32_t mMinInFlightRequests;
    uint32_t mMaxInFlightRequests;
    // Param to trigger instant AEC.
    bool mInstantAEC;
    // Param to know when to reset AEC
    bool mResetInstantAEC;
    // Frame number, untill which we need to drop the frames.
    uint32_t mInstantAECSettledFrameNumber;
    // Max number of frames, that HAL will hold without displaying, for instant AEC mode.
    uint8_t mAecSkipDisplayFrameBound;
    // Counter to keep track of number of frames that took for AEC convergence.
    uint8_t mInstantAecFrameIdxCount;
    /* sensor output size with current stream configuration */
    QCamera3CropRegionMapper mCropRegionMapper;

    cam_feature_mask_t mCurrFeatureState;
    /* Ldaf calibration data */
    bool mLdafCalibExist;
    uint32_t mLdafCalib[2];
    int32_t mLastCustIntentFrmNum;
    CameraMetadata  mCachedMetadata;

    static const QCameraMap<camera_metadata_enum_android_control_effect_mode_t,
            cam_effect_mode_type> EFFECT_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_awb_mode_t,
            cam_wb_mode_type> WHITE_BALANCE_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_scene_mode_t,
            cam_scene_mode_type> SCENE_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_af_mode_t,
            cam_focus_mode_type> FOCUS_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_color_correction_aberration_mode_t,
            cam_aberration_mode_t> COLOR_ABERRATION_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_ae_antibanding_mode_t,
            cam_antibanding_mode_type> ANTIBANDING_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_lens_state_t,
            cam_af_lens_state_t> LENS_STATE_MAP[];
    static const QCameraMap<camera_metadata_enum_android_control_ae_mode_t,
            cam_flash_mode_t> AE_FLASH_MODE_MAP[];
    static const QCameraMap<camera_metadata_enum_android_flash_mode_t,
            cam_flash_mode_t> FLASH_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_statistics_face_detect_mode_t,
            cam_face_detect_mode_t> FACEDETECT_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_lens_info_focus_distance_calibration_t,
            cam_focus_calibration_t> FOCUS_CALIBRATION_MAP[];
    static const QCameraMap<camera_metadata_enum_android_sensor_test_pattern_mode_t,
            cam_test_pattern_mode_t> TEST_PATTERN_MAP[];
    static const QCameraMap<camera_metadata_enum_android_video_hdr_mode_t,
            cam_video_hdr_mode_t> VIDEO_HDR_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_sensor_reference_illuminant1_t,
            cam_illuminat_t> REFERENCE_ILLUMINANT_MAP[];
    static const QCameraMap<int32_t,
            cam_hfr_mode_t> HFR_MODE_MAP[];
    static const QCameraMap<camera_metadata_enum_android_ir_mode_t,
            cam_ir_mode_type_t> IR_MODES_MAP[];
    static const QCameraMap<qcamera3_ext_instant_aec_mode_t,
            cam_aec_convergence_type> INSTANT_AEC_MODES_MAP[];
    static const QCameraMap<camera_metadata_enum_android_binning_correction_mode_t,
            cam_binning_correction_mode_t> BINNING_CORRECTION_MODES_MAP[];
    static const QCameraMap<qcamera3_ext_exposure_meter_mode_t,
            cam_auto_exposure_mode_type> AEC_MODES_MAP[];
    static const QCameraMap<qcamera3_ext_iso_mode_t,
            cam_iso_mode_type> ISO_MODES_MAP[];
    static const QCameraPropMap CDS_MAP[];

    pendingRequestIterator erasePendingRequest(pendingRequestIterator i);
    //GPU library to read buffer padding details.
    void *lib_surface_utils;
    int (*LINK_get_surface_pixel_alignment)();
    uint32_t mSurfaceStridePadding;

    State mState;
    //Dual camera related params
    bool mIsDeviceLinked;
    bool mIsMainCamera;
    uint8_t mLinkedCameraId;
    QCamera3HeapMemory *m_pDualCamCmdHeap;
    cam_dual_camera_cmd_info_t *m_pDualCamCmdPtr[MM_CAMERA_MAX_CAM_CNT];
    cam_sync_related_sensors_event_info_t m_relCamSyncInfo;
    Mutex mFlushLock;
    bool m_bSensorHDREnabled;

    uint8_t mCurrentSceneMode;
    bool m_bOfflineIsp;

    // for quad cfa
    bool m_bQuadraCfaSensor;
    uint8_t mQuadraCfaStage;
    bool m_bQuadraSizeConfigured;
    int8_t m_ppChannelCnt;
    camera3_stream_configuration_t mStreamList;

    //UDCF
    bool mDualCamera;
    bool m_bNeedHalPP;
    bool mBundledSnapshot;
    uint32_t mActiveCameras;
    uint32_t mMasterCamera;
    cam_fallback_mode_t mFallbackMode;
    bool mLPMEnable;
    cam_rtb_msg_type_t mRTBStatus;
};

}; // namespace qcamera

#endif /* __QCAMERA2HARDWAREINTERFACE_H__ */
