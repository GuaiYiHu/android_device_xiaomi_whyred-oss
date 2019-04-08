ifneq (,$(filter $(TARGET_ARCH), arm arm64))

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

SDCLANG_COMMON_DEFS := $(LOCAL_PATH)/sdllvm-common-defs.mk
SDCLANG_FLAG_DEFS := $(LOCAL_PATH)/sdllvm-flag-defs.mk

LOCAL_COPY_HEADERS_TO := qcom/camera
LOCAL_COPY_HEADERS := QCameraFormat.h

IS_QC_BOKEH_SUPPORTED := false

LOCAL_SRC_FILES := \
        util/QCameraBufferMaps.cpp \
        util/QCameraCmdThread.cpp \
        util/QCameraFlash.cpp \
        util/QCameraPerf.cpp \
        util/QCameraQueue.cpp \
        util/QCameraDisplay.cpp \
        util/QCameraCommon.cpp \
        util/QCameraTrace.cpp \
        util/camscope_packet_type.cpp \
        QCamera2Hal.cpp \
        QCamera2Factory.cpp

#HAL 3.0 source
LOCAL_SRC_FILES += \
        HAL3/QCamera3HWI.cpp \
        HAL3/QCamera3Mem.cpp \
        HAL3/QCamera3Stream.cpp \
        HAL3/QCamera3Channel.cpp \
        HAL3/QCamera3VendorTags.cpp \
        HAL3/QCamera3PostProc.cpp \
        HAL3/QCamera3CropRegionMapper.cpp \
        HAL3/QCamera3StreamMem.cpp

LOCAL_CFLAGS := -Wall -Wextra -Werror
LOCAL_CFLAGS += -DFDLEAK_FLAG
LOCAL_CFLAGS += -DMEMLEAK_FLAG
#HAL 1.0 source

ifeq ($(TARGET_SUPPORT_HAL1),false)
LOCAL_CFLAGS += -DQCAMERA_HAL3_SUPPORT
else
LOCAL_CFLAGS += -DQCAMERA_HAL1_SUPPORT
LOCAL_SRC_FILES += \
        HAL/QCamera2HWI.cpp \
        HAL/QCameraMuxer.cpp \
        HAL/QCameraMem.cpp \
        HAL/QCameraStateMachine.cpp \
        HAL/QCameraChannel.cpp \
        HAL/QCameraStream.cpp \
        HAL/QCameraPostProc.cpp \
        HAL/QCamera2HWICallbacks.cpp \
        HAL/QCameraParameters.cpp \
	HAL/CameraParameters.cpp \
        HAL/QCameraParametersIntf.cpp \
        HAL/QCameraThermalAdapter.cpp \
        util/QCameraFOVControl.cpp \
        util/QCameraHALPP.cpp \
        util/QCameraDualFOVPP.cpp \
        util/QCameraExtZoomTranslator.cpp \
        util/QCameraPprocManager.cpp \
        util/QCameraBokeh.cpp \
        util/QCameraClearSight.cpp
endif

# System header file path prefix
LOCAL_CFLAGS += -DSYSTEM_HEADER_PREFIX=sys

LOCAL_CFLAGS += -DHAS_MULTIMEDIA_HINTS -D_ANDROID


ifeq (1,$(filter 1,$(shell echo "$$(( $(PLATFORM_SDK_VERSION) <= 23 ))" )))
LOCAL_CFLAGS += -DUSE_HAL_3_3
endif

#use media extension
ifeq ($(TARGET_USES_MEDIA_EXTENSIONS), true)
LOCAL_CFLAGS += -DUSE_MEDIA_EXTENSIONS
endif

#USE_DISPLAY_SERVICE from Android O onwards
#to receive vsync event from display
ifeq ($(call is-platform-sdk-version-at-least,26),true)
USE_DISPLAY_SERVICE := true
LOCAL_CFLAGS += -DUSE_DISPLAY_SERVICE
LOCAL_CFLAGS += -std=c++11 -std=gnu++1y
else
LOCAL_CFLAGS += -std=c++11 -std=gnu++0x
endif

#Android P onwards we use vendor prefix
ifeq ($(call is-platform-sdk-version-at-least,28),true)
LOCAL_CFLAGS += -DUSE_VENDOR_PROP
endif

#HAL 1.0 Flags
LOCAL_CFLAGS += -DDEFAULT_DENOISE_MODE_ON -DHAL3 -DQCAMERA_REDEFINE_LOG
LOCAL_LDFLAGS += -Wl,--wrap=open -Wl,--wrap=close -Wl,--wrap=socket -Wl,--wrap=pipe -Wl,--wrap=mmap -Wl,--wrap=__open_2
LOCAL_LDFLAGS += -Wl,--wrap=malloc -Wl,--wrap=free -Wl,--wrap=realloc -Wl,--wrap=calloc
LOCAL_C_INCLUDES := \
        $(LOCAL_PATH)/../mm-image-codec/qexif \
        $(LOCAL_PATH)/../mm-image-codec/qomx_core \
        $(LOCAL_PATH)/include \
        $(LOCAL_PATH)/stack/common \
        $(LOCAL_PATH)/stack/common/leak \
        $(LOCAL_PATH)/stack/mm-camera-interface/inc \
        $(LOCAL_PATH)/util \
        $(LOCAL_PATH)/HAL3 \
        $(call project-path-for,qcom-media)/libstagefrighthw \
        $(call project-path-for,qcom-media)/mm-core/inc \
        $(TARGET_OUT_HEADERS)/mm-camera-lib/cp/prebuilt

LOCAL_HEADER_LIBRARIES := media_plugin_headers
LOCAL_HEADER_LIBRARIES += libandroid_sensor_headers
LOCAL_HEADER_LIBRARIES += libcutils_headers
LOCAL_HEADER_LIBRARIES += libsystem_headers
LOCAL_HEADER_LIBRARIES += libhardware_headers

#HAL 1.0 Include paths
LOCAL_C_INCLUDES += \
        $(call project-path-for,qcom-camera)/QCamera2/HAL

ifeq ($(TARGET_COMPILE_WITH_MSM_KERNEL),true)
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
endif
ifeq ($(TARGET_TS_MAKEUP),true)
LOCAL_CFLAGS += -DTARGET_TS_MAKEUP
LOCAL_C_INCLUDES += $(LOCAL_PATH)/HAL/tsMakeuplib/include
endif
ifneq (,$(filter msm8974 msm8916 msm8226 msm8610 msm8916 apq8084 msm8084 msm8994 msm8992 msm8952 msm8937 msm8953 msm8996 sdm660 msm8998 apq8098_latv, $(TARGET_BOARD_PLATFORM)))
    LOCAL_CFLAGS += -DVENUS_PRESENT
endif

ifneq (,$(filter msm8996 sdm660 msm8998 apq8098_latv,$(TARGET_BOARD_PLATFORM)))
    LOCAL_CFLAGS += -DUBWC_PRESENT
endif

ifneq (,$(filter msm8996,$(TARGET_BOARD_PLATFORM)))
    LOCAL_CFLAGS += -DTARGET_MSM8996
endif

LOCAL_CFLAGS += -DUSE_CAMERA_METABUFFER_UTILS

#LOCAL_STATIC_LIBRARIES := libqcamera2_util
LOCAL_C_INCLUDES += \
        $(TARGET_OUT_HEADERS)/qcom/display
LOCAL_C_INCLUDES += \
        $(call project-path-for,qcom-display)/libqservice
LOCAL_SHARED_LIBRARIES := liblog libhardware libutils libcutils libdl libsync
LOCAL_SHARED_LIBRARIES += libmmcamera_interface libmmjpeg_interface libui libcamera_metadata
LOCAL_SHARED_LIBRARIES += libqdMetaData libqservice libbinder
LOCAL_SHARED_LIBRARIES += libcutils libdl libhal_dbg
ifeq ($(IS_QC_BOKEH_SUPPORTED),true)
LOCAL_SHARED_LIBRARIES += libdualcameraddm
LOCAL_CFLAGS += ENABLE_QC_BOKEH
endif
ifeq ($(USE_DISPLAY_SERVICE),true)
LOCAL_SHARED_LIBRARIES += android.frameworks.displayservice@1.0 android.hidl.base@1.0 libhidlbase libhidltransport
else
LOCAL_SHARED_LIBRARIES += libgui
endif
ifeq ($(TARGET_TS_MAKEUP),true)
LOCAL_SHARED_LIBRARIES += libts_face_beautify_hal libts_detected_face_hal
endif

LOCAL_STATIC_LIBRARIES := android.hardware.camera.common@1.0-helper


LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MODULE := camera.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_PATH_32 := $(TARGET_OUT_VENDOR)/lib
LOCAL_MODULE_TAGS := optional

LOCAL_32_BIT_ONLY := $(BOARD_QTI_CAMERA_32BIT_ONLY)
include $(BUILD_SHARED_LIBRARY)

include $(call first-makefiles-under,$(LOCAL_PATH))
endif

# Clear SDCLANG_FLAG_DEFS after use
SDCLANG_FLAG_DEFS :=
