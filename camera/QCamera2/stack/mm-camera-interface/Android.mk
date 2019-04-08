OLD_LOCAL_PATH := $(LOCAL_PATH)
LOCAL_PATH := $(call my-dir)

include $(LOCAL_PATH)/../../../common.mk
include $(CLEAR_VARS)

LOCAL_HEADER_LIBRARIES := libhardware_headers
LOCAL_HEADER_LIBRARIES += media_plugin_headers

MM_CAM_FILES := \
src/mm_camera_interface.c \
src/mm_camera.c \
src/mm_camera_muxer.c \
src/mm_camera_channel.c \
src/mm_camera_stream.c \
src/mm_camera_thread.c \
src/mm_camera_sock.c

# System header file path prefix
LOCAL_CFLAGS += -DSYSTEM_HEADER_PREFIX=sys

ifeq ($(strip $(TARGET_USES_ION)),true)
LOCAL_CFLAGS += -DUSE_ION
endif

ifeq ($(shell expr $(TARGET_KERNEL_VERSION) \>= 4.4), 1)
LOCAL_CFLAGS += -DUSE_KERNEL_VERSION_GE_4_4_DEFS
endif

ifneq (,$(filter msm8974 msm8916 msm8226 msm8610 msm8916 apq8084 msm8084 msm8994 msm8992 msm8952 msm8937 msm8953 msm8996 sdm660 msm8998 apq8098_latv, $(TARGET_BOARD_PLATFORM)))
    LOCAL_CFLAGS += -DVENUS_PRESENT
endif

ifneq (,$(filter msm8996 sdm660 msm8998 apq8098_latv,$(TARGET_BOARD_PLATFORM)))
    LOCAL_CFLAGS += -DUBWC_PRESENT
endif

LOCAL_CFLAGS += -D_ANDROID_ -DQCAMERA_REDEFINE_LOG
LOCAL_COPY_HEADERS_TO := mm-camera-interface
LOCAL_COPY_HEADERS += ../common/cam_intf.h
LOCAL_COPY_HEADERS += ../common/cam_types.h
LOCAL_CFLAGS  += -DFDLEAK_FLAG
LOCAL_CFLAGS  += -DMEMLEAK_FLAG
LOCAL_LDFLAGS += -Wl,--wrap=open -Wl,--wrap=close -Wl,--wrap=socket -Wl,--wrap=pipe -Wl,--wrap=mmap -Wl,--wrap=__open_2
LOCAL_LDFLAGS += -Wl,--wrap=malloc -Wl,--wrap=free -Wl,--wrap=realloc -Wl,--wrap=calloc
LOCAL_C_INCLUDES := \
system/media/camera/include \
$(LOCAL_PATH)/inc \
$(LOCAL_PATH)/../common \
$(LOCAL_PATH)/../common/leak \

LOCAL_CFLAGS += -DCAMERA_ION_HEAP_ID=ION_IOMMU_HEAP_ID
LOCAL_C_INCLUDES+= $(kernel_includes)
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)

ifneq (1,$(filter 1,$(shell echo "$$(( $(PLATFORM_SDK_VERSION) >= 17 ))" )))
LOCAL_CFLAGS += -include bionic/libc/kernel/common/linux/socket.h
LOCAL_CFLAGS += -include bionic/libc/kernel/common/linux/un.h
endif

LOCAL_CFLAGS += -Wall -Wextra -Werror
ifeq ($(TARGET_KERNEL_VERSION), 4.9)
LOCAL_CFLAGS += -DUSE_4_9_DEFS
endif

LOCAL_SRC_FILES := $(MM_CAM_FILES)

LOCAL_MODULE           := libmmcamera_interface
include $(SDCLANG_COMMON_DEFS)

LOCAL_SHARED_LIBRARIES := libdl libcutils liblog \
                          libhal_dbg

LOCAL_MODULE_TAGS := optional
LOCAL_VENDOR_MODULE := true

LOCAL_32_BIT_ONLY := $(BOARD_QTI_CAMERA_32BIT_ONLY)
include $(BUILD_SHARED_LIBRARY)

LOCAL_PATH := $(OLD_LOCAL_PATH)
