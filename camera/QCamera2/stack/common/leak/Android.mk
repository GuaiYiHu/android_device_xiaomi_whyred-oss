LOCAL_PATH:= $(call my-dir)

# trace logging lib
include $(CLEAR_VARS)
LOCAL_CFLAGS  := -D_ANDROID_
LOCAL_CFLAGS += -Werror -Wunused-parameter

#************* HAL headers ************#
LOCAL_C_INCLUDES := $(LOCAL_PATH)/../
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../mm-camera-interface/inc

LOCAL_HEADER_LIBRARIES := libutils_headers

#************* Kernel headers ************#
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_SRC_FILES := fdleak.cpp memleak.cpp
LOCAL_SHARED_LIBRARIES := libdl libcutils liblog
LOCAL_MODULE := libhal_dbg
LOCAL_MODULE_TAGS := optional eng

LOCAL_VENDOR_MODULE := true
include $(BUILD_SHARED_LIBRARY)

