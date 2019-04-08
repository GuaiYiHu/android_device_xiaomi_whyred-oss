LOCAL_PATH:= $(call my-dir)
include $(LOCAL_PATH)/mm-camera-interface/Android.mk
include $(LOCAL_PATH)/mm-jpeg-interface/Android.mk
include $(LOCAL_PATH)/mm-jpeg-interface/test/Android.mk
include $(LOCAL_PATH)/mm-camera-test/Android.mk
include $(LOCAL_PATH)/mm-lib2d-interface/Android.mk
include $(LOCAL_PATH)/common/leak/Android.mk

