
LOCAL_PATH := $(call my-dir)

################################################################################
# Bolt Simulation Plugins
################################################################################

include $(CLEAR_VARS)
LOCAL_MODULE := bolt
LOCAL_DESCRIPTION := Bolt Simulation Plugins

LOCAL_LIBRARIES := \
	gz-sim \
	libnexus

# LOCAL_EXPORT_C_INCLUDES := \
# 	$(LOCAL_PATH)/gnss \
# 	$(TARGET_OUT_BUILD)/$(LOCAL_MODULE)/obj/generated/protobuf

include $(BUILD_CMAKE)
