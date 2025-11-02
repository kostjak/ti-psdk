ifeq ($(TARGET_OS),$(filter $(TARGET_OS),SYSBIOS FREERTOS SAFERTOS QNX THREADX))
ifeq ($(TARGET_CPU),$(filter $(TARGET_CPU),R5F A53))

include $(PRELUDE)
TARGET      := app_utils_sensors
TARGETTYPE  := library

ifneq ($(SOC)$(TARGET_OS),j722sQNX)
CSOURCES    := app_sensors.c
endif

ifeq ($(TARGET_CPU),$(filter $(TARGET_OS), QNX))
ifneq ($(SOC),$(filter $(SOC), am62a))
SKIPBUILD=1
endif
endif

include $(FINALE)

endif
endif
