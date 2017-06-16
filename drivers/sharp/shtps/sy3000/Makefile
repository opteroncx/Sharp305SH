ifneq (, $(filter F%, $(SH_BUILD_ID)))
	EXTRA_CFLAGS += -DSHTPS_FACTORY_MODE_ENABLE
endif

ifeq ($(TARGET_BUILD_VARIANT),eng)
	EXTRA_CFLAGS += -DSHTPS_ENGINEER_BUILD_ENABLE
endif

obj-$(CONFIG_SHTPS_SY3000_TM2949_001)	+= shtps_rmi_spi.o shtps_rmi_devctl.o shtps_rmi_debug.o
