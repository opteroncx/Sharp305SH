ifneq (, $(filter F%, $(SH_BUILD_ID)))
	EXTRA_CFLAGS += -DSHTPS_FACTORY_MODE_ENABLE
endif

ifeq ($(TARGET_BUILD_VARIANT),eng)
	EXTRA_CFLAGS += -DSHTPS_ENGINEER_BUILD_ENABLE
endif

ifeq ($(CONFIG_SHTPS_SY3000_PLG500_001),y)
	SHTPS_SY3400_DEV = y
endif

obj-$(SHTPS_SY3400_DEV)	+= 	shtps_rmi.o				\
							shtps_rmi_sub.o			\
							shtps_rmi_ldm.o			\
							shtps_rmi_devctl.o		\
							shtps_rmi_debug.o

obj-$(SHTPS_SY3400_DEV)	+= 	shtps_fwctl.o			\
							shtps_filter.o			\
							shtps_spi.o

obj-$(SHTPS_SY3400_DEV)	+= 	filter/
obj-$(SHTPS_SY3400_DEV)	+= 	fwctl/
