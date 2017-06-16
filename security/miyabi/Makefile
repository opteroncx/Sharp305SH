# miyabi
obj-$(CONFIG_SECURITY_MIYABI)		+= miyabi.o

ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DLOCAL_MIYABI_FBUILD
endif

