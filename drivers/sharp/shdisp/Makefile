obj-$(CONFIG_SHLCDC_BOARD) += shdisp_kerl.o
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_system.o
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_bl71y6.o
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_pm.o
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_panel.o
ifeq ($(CONFIG_SHDISP_PANEL_CARIN),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_carin.o
endif
ifeq ($(CONFIG_SHDISP_PANEL_KITE),y)
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_kite.o
endif
obj-$(CONFIG_SHLCDC_BOARD) += shdisp_dbg.o
#obj-$(CONFIG_SHLCDC_BOARD) += shlcdc_eventlog.o
EXTRA_CFLAGS    += -DSH_BUILD_ID=\"$(SH_BUILD_ID)\"
EXTRA_CFLAGS    += -DUSE_LINUX
