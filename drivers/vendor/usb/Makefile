include $(src)/version.mak

KBUILD_CFLAGS   += -Werror

ccflags-y += -I$(srctree)/drivers/usb/dwc3/ -DUSB_KERNEL_VERSION=\"$(USB_KERNEL_VERSION)\"

ifeq ($(CONFIG_SOCT_DRV_BUILD_KO),y)
CONFIG_USB_WING = m
endif

obj-$(CONFIG_USB_WING) += wing-usb.o
wing-usb-y := wing_usb.o proc.o
