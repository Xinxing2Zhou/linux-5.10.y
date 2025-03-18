KBUILD_CFLAGS   += -Werror

ifneq ($(CONFIG_ARCH_SHAOLINSWORD),y)

obj-$(CONFIG_WING_UPS_PHY) = wing-ups-phy.o
wing-ups-phy-y := phy.o common.o

ifeq ($(CONFIG_WING_UPS_XVP_PHY), y)
wing-ups-phy-y += xvp.o
endif

ifeq ($(CONFIG_WING_UPS_NANO_PHY), y)
wing-ups-phy-y += nano.o
endif

ifeq ($(CONFIG_WING_UPS_MISSILE_PHY), y)
wing-ups-phy-y += missile.o
endif

endif

