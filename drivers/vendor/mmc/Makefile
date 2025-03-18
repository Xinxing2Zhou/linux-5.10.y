include $(src)/version.mak
ccflags-y += -DSDHCI_NEBULA_KERNEL_VERSION=\"$(SDHCI_NEBULA_KERNEL_VERSION)\"

ccflags-y += -I$(src)/
ccflags-y += -I$(src)/dfx
ccflags-y += -I$(srctree)/drivers/mmc/host
ccflags-y += -I$(srctree)/drivers/mmc/core
ccflags-y += -I$(srctree)/include/huanglong/utils/
ccflags-y += -I$(srctree)/include/huanglong/
ccflags-y += -I$(srctree)/../../huanglong/linux/include/generic/
ccflags-y += -I$(srctree)/drivers/drv/ext_inc/dftevent/

mmc_src  := sdhci_nebula.o \
			adapter/nebula_adapter.o \
			adapter/nebula_quick.o \
			dfx/nebula_dfx.o \
			platform/platform_comm.o \
			dfx/mci_proc.o \
			nebula_intf.o

mod_name := sdhcinebula

mmc_plat_src-$(CONFIG_ARCH_SS928V100)    += platform/sdhci_ss928v100.o
mmc_plat_src-$(CONFIG_ARCH_HI3519DV500_FAMILY)  += platform/sdhci_hi3519dv500.o

ifeq ($(CONFIG_SOCT_DRV_BUILD_KO),y)
CONFIG_MMC_SDHCI_SOCT := m
endif

obj-$(CONFIG_MMC_SDHCI_NEBULA) += ${mod_name}.o
obj-$(CONFIG_MMC_SDHCI_SOCT)   += ${mod_name}.o
obj-$(CONFIG_MMC_SDHCI_SHAOLINSWORD) += ${mod_name}.o

ifneq ($(CONFIG_MMC_SDHCI_SOCT)$(CONFIG_MMC_SDHCI_NEBULA),)
${mod_name}-objs := ${mmc_src} ${mmc_plat_src-y}
else
ifneq ($(findstring $(CONFIG_SOCT_CHIP_REVERSION), "shaolinsword_c"),)
${mod_name}-objs := ${mmc_src} ${mmc_plat_src-y}
else
${mod_name}-objs := platform/sdhci_shaolinsword.o adapter/nebula_fmea.o
endif
endif
