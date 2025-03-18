#
# arch/arm64/boot/Makefile
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies.
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 2012, ARM Ltd.
# Author: Will Deacon <will.deacon@arm.com>
#
# Based on the ia64 boot/Makefile.
#

OBJCOPYFLAGS_Image :=-O binary -R .note -R .note.gnu.build-id -R .comment -S

targets := Image Image.bz2 Image.gz Image.lz4 Image.lzma Image.lzo uImage uImage-fdt

$(obj)/Image: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/Image.bz2: $(obj)/Image FORCE
	$(call if_changed,bzip2)

$(obj)/Image.gz: $(obj)/Image FORCE
	$(call if_changed,gzip)

$(obj)/Image.lz4: $(obj)/Image FORCE
	$(call if_changed,lz4)

$(obj)/Image.lzma: $(obj)/Image FORCE
	$(call if_changed,lzma)

$(obj)/Image.lzo: $(obj)/Image FORCE
	$(call if_changed,lzo)

TEXT_OFFSET := 0x0
UIMAGE_LOADADDR := $(TEXT_OFFSET)
UIMAGE_ENTRYADDR := $(TEXT_OFFSET)
check_for_multiple_loadaddr = \
if [ $(words $(UIMAGE_LOADADDR)) -ne 1 ]; then \
	echo 'multiple (or no) load addresses: $(UIMAGE_LOADADDR)'; \
	echo 'This is incompatible with uImages'; \
	echo 'Specify LOADADDR on the commandline to build an uImage'; \
	false; \
fi

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
DTB_OBJS := $(addprefix $(obj)/dts/,$(DTB_LIST))

$(obj)/uImage: $(obj)/Image FORCE
	@$(check_for_multiple_loadaddr)
	@dd if=$< of=$<.dd ibs=4096 conv=sync && mv $<.dd $<
	$(call cmd,uimage)
	@cp $@ $@-fdt
	$(if $(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE),@$(kecho) '  CAT   $(DTB_OBJS) to $@-fdt')
	$(if $(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE),@cat $(DTB_OBJS) >>$@-fdt,)
	@$(kecho) '  Image $@ is ready'

install:
	$(CONFIG_SHELL) $(srctree)/$(src)/install.sh $(KERNELRELEASE) \
	$(obj)/Image System.map "$(INSTALL_PATH)"

zinstall:
	$(CONFIG_SHELL) $(srctree)/$(src)/install.sh $(KERNELRELEASE) \
	$(obj)/Image.gz System.map "$(INSTALL_PATH)"
