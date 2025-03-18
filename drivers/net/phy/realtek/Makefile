# SPDX-License-Identifier: GPL-2.0
#
# Makefile for REALTEK networking PHY driver

obj-$(CONFIG_REALTEK_PHY) := realtek.o
realtek-objs := realtek_main.o

ifdef CONFIG_NETWORK_PHY_TIMESTAMPING
realtek-objs += realtek_ptp.o
endif
