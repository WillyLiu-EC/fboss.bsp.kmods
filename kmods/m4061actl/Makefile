#!/bin/make -f
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

obj-m += m4061actl_scmcpld.o
m4061actl_scmcpld-objs := m4061actl/m4061actl_scmcpld_i2c.o fboss_iob_led_trigger.o lib/fbcpld-cdev.o

# DS250DF810 driver
obj-m += ds250df810.o
ds250df810-objs := m4061actl/ds250df810.o

