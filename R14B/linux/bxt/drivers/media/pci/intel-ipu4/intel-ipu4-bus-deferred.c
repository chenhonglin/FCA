/*
 * Copyright (c) 2013--2017 Intel Corporation.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include "intel-ipu4-bus.h"

static int intel_ipu4_deferred_probe(void *arg)
{
	process_deferred_probe(&intel_ipu4_bus.deferred_probe_pending_list);
	return 0;
}

static int __init intel_ipu4_deferred_init(void)
{
	if (list_empty(&intel_ipu4_bus.deferred_probe_pending_list))
		printk("deferred list empty!!\n");
	kthread_run(intel_ipu4_deferred_probe, NULL, "intel ipu4 deferred probe");

	return 0;
}
#ifdef CONFIG_CAMERA_IPU4_EARLY_DEVICE
fs_initcall_sync(intel_ipu4_deferred_init);
#else
device_initcall(intel_ipu4_deferred_init);
#endif
