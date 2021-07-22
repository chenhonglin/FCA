/*
 * linux/drivers/firmware/harman-hwid_sysfs.c
 *  Copyright (C) 2016 Harman International Ltd,
 *  by Vikram N <vikramnarayanarao20@harman.com>
 *  Created on: 13-Jan-2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as published by
 * the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "harman_hwid_sysfs.h"

#define HWID_ATTR_RO(_name)	\
		static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define HWID_ATTR_SHOW(_name)						       \
	static ssize_t _name##_show(struct kobject *kobj,		       \
				struct kobj_attribute *attr, char *buf)        \
	{								       \
		ssize_t len;						       \
		len = scnprintf(buf, PAGE_SIZE, "%s\n", harman_get_##_name()); \
		return len;						       \
	}								       \
	HWID_ATTR_RO(_name)

HWID_ATTR_SHOW(board_name);
HWID_ATTR_SHOW(board_vendor);
HWID_ATTR_SHOW(board_revision);
HWID_ATTR_SHOW(emodule_revision);
HWID_ATTR_SHOW(abl_date);
HWID_ATTR_SHOW(abl_user);
HWID_ATTR_SHOW(abl_git);
HWID_ATTR_SHOW(abl_ver);
HWID_ATTR_SHOW(saturn_variant);
HWID_ATTR_SHOW(tmc_variant);
HWID_ATTR_SHOW(sxm_variant);
HWID_ATTR_SHOW(AMP4_variant);
HWID_ATTR_SHOW(AMP7_variant);
HWID_ATTR_SHOW(rvc_variant);
HWID_ATTR_SHOW(svc_variant);
HWID_ATTR_SHOW(avm_variant);
HWID_ATTR_SHOW(A2B_variant);
HWID_ATTR_SHOW(DVR_variant);

static struct attribute *hwid_attrs[] = {
	&board_name_attr.attr,
	&board_vendor_attr.attr,
	&board_revision_attr.attr,
	&emodule_revision_attr.attr,
	&abl_date_attr.attr,
	&abl_user_attr.attr,
	&abl_git_attr.attr,
	&abl_ver_attr.attr,
	&saturn_variant_attr.attr,
	&tmc_variant_attr.attr,
	&sxm_variant_attr.attr,
	&AMP4_variant_attr.attr,
	&AMP7_variant_attr.attr,
	&rvc_variant_attr.attr,
	&svc_variant_attr.attr,
	&avm_variant_attr.attr,
	&A2B_variant_attr.attr,
	&DVR_variant_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(hwid);



static int bootinfo_proc_open(struct inode *inode, struct file *file)
{
        return single_open(file, bootinfo_proc_show, NULL);
}

static const struct file_operations bootinfo_proc_ops = {
        .open           = bootinfo_proc_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static struct proc_dir_entry *proc_bootinfo;

static void hwid_dev_release(struct device *dev)
{
	kfree(dev);
}

static struct class hwid_class = {
    #if defined(CONFIG_BJEVN60_ED2_BOARD)
	.name = "ami",
    #else
	.name = "dmi",
    #endif
	.dev_release = hwid_dev_release,
};

static struct device *hwid_dev;

static int __init harman_hwid_sysfs_init(void)
{
	int ret;

	ret = class_register(&hwid_class);
	if (ret)
		return ret;

	hwid_dev = kzalloc(sizeof(*hwid_dev), GFP_KERNEL);
	if (!hwid_dev) {
		ret = -ENOMEM;
		goto fail_class_unregister;
	}

	hwid_dev->class = &hwid_class;
	dev_set_name(hwid_dev, "id");
	hwid_dev->groups = hwid_groups;

	ret = device_register(hwid_dev);
	if (ret)
		goto fail_free_hwid_dev;
	 proc_bootinfo = proc_create("bootinfo", 0, NULL, &bootinfo_proc_ops);
        if (!proc_bootinfo) {
                pr_err("Failed to create proc entry\n");
                ret = -EAGAIN;
                goto fail_proc_bootinfo;
        }

	return 0;
fail_proc_bootinfo:
	device_unregister(hwid_dev);
fail_free_hwid_dev:
	kfree(hwid_dev);
fail_class_unregister:
	class_unregister(&hwid_class);

	return ret;
}
module_init(harman_hwid_sysfs_init);

static void __exit harman_hwid_sysfs_exit(void)
{
	device_unregister(hwid_dev);
	class_unregister(&hwid_class);
        proc_remove(proc_bootinfo);
}
module_exit(harman_hwid_sysfs_exit);

MODULE_AUTHOR("Vikram N <vikramnarayanarao20@harman.com>");
MODULE_DESCRIPTION("Module to export Harman Hardware ID via sysfs");
MODULE_LICENSE("GPL");
