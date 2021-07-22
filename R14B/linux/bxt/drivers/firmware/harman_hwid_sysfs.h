/*
 * linux/drivers/firmware/harman-hwid_sysfs.h
 * Definitions for Harman command line based Hardware ID reader for GWM H/W
 *  Copyright (C) 2016 Harman International Ltd,
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


extern char *harman_get_board_name(void);
extern char *harman_get_board_vendor(void);
extern char *harman_get_board_revision(void);
extern char *harman_get_emodule_revision(void);
extern int bootinfo_proc_show(struct seq_file *m, void *v);
extern char *harman_get_abl_date(void);
extern char *harman_get_abl_user(void);
extern char *harman_get_abl_git(void);
extern char *harman_get_abl_ver(void);
extern char *harman_get_saturn_variant(void);
extern char *harman_get_tmc_variant(void);
extern char *harman_get_sxm_variant(void);
extern char *harman_get_AMP4_variant(void);
extern char *harman_get_AMP7_variant(void);
extern char *harman_get_rvc_variant(void);
extern char *harman_get_svc_variant(void);
extern char *harman_get_avm_variant(void);
extern char *harman_get_A2B_variant(void);
extern char *harman_get_DVR_variant(void);

