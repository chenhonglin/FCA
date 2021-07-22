#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/seq_file.h>

#if defined(CONFIG_BJEVN60_ED2_BOARD)
#define K_CMD_LINE_FCID     32
#define K_CMD_LINE_CAMID    28
#define K_CMD_LINE_HWID     10
#else
#define K_CMD_LINE_FCID     16
#define K_CMD_LINE_HWID     6
#endif
#define HWID_STR_CPY        2
#if defined(CONFIG_BJEVN60_ED2_BOARD)
#define CAMID_STR_CPY       2
#endif
#define FUNCTION_STR_CPY    4
#define VAR_HEAD_SIZE       9
#define BOOTINFO_SIZE       64
#define ABL_DATE            20
#define ABL_USER            24
#define ABL_GIT             47
#define ABL_VER             5

enum sample_id {
#if defined(CONFIG_BJEVN60_ED2_BOARD)
	ED2 = 0,
	ED3,
	DV,
	DVX,
	PV,
	PV_B,
#else
	B0 = 0,
	B1,
	C0,
	B2,
	C1,
	C2,
	C3,
#endif
	SAMPLE_UNKNOWN,
};

enum variant_id {
	VARIANT_NULL = 0,
	X108,
	X111,
	X139,
	X140,
	X099,
	X112,
	X132,
	VARIANT_UNKNOWN,
};

static char *sample_id_name[SAMPLE_UNKNOWN + 1] = {
#if defined(CONFIG_BJEVN60_ED2_BOARD)
	[ED2] = "ED2",
	[ED3] = "ED3",
	[DV] =  "DV",
	[DVX] = "DVX",
	[PV] =  "PV",
	[PV_B] = "PV_B",
#else
	[B0] = "B0",
	[B1] = "B1",
	[C0] = "C0",
	[B2] = "B2",
	[C1] = "C1",
	[C2] = "C2",
	[C3] = "C3",
#endif
	[SAMPLE_UNKNOWN] = "UNKNOWN",
};

static char *variant_id_name[VARIANT_UNKNOWN + 1] = {
	[VARIANT_NULL] = "NULL",
	[X108] = "V2MM_X108",
	[X111] = "V2HH_X111",
	[X139] = "V2HT_X139",
	[X140] = "V2HT_X140",
	[X099] = "V2MH_X099",
	[X112] = "V2MH_X112",
	[X132] = "V2MH_X132",
	[VARIANT_UNKNOWN] = "UNKNOWN",
};

static char *function_id_name[3] = {"Yes","No","UNKNOWN"};

struct func_id {
	unsigned char PF_SATURN:1 ;   /* bit 0 Saturn */
	unsigned char PF_TMC:1 ;      /* bit 1 TMC */
	unsigned char PF_SXM:1 ;      /* bit 2 SXM */
	unsigned char PF_4CH_AMP:1 ;  /* bit 3 4ch AMP */
	unsigned char PF_7CH_AMP:1 ;  /* bit 4 7ch AMP */
	unsigned char PF_RVC:1 ;      /* bit 5 RVC */
	unsigned char PF_SVC:1 ;      /* bit 6 SVC */
	unsigned char PF_AVM:1 ;      /* bit 7 AVM */
	#if defined(CONFIG_BJEVN60_ED2_BOARD)
	unsigned char PF_A2B:1 ;      /* bit 1 A2B */
	#else
	unsigned char PF_A2B:1 ;      /* bit 0 A2B */
	#endif
	unsigned char PF_DVR:1 ;      /* bit 1 DVR */
} func_id_init = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static char *vendor_name = "Harman International";
static char *emod_id;
static char sample_variant_id[HWID_STR_CPY + 1] = {0};
static char function_id[FUNCTION_STR_CPY + 1] = {0};
#if defined(CONFIG_BJEVN60_ED2_BOARD)
static char camera_id[CAMID_STR_CPY + 1] = {0};
#endif
static long int id;
static long int fid;
unsigned int hw_sample_id = 0;
unsigned int hw_variant_id = 0;
unsigned int byte8 = 0;
unsigned int byte9 = 0;
#if defined(CONFIG_BJEVN60_ED2_BOARD)
unsigned int byte16 = 0;
unsigned int byte17 = 0;
#endif
static char bootinfo[BOOTINFO_SIZE + 2] = {0};
static char abl_date[ABL_DATE] = {0};
static char abl_user[ABL_USER] = {0};
static char abl_git[ABL_GIT] = {0};
static char abl_ver[ABL_VER] = {0};
static struct func_id *func_variant_id = &func_id_init;

char *harman_get_board_vendor(void)
{
    return vendor_name;
}
EXPORT_SYMBOL_GPL(harman_get_board_vendor);

char *harman_get_board_name(void)
{
    return variant_id_name[hw_variant_id];
}
EXPORT_SYMBOL_GPL(harman_get_board_name);


char *harman_get_board_revision(void)
{
    return sample_id_name[hw_sample_id];
}
EXPORT_SYMBOL_GPL(harman_get_board_revision);

char *harman_get_emodule_revision(void)
{
    return emod_id;
}
EXPORT_SYMBOL_GPL(harman_get_emodule_revision);

int bootinfo_proc_show(struct seq_file *m, void *v)
{
        seq_printf(m, bootinfo);
        return 0;
}
EXPORT_SYMBOL_GPL(bootinfo_proc_show);

char *harman_get_abl_date(void)
{
    return abl_date;
}
EXPORT_SYMBOL_GPL(harman_get_abl_date);

char *harman_get_abl_user(void)
{
    return abl_user;
}
EXPORT_SYMBOL_GPL(harman_get_abl_user);

char *harman_get_abl_git(void)
{
    return abl_git;
}
EXPORT_SYMBOL_GPL(harman_get_abl_git);

char *harman_get_abl_ver(void)
{
    return abl_ver;
}
EXPORT_SYMBOL_GPL(harman_get_abl_ver);

char *harman_get_saturn_variant(void)
{
	if (func_variant_id->PF_SATURN == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_SATURN == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_saturn_variant);

char *harman_get_tmc_variant(void)
{
	if (func_variant_id->PF_TMC == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_TMC == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_tmc_variant);

char *harman_get_sxm_variant(void)
{
	if (func_variant_id->PF_SXM == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_SXM == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_sxm_variant);

char *harman_get_AMP4_variant(void)
{
	if (func_variant_id->PF_4CH_AMP == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_4CH_AMP == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_AMP4_variant);

char *harman_get_AMP7_variant(void)
{
	if (func_variant_id->PF_7CH_AMP == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_7CH_AMP == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_AMP7_variant);

char *harman_get_rvc_variant(void)
{
	if (func_variant_id->PF_RVC == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_RVC == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_rvc_variant);

char *harman_get_svc_variant(void)
{
	if (func_variant_id->PF_SVC == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_SVC == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_svc_variant);

char *harman_get_avm_variant(void)
{
	if (func_variant_id->PF_AVM == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_AVM == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_avm_variant);

char *harman_get_A2B_variant(void)
{
	if (func_variant_id->PF_A2B == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_A2B == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_A2B_variant);

char *harman_get_DVR_variant(void)
{
	if (func_variant_id->PF_DVR == 1)
		return function_id_name[0];
	else if (func_variant_id->PF_DVR == 0)
		return function_id_name[1];
	else
		return function_id_name[2];
}
EXPORT_SYMBOL_GPL(harman_get_DVR_variant);

static void harman_hwid_read(char * fn_add) {
	char *temp_ptr = fn_add + VAR_HEAD_SIZE;
	strncpy(bootinfo, temp_ptr, BOOTINFO_SIZE);
	bootinfo[BOOTINFO_SIZE] = '\n';
	bootinfo[BOOTINFO_SIZE + 1] = '\0';
	strncpy(sample_variant_id, temp_ptr + K_CMD_LINE_HWID, HWID_STR_CPY);
	strncpy(function_id, temp_ptr + K_CMD_LINE_FCID, FUNCTION_STR_CPY);
#if defined(CONFIG_BJEVN60_ED2_BOARD)
	strncpy(camera_id, temp_ptr + K_CMD_LINE_CAMID, CAMID_STR_CPY);
#endif
	sample_variant_id[HWID_STR_CPY] = '\0';
	function_id[FUNCTION_STR_CPY] = '\0';
#if defined(CONFIG_BJEVN60_ED2_BOARD)
	camera_id[CAMID_STR_CPY] = '\0';
#endif
	if (kstrtol(sample_variant_id, 16, &id) != 0) {
		printk(KERN_ERR "kstrtol failed\n");
	}
	hw_sample_id = (id & 0xF0) >> 4;
	hw_variant_id = id & 0x0F;

	if (kstrtol(function_id, 16, &fid) != 0) {
		printk(KERN_ERR "kstrtol failed\n");
	}

	byte8 = (fid & 0xFF00) >> 8;
	byte9 = fid & 0x00FF;
#if defined(CONFIG_BJEVN60_ED2_BOARD)
	byte16 = (fid & 0xFF00) >> 8;
	byte17 = fid & 0x00FF;
#endif

	func_variant_id->PF_SATURN = (byte8 & 0x1) >> 0;      /* bit 0 Saturn */
	func_variant_id->PF_TMC = (byte8 & 0x2) >> 1;         /* bit 1 TMC */
	func_variant_id->PF_SXM = (byte8 & 0x4) >> 2;         /* bit 2 SXM */
	func_variant_id->PF_4CH_AMP = (byte8 & 0x8) >> 3;     /* bit 3 4ch AMP */
	func_variant_id->PF_7CH_AMP = (byte8 & 0x10) >> 4;    /* bit 4 7ch AMP */
	func_variant_id->PF_RVC = (byte8 & 0x20) >> 5;        /* bit 5 RVC */
	func_variant_id->PF_SVC = (byte8 & 0x40) >> 6;        /* bit 6 SVC */
	func_variant_id->PF_AVM = (byte8 & 0x80) >> 7;        /* bit 7 AVM */

	#if defined(CONFIG_BJEVN60_ED2_BOARD)
	func_variant_id->PF_A2B = (byte17 & 0x2) >> 1;         /* bit 1 A2B */
	#else
	func_variant_id->PF_A2B = (byte9 & 0x1) >> 0;         /* bit 0 A2B */
	#endif
	func_variant_id->PF_DVR = (byte9 & 0x2) >> 1;         /* bit 1 DVR */
}

static int __init get_abl_date(char *line)
{
	strlcpy(abl_date, line, sizeof(abl_date));
	return 1;
}

__setup("HARMAN.ABL.DATE=", get_abl_date);

static int __init get_abl_user(char *line)
{
	strlcpy(abl_user, line, sizeof(abl_user));
	return 1;
}

__setup("HARMAN.ABL.USER=", get_abl_user);

static int __init get_abl_git(char *line)
{
	strlcpy(abl_git, line, sizeof(abl_git));
	return 1;
}

__setup("HARMAN.ABL.GIT=", get_abl_git);

static int __init get_abl_ver(char *line)
{
	strlcpy(abl_ver, line, sizeof(abl_ver));
	return 1;
}

__setup("HARMAN.ABL.VER=", get_abl_ver);

static int __init harman_hwid_module_entry (void) {
	static char *n_add = NULL;
	if( (n_add = strstr( saved_command_line, "bootinfo=")) ) {
			harman_hwid_read(n_add);
	}
	else {
		printk(KERN_ERR "Hardware and Varient ID not found");
		return -EPERM;
	}

	return 0;
}

static void __exit harman_hwid_module_exit (void) {
        pr_info("Module un initialized successfully \n");
}

arch_initcall(harman_hwid_module_entry);
module_exit(harman_hwid_module_exit);


MODULE_AUTHOR("vivek Pernamitta <vivek.pernamitta@harman.com>");
MODULE_DESCRIPTION("Harman Command line based H/W ID reader");
MODULE_LICENSE("GPL");

