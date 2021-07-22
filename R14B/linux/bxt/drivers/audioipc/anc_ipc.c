/*
 * Author: Kun.You <Kun.You@harman.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <asm/device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <sound/soc.h>
#include <linux/mutex.h>
#include "anc_ipc.h"
#include "../../sound/soc/intel/common/sst-ipc.h"
#include "../../sound/soc/intel/skylake/skl-sst-ipc.h"
#include "../../include/linux/audioconf.h"

/* get_info_audiosystem_conf returns
* 0 for Internal AMP Processing,
* 1 for External AMP Processing and
* -1 for Unknown
*/
extern char *get_info_audiosystem_conf(void);
#define CHECK_CPD_ID
#define CHECK_EXT_VIRIANT
#define CAN_PARM_OPTION1

#define DEVICE_NAME "anc_can_ipc"
#define ANC_CAN_CHANNEL "/dev/ipc11"
#define CPD_PATH "/vendor/harman/config/"
#define SUFFIX 				"_ANC_TUNING.SET"
#define ERROR					(-1)
#define OK					(0)
#define CAN_MSG_OFFSET		(1)
#define CAN_BUFFER_SIZE				(16)
#define CAN_MESSAGE_SIZE			(CAN_BUFFER_SIZE - CAN_MSG_OFFSET)
#define DTC_INFO_SIZE			(3)
#define MIC_INFO_SIZE			(3)
#define DAIG_INFO_SIZE			(10)
#define CAL_INFO_SIZE			(6)
#define CALID_INFO_SIZE			(5)
#define ANCEN_INFO_SIZE			(4)
#define SUM_INFO_SIZE			(7)
#define VER_INFO_SIZE			(6)
#define ANCSTATUS_INFO_SIZE		(4)
#define DVRERRCNT_INFO_SIZE		(4)
#define RUN_PERIOD				(3000)
#define MSG_PRINT_PERIOD		(50)
#define MSG_SAME				(1)
#define MSG_NOT_SAME			(0)
#define CPD_ID_VALID			(0)
#define CPD_ID_INVALID			(1)
#define FILE_NAME_MAX			(64)
#define INTERFACA_VER			((uint8_t)0x01)
#define COMMON_XTPMSG_HEAD_SIEZ	(sizeof(struct common_xtpmsg) - COMMON_XTP_BYTE)

enum {
	RPM_MASK_BIT = 0,
	ENABLE_MASK_BIT,
	WIN_DRV_MASK_BIT,
	WIN_PSG_MASK_BIT,
	WIN_RL_MASK_BIT,
	WIN_RR_MASK_BIT,
	DRV_DOOR_MASK_BIT,
	CYL_DEAT_MASK_BIT,
	CAN_PARAM_MAX_NUM
};


extern struct sst_generic_ipc* ipc_handle;
#ifdef CONFIG_SND_SOC_FCA_ALSA
extern bool get_anc_ipc_sync(void);
#endif
static struct file *ioc_fd = 0;
static struct task_struct * iocReaderThread = NULL;
struct cdev anc_cdev;
static struct class *anc_cls;
//static struct skl_ipc_large_config_msg msg;
static int majorNumber;
static int tuningInitFlag;
static unsigned int run_count;
static unsigned int msg_print_count;
static uint32_t s_cpd;
static uint32_t s_external_flag; //0 non external 1 external
#ifdef CHECK_CPD_ID
static int32_t dtc_flag;
#endif
static int32_t s_recv_e3; //0 no recv E3 msg, 1 recv E3 msg
static struct anc_diag s_anc_diag;

extern int skl_ipc_set_large_config(struct sst_generic_ipc *ipc,
struct skl_ipc_large_config_msg *msg, u32 *param);
static int IOCDataReader(void *data);

static void sendmic_delaywork_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(sendmic_delaywq, sendmic_delaywork_func);

static int anc_open(struct inode*inode,struct file * file)
{
	pr_info("[ANC_CAN_DEVICES] anc_open\n");
	return OK;
}

static int anc_close(struct inode *inode, struct file *file)
{
	pr_info("[ANC_CAN_DEVICES] anc_close\n");
	return OK;
}


static ssize_t anc_read(struct file *filp,char __user *buff, size_t count, loff_t *offp)
{
	pr_info("[ANC_CAN_DEVICES] anc_read\n");
	return OK;
}

static ssize_t anc_write(struct file *filp,const char __user *buf,size_t count, loff_t *f_pos)
{
	pr_info("[ANC_CAN_DEVICES] anc_write\n");
	return OK;
}

static long anc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;

	if (_IOC_TYPE(cmd) != ANC_IOC_MAGIC) {
		return ERROR;
	}

	if (_IOC_NR(cmd) > ANC_IOCMAXNR) {
		return ERROR;
	}
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok(VERIFY_WRITE, (char *)arg, _IOC_SIZE(cmd));
	}else if (_IOC_DIR(cmd) & _IOC_WRITE){
		err = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	}

	if (err){
		return err;
	}

	switch (cmd) {
		case ANC_IOCSTART:
			pr_info("[ANC_CAN_DEVICES] anc start\n");
			break;
		case ANC_IOCSTOP:
			pr_info("[ANC_CAN_DEVICES] anc stop\n");
			break;
		case ANC_IORGETCPD:
			if (copy_to_user((int32_t*) arg, &s_cpd, sizeof(s_cpd)))
				err = -EFAULT;
			pr_debug("[ANC_CAN_DEVICES] GET_CPD,%d\n",s_cpd);
			break;
		case ANC_IORGETVARID:
			if (copy_to_user((int32_t*) arg, &s_external_flag, sizeof(s_external_flag)))
				err = -EFAULT;
			pr_debug("[ANC_CAN_DEVICES] GET_VARID %d\n",s_external_flag);
			break;
		case ANC_IOCSENDMIC:
			/* Send mic status and cpdid after 3s */
			if(s_external_flag == 0)
				schedule_delayed_work(&sendmic_delaywq, msecs_to_jiffies(3000));
			break;
		case ANC_IOCSETDANG:
			if(s_external_flag == 0) {
				pr_info("[ANC_CAN_DEVICES] store anc diag\n");
			    if(copy_from_user(&s_anc_diag, (void *)arg, sizeof(struct anc_diag)))
					err = -EFAULT;
			}
			break;
		default:
			err =  ERROR;
	}

	return err;
}

static struct file_operations anc_fops = {
	.owner			= THIS_MODULE,
	.open			= anc_open,
	.release		= anc_close,
	.read			= anc_read,
	.write			= anc_write,
	.unlocked_ioctl	= anc_ioctl,
};

static int check_msg_same(u8 * msg1, u8 * msg2, u32 length)
{
	u32 sameflg = MSG_SAME;
	u32 i;
	for (i = 0; i < length; i++) {
		if (msg1[i] != msg2[i]) {
			sameflg = MSG_NOT_SAME;
			break;
		}
	}
	return sameflg;
}
#ifdef CHECK_CPD_ID
static int check_cpdid(u16 cpdid)
{
	int ret  = OK;
	int len;
	static struct file * cpd_fd;
	char cpd_file_name[FILE_NAME_MAX];
	memset(cpd_file_name,0x00,sizeof(cpd_file_name));
	len = snprintf(cpd_file_name,FILE_NAME_MAX,CPD_PATH"%d"SUFFIX,cpdid);
	if (len >= FILE_NAME_MAX) {
		pr_err("[ANC_CAN_DEVICES] cpd_file_name file too long. \n");
		ret = -ENAMETOOLONG;
		return ret;
	}
	pr_debug("[ANC_CAN_DEVICES] cpd_file_name %s\n",cpd_file_name);
	cpd_fd = filp_open(cpd_file_name,
				O_RDONLY | O_NONBLOCK, 0);
	if (IS_ERR(cpd_fd)) {
		pr_debug("[ANC_CAN_DEVICES] cpd mismatch\n");
		return CPD_ID_INVALID;
	} else {
		pr_debug("[ANC_CAN_DEVICES] cpd match\n");
		filp_close(cpd_fd,NULL);
		return CPD_ID_VALID;
	}
}
static int send_dtc_info(int32_t iFlag)
{
	mm_segment_t originalFs;
	loff_t offset;
	int ret = OK;
	char dtc_info[DTC_INFO_SIZE];
	memset(dtc_info,0,sizeof(dtc_info));
	dtc_info[0] = 0xE0;
	dtc_info[1] = 0x01;
	if (iFlag == CPD_ID_VALID)
		dtc_info[2] = 0x00;
	else
		dtc_info[2] = 0x01;

	//send msg to ioc
	originalFs = get_fs();
	set_fs(get_ds());
	offset = 0;
	ret = vfs_write(ioc_fd, dtc_info,
		DTC_INFO_SIZE*sizeof(char), &offset);

	set_fs(originalFs);
	pr_debug("[ANC_CAN_DEVICES] sent dtc info ret %d\n",ret);
	if (ret < 0) {
		pr_debug("[ANC_CAN_DEVICES] sent dtc info failed %d\n",ret);
		msleep(10);
	}
	return ret;
}
#endif

#ifdef CHECK_EXT_VIRIANT
static int check_hu_external_variant(u32 *is_extern_variant)
{
	int ret = OK;
	long varintid;
	char * pcvarint = NULL;
	char cvarint[10];
	pcvarint = get_info_audiosystem_conf();
	memset(&cvarint[0],0,10);
	memcpy(&cvarint[0],pcvarint,ANDROIDBOOT_AUDIOSYSTEM_CONF);
	pr_info("[ANC_CAN_DEVICES] cvarint %s\n",cvarint);
	ret = kstrtol(pcvarint,10,&varintid);
	if (ret == OK) {
		pr_info("[ANC_CAN_DEVICES] varintid %ld \n",varintid);
		//external
		if (varintid == 1) {
			*is_extern_variant = 1;
			pr_info("[ANC_CAN_DEVICES] external amp\n");
			ret = OK;
		} // intelnal
		else if (varintid == 0) {
			*is_extern_variant = 0;
			pr_info("[ANC_CAN_DEVICES] internal amp\n");
			ret = OK;
		}
		// error
		else {
			*is_extern_variant = VARID_DEFAULT;
			pr_info("[ANC_CAN_DEVICES] unknown amp\n");
			ret = ERROR;
		}
	}
	else {
		ret = ERROR;
	}
	return ret;
}
#endif

bool cpdid_is_valid(uint16_t cpdid)
{
	return cpdid >= 1 && cpdid < 0xEEEE && cpdid != 0xDF00;
}

static int get_mic_status(u8 *mic_status)
{
	int ret;
	struct skl_ipc_large_config_msg mic_msg;
	t_AM_Params amParams;
	tMicParam *micParam;
	size_t rx_bytes;
	u32 i;
	u8 ms_max = 0;  /* max mic status value */

	mic_msg.module_id = HIFI_MODULE_ID;
	mic_msg.instance_id = 0;
	mic_msg.large_param_id = 0x4D; /* 4D == 'M', mic status */
	mic_msg.param_data_size = sizeof(t_AM_Params);

	memset(&amParams, 0, sizeof(t_AM_Params));
	ret = skl_ipc_get_large_config(ipc_handle, &mic_msg,
					(u32 *)&amParams, NULL, 0, &rx_bytes);
	if(ret < 0) {
		pr_err("[ANC_CAN_DEVICES] skl_ipc_get_large_config err %d\n", ret);
	}
	if(amParams.InstID != 0x4D)
		return -EINVAL;
	micParam = (tMicParam *)amParams.Data;
	pr_debug("[ANC_CAN_DEVICES] mic total: %d, active: %d, mic status: %x\n",
				(int)micParam->totalMic, (int)micParam->activeMic, (int)micParam->micStatus);

	/* mic status is an LSB byte */
	for(i=0; i<micParam->totalMic; i++)
		ms_max |= 1 << i;
	if(micParam->micStatus > ms_max)
		return -EINVAL;

	*mic_status = micParam->micStatus;
	return 0;
}

static int send_mic_info(void)
{
	mm_segment_t originalFs;
	loff_t offset;
	char mic_status = 0xFF;
	char mic_info[MIC_INFO_SIZE];
	int ret;
	int wait_ms = 0;

	/* clean e3 msg flag */
	s_recv_e3 = 0;

	ret = get_mic_status(&mic_status);
	if(ret != 0) {
		pr_err("[ANC_CAN_DEVICES] get mic status faild, %d", ret);
		return ret;
	}

send_mic:
	pr_info("[ANC_CAN_DEVICES] send mic status: %d\n", mic_status);
	wait_ms = 0;
	memset(mic_info, 0, sizeof(mic_info));
	mic_info[0] = 0xE2;
	mic_info[1] = 0x01;
	mic_info[2] = mic_status;

	/* send mic msg to ioc */
	originalFs = get_fs();
	set_fs(get_ds());
	offset = 0;

	if(ioc_fd == NULL) {
		pr_err("[ANC_CAN_DEVICES] send mic status faild, ioc fd error");
		return -EINVAL;
	}

	ret = vfs_write(ioc_fd, mic_info,
		sizeof(mic_info), &offset);

	set_fs(originalFs);
	pr_debug("[ANC_CAN_DEVICES] sent mic info ret %d\n", ret);
	if (ret < 0) {
		pr_err("[ANC_CAN_DEVICES] sent mic info failed %d\n", ret);
		msleep(10);
	}

	do {
		msleep_interruptible(10);
		wait_ms += 10;
		if(wait_ms > 500) {
			pr_info("[ANC_CAN_DEVICES] wait ioc reply, time out ...\n");
			goto send_mic;
		}
	} while(s_recv_e3 == 0);

	return ret;
}

static void sendmic_delaywork_func(struct work_struct *work)
{
	int ret;

	/* wait ipc handle and ioc_fd ready */
	while(!ipc_handle) {
		msleep(100);
	}

	while(!get_anc_ipc_sync()) {
		msleep(100);
	}

	while(!ioc_fd) {
		msleep(100);
	}

#ifdef CHECK_CPD_ID
	ret = send_dtc_info(dtc_flag);
	if(ret < 0)
            pr_err("[ANC_CAN_DEVICES] send cpd mismatch info to ioc failed, ret: %d\n", ret);
        else
            pr_info("[ANC_CAN_DEVICES] send cpd mismatch info to ioc success, ret: %d\n", ret);
#endif

        if(s_cpd == 0x6100 || s_cpd == 0x6101) {
            ret = send_mic_info();
            if(ret < 0)
                pr_info("[ANC_CAN_DEVICES] send mic status failed, ret: %d\n", ret);

            pr_info("[ANC_CAN_DEVICES] send mic status success, ret: %d\n", ret);
        }

	return ;
}

static int send_diag_info(void *buf, int len)
{
	mm_segment_t originalFs;
	loff_t offset;
	int ret = OK;

	if(buf == NULL || len > DAIG_INFO_SIZE)
		return -1;

	//send msg to ioc
	originalFs = get_fs();
	set_fs(get_ds());
	offset = 0;
	ret = vfs_write(ioc_fd, buf, len, &offset);

	set_fs(originalFs);
	if (ret < 0) {
		msleep(10);
	}
	return ret;
}

static int __init anc_init(void)
{
	dev_t dev_id;
	int ret;
	#ifdef CHECK_EXT_VIRIANT
	u32 external_flag = 0; //0 non external 1 external
	#endif

	pr_info("[ANC_CAN_DEVICES] anc init\n");
	s_cpd = CPD_DEFAULT;
	s_external_flag = VARID_DEFAULT;

	ioc_fd = filp_open(ANC_CAN_CHANNEL, O_RDWR | O_NONBLOCK, 0);
	if (IS_ERR(ioc_fd)) {
		ioc_fd = 0;
		pr_debug("[ANC_CAN_DEVICES] open ipc11 failed\n");
	}
	#ifdef CHECK_EXT_VIRIANT
	ret = check_hu_external_variant(&external_flag);
	#endif

	tuningInitFlag = 0;
	#ifdef CHECK_EXT_VIRIANT
	if ((ret == OK) && (external_flag == 1)) {
		// exeternal_variant don't create the thread for process ioc msg
		s_external_flag = 1;
	} else {
	#endif
		iocReaderThread = kthread_run(IOCDataReader, 0, "IOCDataReader");
	#ifdef CHECK_EXT_VIRIANT
		s_external_flag = 0;
	}
	#endif

	#ifdef CHECK_EXT_VIRIANT
	if ((!IS_ERR(iocReaderThread)) || (s_external_flag == 1)) {
	#else
	if (!IS_ERR(iocReaderThread)) {
	#endif
		ret = alloc_chrdev_region(&dev_id, 0, 1, DEVICE_NAME);
		if (ret){
			pr_debug("[ANC_CAN_DEVICES] Unable to register anc can device node %d\n",ret);
			return ERROR;
		}

		majorNumber = MAJOR(dev_id);

		cdev_init(&anc_cdev, &anc_fops);

		ret = cdev_add(&anc_cdev, dev_id, 1);
		if (ret) {
			pr_debug("[ANC_CAN_DEVICES] Unable to add anc can device %d\n",ret);
			return ERROR;
		}

		anc_cls = class_create(THIS_MODULE, DEVICE_NAME);
		if (IS_ERR(anc_cls)) {
			pr_debug("[ANC_CAN_DEVICES] Failed to create deivce class\n");
			return ERROR;
		}

		device_create(anc_cls, NULL, dev_id, NULL, DEVICE_NAME);
	}else {
		pr_debug("[ANC_CAN_DEVICES] failed to create IOC msg process thread\n");
		return ERROR;
	}

	pr_info("[ANC_CAN_DEVICES] anc init done\n");
	return OK;
}

static void __exit anc_exit(void)
{
	int ret = OK;
	if (!IS_ERR(iocReaderThread)) {
		kthread_stop(iocReaderThread);
		pr_info("[ANC_CAN_DEVICES] stop thread %d\n",ret);
	}
	msleep(100);

	if(s_external_flag != 1)
	cancel_delayed_work_sync(&sendmic_delaywq);

	device_destroy(anc_cls, MKDEV(majorNumber, 0));
	class_unregister(anc_cls);
	class_destroy(anc_cls);
	unregister_chrdev_region(MKDEV(majorNumber, 0), 1);
}

static bool widgets_on(struct snd_soc_card *card)
{
	struct snd_soc_dapm_widget *w;
	int widgets_on = 0;

	mutex_lock(&card->dapm_mutex);
	list_for_each_entry(w, &card->widgets, list) {
		if (!strcmp(w->name, "dirana_out") && w->power == 1)
			widgets_on = 1;
	}
	mutex_unlock(&card->dapm_mutex);

	if (widgets_on == 1)
		return true;
	else
		return false;
}

static int send_sklmsg(struct sklmsg *sklmsg, uint32_t config_id)
{
	int ret;
	struct skl_ipc_large_config_msg msg;
	size_t rx_bytes;

	msg.module_id = HIFI_MODULE_ID;
	msg.instance_id = 0;
	msg.large_param_id = config_id;
	msg.param_data_size = sizeof(struct sklmsg);

	ret = skl_ipc_set_large_config(ipc_handle, &msg, (uint32_t *)sklmsg);
	if(ret < 0) {
		pr_err("[ANC_CAN_DEVICES] set large config faild\n");
		return -1;
	}

	msleep(10);
	memset(sklmsg, 0, sizeof(struct sklmsg));
	ret = skl_ipc_get_large_config(ipc_handle, &msg,
					(uint32_t *)sklmsg, NULL, 0, &rx_bytes);
	if(ret < 0) {
		pr_err("[ANC_CAN_DEVICES] get large config faild\n");
		return -1;
	}
	return 0;
}

static int get_anc_status(uint8_t *anc_status)
{
	int ret;
	uint8_t ret_byte0;
	struct sklmsg sklmsg;
	uint8_t read_size;

	if(ipc_handle == NULL || get_anc_ipc_sync() == 0)
		return -1;

	read_size = 4;

	sklmsg.xtpmsg.com_xtpmsg.cmd_id = 0x6a;
	sklmsg.xtpmsg.com_xtpmsg.dev_id = 0;
	sklmsg.xtpmsg.com_xtpmsg.core_id = 0;
	sklmsg.xtpmsg.com_xtpmsg.instance_id = 0;
	sklmsg.xtpmsg.com_xtpmsg.blockid_h = 0x00;
	sklmsg.xtpmsg.com_xtpmsg.blockid_l = 0x3f;
	sklmsg.xtpmsg.com_xtpmsg.subblock_id = 0;
	sklmsg.xtpmsg.com_xtpmsg.off_bytes[0] = 0x00;
	sklmsg.xtpmsg.com_xtpmsg.off_bytes[1] = 0x00;
	sklmsg.xtpmsg.com_xtpmsg.off_bytes[2] = 0x00;
	sklmsg.xtpmsg.com_xtpmsg.off_bytes[3] = 0x04;
	sklmsg.xtpmsg.com_xtpmsg.parm.read_size = read_size;
	sklmsg.len = 12;
	sklmsg.msg_type = SKLMSG_TYPE_COMMON;

	/* 0x43 = 'C' common xtp msg */
	ret = send_sklmsg(&sklmsg, 0x43);
	if(ret != 0) {
		pr_err("[ANC_CAN_DEVICES] send common skl msg faild\n");
		return -1;
	}

	if(sklmsg.len != (COMMON_XTPMSG_HEAD_SIEZ + read_size)) {
		pr_err("[ANC_CAN_DEVICES] read xtp command faild, ret len: %u\n", (uint32_t)sklmsg.len);
		return -1;
	}

	ret_byte0 = sklmsg.xtpmsg.com_xtpmsg.parm.ret_bytes[0];
	if(ret_byte0 > 1) {
		pr_err("[ANC_CAN_DEVICES] read anc status faild %u\n", (uint32_t)ret_byte0);
		return -1;
	}

	*anc_status = ret_byte0;
	return 0;
}

static int get_diver_cnt(uint8_t *dvr_cnt, uint8_t quad_id)
{
	int ret;
	int i;
	struct sklmsg sklmsg;
	uint8_t read_size;
	uint8_t (*orders)[4];
	uint32_t order;
	uint64_t dvrcnt = 0;

	if(ipc_handle == NULL || get_anc_ipc_sync() == 0)
		return -1;

	read_size = DVR_ORDER_NUM * 4;

	sklmsg.xtpmsg.com_xtpmsg.cmd_id = 0x6a;
	sklmsg.xtpmsg.com_xtpmsg.dev_id = 0;
	sklmsg.xtpmsg.com_xtpmsg.core_id = 0;
	sklmsg.xtpmsg.com_xtpmsg.instance_id = 0;
	sklmsg.xtpmsg.com_xtpmsg.blockid_h = 0x00;
	sklmsg.xtpmsg.com_xtpmsg.blockid_l = 0x3f;
	sklmsg.xtpmsg.com_xtpmsg.subblock_id = 0;
	sklmsg.xtpmsg.com_xtpmsg.off_bytes[0] = 0x00;
	sklmsg.xtpmsg.com_xtpmsg.off_bytes[1] = 0x00;
	sklmsg.xtpmsg.com_xtpmsg.off_bytes[2] = 0x00;
	switch(quad_id) {
		case 1:
			sklmsg.xtpmsg.com_xtpmsg.off_bytes[3] = 0x10;
			break;
		case 2:
			sklmsg.xtpmsg.com_xtpmsg.off_bytes[3] = 0x20;
			break;
		case 3:
			sklmsg.xtpmsg.com_xtpmsg.off_bytes[3] = 0x30;
			break;
		case 4:
			sklmsg.xtpmsg.com_xtpmsg.off_bytes[3] = 0x40;
			break;
		default:
			return -1;
	}
	sklmsg.xtpmsg.com_xtpmsg.parm.read_size = read_size;
	sklmsg.len = 12;
	sklmsg.msg_type = SKLMSG_TYPE_COMMON;

	/* 0x43 = 'C' common xtp msg */
	ret = send_sklmsg(&sklmsg, 0x43);
	if(ret != 0) {
		pr_err("[ANC_CAN_DEVICES] send common skl msg faild\n");
		return -1;
	}

	if(sklmsg.len != (COMMON_XTPMSG_HEAD_SIEZ + read_size)) {
		pr_err("[ANC_CAN_DEVICES] read xtp command faild, ret len: %u\n", (uint32_t)sklmsg.len);
		return -1;
	}

	orders = (order_map)sklmsg.xtpmsg.com_xtpmsg.parm.ret_bytes;
	for(i=0; i<DVR_ORDER_NUM; i++) {
		/* little endian */
		order = ((uint32_t)orders[i][0])			\
				| ((uint32_t)orders[i][1] << 8)		\
				| ((uint32_t)orders[i][2] << 16)	\
				| ((uint32_t)orders[i][3] << 24);
		dvrcnt += order;
		pr_info("[ANC_CAN_DEVICES] quad_id: %u, order %d: %x\n", (uint32_t)quad_id, i, order);
	}

	*dvr_cnt = (dvrcnt > 255) ? 255 : dvrcnt;

	return 0;
}

int canparm_isupdate(tCanParam *parm)
{
	return parm->mask;
}

#define CAN_PARAM_TO_SKLMSG(canparm, sklmsg, cltidh, cltidl, parm_num)									\
	sklmsg->xtpmsg.can_xtpmsg.param[parm_num].ctlid_h = cltidh;											\
	sklmsg->xtpmsg.can_xtpmsg.param[parm_num].ctlid_l = cltidl;											\
	sklmsg->xtpmsg.can_xtpmsg.param[parm_num].param_h = (uint8_t)(((uint16_t)canparm & 0xff00) >> 8);	\
	sklmsg->xtpmsg.can_xtpmsg.param[parm_num].param_l = (uint8_t)((uint16_t)canparm & 0x00ff);			\
	sklmsg->xtpmsg.can_xtpmsg.param[parm_num].dummy1 = 0x00;											\
	sklmsg->xtpmsg.can_xtpmsg.param[parm_num].dummy2 = 0x00;


static int can_parms2sklmsg(tCanParam *parm, struct sklmsg *sklmsg)
{
	int ret = 0;
	int i;
	int update_num = 0;
	uint16_t mask = parm->mask;
	uint16_t mute;
	int mute_set = 0;

	pr_debug("[ANC_CAN_DEVICES] anc mask: %d\n", mask);
	sklmsg->xtpmsg.can_xtpmsg.cmd_id = 0x6d;
	sklmsg->xtpmsg.can_xtpmsg.record_num = 0x06;
	sklmsg->len = 2;
	sklmsg->msg_type = SKLMSG_TYPE_CAN;

	if(!canparm_isupdate(parm))
		return -1;

	for(i=0; i<CAN_PARAM_MAX_NUM; i++)
	{
		if(!(mask & 0x01)) {
			mask = mask >> 1;
			continue;
		}
		switch(i) {
			case RPM_MASK_BIT:
				pr_debug("[ANC_CAN_DEVICES] anc rpm: %x\n", (uint32_t)parm->RPM);
				CAN_PARAM_TO_SKLMSG(parm->RPM, sklmsg, 0x02, 0x00, update_num);
				sklmsg->len += 6;
				update_num ++;
				break;
			case ENABLE_MASK_BIT:
			case DRV_DOOR_MASK_BIT:
				/* the mute param can only be set once */
				if(mute_set) {
					mask = mask >> 1;
					continue;
				}
				mute = !(parm->enable && (!parm->DrvDooAjar));
				pr_debug("[ANC_CAN_DEVICES] anc mute: %x\n", (uint32_t)mute);
				CAN_PARAM_TO_SKLMSG(mute, sklmsg, 0x0a, 0x00, update_num);
				sklmsg->len += 6;
				mute_set = 1;
				update_num ++;
				break;
			case WIN_DRV_MASK_BIT:
				pr_debug("[ANC_CAN_DEVICES] anc win pos drv: %x\n", (uint32_t)parm->winPosDrv);
				CAN_PARAM_TO_SKLMSG(parm->winPosDrv, sklmsg, 0xf0, 0x03, update_num);
				sklmsg->len += 6;
				update_num ++;
				break;
			case WIN_PSG_MASK_BIT:
				pr_debug("[ANC_CAN_DEVICES] anc win pos psg: %x\n", (uint32_t)parm->winPosPsg);
				CAN_PARAM_TO_SKLMSG(parm->winPosPsg, sklmsg, 0xf0, 0x04, update_num);
				sklmsg->len += 6;
				update_num ++;
				break;
			case WIN_RL_MASK_BIT:
				pr_debug("[ANC_CAN_DEVICES] anc win pos rl: %x\n", (uint32_t)parm->winPosRL);
				CAN_PARAM_TO_SKLMSG(parm->winPosRL, sklmsg, 0xf0, 0x05, update_num);
				sklmsg->len += 6;
				update_num ++;
				break;
			case WIN_RR_MASK_BIT:
				pr_debug("[ANC_CAN_DEVICES] anc win pos rr: %x\n", (uint32_t)parm->winPosRR);
				CAN_PARAM_TO_SKLMSG(parm->winPosRR, sklmsg, 0xf0, 0x06, update_num);
				sklmsg->len += 6;
				update_num ++;
				break;
			case CYL_DEAT_MASK_BIT:
				pr_debug("[ANC_CAN_DEVICES] anc win cyl deac: %x\n", (uint32_t)parm->Cyl_Deac);
				CAN_PARAM_TO_SKLMSG(parm->Cyl_Deac, sklmsg, 0xf0, 0x00, update_num);
				sklmsg->len += 6;
				update_num ++;
				break;
			default:
				ret = -1;
				goto end;
		}

		mask = mask >> 1;
	}


	return 0;

end:
	return ret;
}

static int send_can_sklmsg(struct sklmsg *sklmsg)
{
	int ret;
	if(sklmsg == NULL)
		return -1;

	ret = send_sklmsg(sklmsg, 0x02);
	if(ret != 0)
		return -1;

	if(sklmsg->len == 0)
		return -1;

	return 0;
}

static int IOCDataReader(void *data)
{
	mm_segment_t originalFs;
	unsigned char canParamsBuffer[CAN_BUFFER_SIZE];
	unsigned char *pCanMessage;
	tCanParam *ptCanParam;
	loff_t offset;
	int ret;
	int i;
	unsigned char diag_info[DAIG_INFO_SIZE];
	static unsigned int s_read_ioc_fail_count;
	static unsigned int s_open_ioc_fail_count;
	static unsigned int s_can_parameter_fail_count;
	static unsigned int s_can_msg_don_change_count;
	uint8_t anc_status = 0;
	uint8_t last_ancstatus = 0;
	uint8_t quad_id;
	uint8_t diver_errcnt;
	char lastcanParams[CAN_MESSAGE_SIZE];
	struct sklmsg sklmsg;
	uint8_t same_check = 1;
	bool sys_boot = true;
	bool cpdid_valid = false;
	#ifdef CHECK_CPD_ID
	u16 cpd;
	#endif
	s_read_ioc_fail_count = 0;
	s_open_ioc_fail_count = 0;
	s_can_parameter_fail_count = 0;
	s_can_msg_don_change_count = 0;

	struct snd_soc_dai *dai = NULL;
	struct snd_soc_card *card = NULL;

	/* Any sound card machine will be associated with snd-soc-dummy-dai */
	const struct snd_soc_dai_link_component dlc = { .name = "snd-soc-dummy",
							.dai_name = "snd-soc-dummy-dai",
							};

	dai = snd_soc_find_dai(&dlc);
	if(dai == NULL) {
		pr_err("[ANC_CAN_DEVICES] error: dai not found.\n");
		return ERROR;
	}

	card = dai->component->card;
	if(card == NULL) {
		pr_err("[ANC_CAN_DEVICES] error: card not found.\n");
		return ERROR;
	}

	pCanMessage = canParamsBuffer+CAN_MSG_OFFSET;
	ptCanParam = (tCanParam *)pCanMessage;

	memset(lastcanParams,0,sizeof(lastcanParams));

	do {
		if (run_count > RUN_PERIOD) {
			pr_debug("[ANC_CAN_DEVICES] running ... tuningInitFlag = %d\n",
				tuningInitFlag);
			pr_debug("[ANC_CAN_DEVICES] s_open_ioc_fail_count = %d\n",
				s_open_ioc_fail_count);
			pr_debug("[ANC_CAN_DEVICES] s_read_ioc_fail_count = %d\n",
				s_read_ioc_fail_count);
			pr_debug("[ANC_CAN_DEVICES] s_can_parameter_fail_count = %d\n",
				s_can_parameter_fail_count);
			pr_debug("[ANC_CAN_DEVICES] s_can_msg_don_change_count = %d\n",
				s_can_msg_don_change_count);
			run_count = 0;
		}
		run_count++;

		if (ioc_fd == 0) {
			ioc_fd = filp_open(ANC_CAN_CHANNEL,
				O_RDWR /*| O_NONBLOCK*/, 0);
			if (IS_ERR(ioc_fd)) {
				ioc_fd = 0;
			}

			if (ioc_fd == 0) {
				// Wait a while before attempting to open again
				s_open_ioc_fail_count++;
				msleep_interruptible(100);
			}
		} else {
			originalFs = get_fs();
			set_fs(get_ds());
			offset = 0;
			ret = vfs_read(ioc_fd, (char *)canParamsBuffer,
						   CAN_BUFFER_SIZE*sizeof(char), &offset);
			set_fs(originalFs);
			if (ret < 0) {
				s_read_ioc_fail_count++;
				msleep_interruptible(10);
			} else {
				if (msg_print_count > MSG_PRINT_PERIOD) {
					pr_debug("[ANC_CAN_DEVICES] rpm data :");
					for (i = 0; i < CAN_BUFFER_SIZE;i++) {
						pr_debug("[ANC_CAN_DEVICES] %x",(canParamsBuffer[i]) & 0xFF);
					}
					pr_debug("\n");
					msg_print_count = 0;
				}
				msg_print_count++;

				if (tuningInitFlag == 0) {
					#ifdef CHECK_CPD_ID
					cpd = ptCanParam -> CPD;
					s_cpd = (uint32_t) cpd;
					pr_debug("[ANC_CAN_DEVICES] cpd = 0x%x\n",cpd);
					//check cpd id and load xml
					dtc_flag = check_cpdid(cpd);
					if (dtc_flag == CPD_ID_INVALID) {
						pr_debug("[ANC_CAN_DEVICES] cpd %d id not valid\n",cpd);
					}
					#endif
					cpdid_valid = cpdid_is_valid(ptCanParam->CPD);
					tuningInitFlag = 1;
				} else if (tuningInitFlag == 1) {
					if(canParamsBuffer[0] == 0xE1) {
					// check msg if as same as last msg
					ret = check_msg_same(pCanMessage,lastcanParams,CAN_MESSAGE_SIZE);
						if (!same_check || ret == MSG_NOT_SAME) {
#ifdef CONFIG_SND_SOC_FCA_ALSA
						if (get_anc_ipc_sync() != true) {
							pr_info("[ANC_CAN_DEVICES] hifi ipc not ready\n");
							continue;
						}
#endif
							// backup can_msg for compare next time.
							memcpy(lastcanParams,pCanMessage,CAN_MESSAGE_SIZE);

							/* if audio widgets is not power on, do not send any dsp ipc msg
							 * and we need ignore same msg check at next time
							 * audio widgets power on is during system startup or s2r
							 */
							if(!widgets_on(card)) {
								same_check = 0;
								sys_boot = 1;
								continue;
							}

							#ifdef CAN_PARM_OPTION1
							/* set all mask bit */
							ptCanParam->mask = 0xff;
							#else
							/* can parm option 2 */
							if(!same_check)
							{
								ptCanParam->mask = 0xff;
							}
							#endif
							ret = can_parms2sklmsg(ptCanParam, &sklmsg);
							if(ret != 0) {
								pr_err("[ANC_CAN_DEVICES] can parms to sklmsg faild\n");
								continue;
							}
							ret = send_can_sklmsg(&sklmsg);
							if(ret != 0) {
								pr_info("[ANC_CAN_DEVICES] send can parms faild\n");
								s_can_parameter_fail_count++;
								continue;
							}

							same_check = 1;
							pr_debug("[ANC_CAN_DEVICES] send can parms success\n");

							anc_status = ptCanParam->enable == 1 &&							\
										 ptCanParam->DrvDooAjar == 0 &&						\
										 cpdid_valid == true &&								\
										 dtc_flag == CPD_ID_VALID &&						\
										 (ptCanParam->RPM >= 180 && ptCanParam->RPM <= 45000);

							if(sys_boot == 1 || (anc_status != last_ancstatus)) {
								pr_info("[ANC_CAN_DEVICES] anc kpi marker: %s\n",
													anc_status ? "on" : "off");
								last_ancstatus = anc_status;
							}
							if(sys_boot == 1) {
								#ifdef CHECK_CPD_ID
								ret = send_dtc_info(dtc_flag);
								if(ret < 0)
									pr_err("[ANC_CAN_DEVICES] send cpd mismatch info to ioc failed, ret: %d\n", ret);
								else
									pr_info("[ANC_CAN_DEVICES] send cpd mismatch info to ioc success,dtc: %d\n", dtc_flag);
								#endif
								sys_boot = 0;
							}
						} else {
							s_can_msg_don_change_count++;
							//pr_debug("[ANC_CAN_DEVICES] can Msg keep don't update\n");
						}
						} else if(canParamsBuffer[0] == 0xE3) {
							pr_debug("[ANC_CAN_DEVICES] recv e3 from can msg\n");
							s_recv_e3 = 1;
						} else if(canParamsBuffer[0] == 0xE4) {
							pr_info("[ANC_CAN_DEVICES] cpd checksum %u\n", s_anc_diag.sum);
							diag_info[0] = 0xE4;
							diag_info[1] = INTERFACA_VER;
							diag_info[2] = s_anc_diag.sum_resp;
							/* checksum is big-endian */
							diag_info[3] = s_anc_diag.sum >> 24;
							diag_info[4] = s_anc_diag.sum << 8 >> 24;
							diag_info[5] = s_anc_diag.sum << 16 >> 24;
							diag_info[6] = s_anc_diag.sum << 24 >> 24;
							ret = send_diag_info(diag_info, SUM_INFO_SIZE);
							if (ret < 0)
								pr_err("[ANC_CAN_DEVICES] send cpd checksum faild, ret = %d\n", ret);

						} else if(canParamsBuffer[0] == 0xE5) {
							pr_info("[ANC_CAN_DEVICES] cal date,year %u, week %u, pl %u\n",
											(uint32_t)s_anc_diag.date.year,
											(uint32_t)s_anc_diag.date.week,
											(uint32_t)s_anc_diag.date.pl);
							diag_info[0] = 0xE5;
							diag_info[1] = INTERFACA_VER;
							diag_info[2] = s_anc_diag.cal_resp;
							diag_info[3] = s_anc_diag.date.year;
							diag_info[4] = s_anc_diag.date.week;
							diag_info[5] = s_anc_diag.date.pl;
							ret = send_diag_info(diag_info, CAL_INFO_SIZE);
							if(ret < 0)
								pr_err("[ANC_CAN_DEVICES] send calibration date faild, ret = %d\n", ret);
						} else if(canParamsBuffer[0] == 0xE6) {
							pr_info("[ANC_CAN_DEVICES] anc cal id 0x%x\n", s_anc_diag.cal_id);
							diag_info[0] = 0xE6;
							diag_info[1] = INTERFACA_VER;
							diag_info[2] = s_anc_diag.id_resp;
							diag_info[3] = s_anc_diag.cal_id >> 8;
							diag_info[4] = s_anc_diag.cal_id << 8 >> 8;
							ret = send_diag_info(diag_info, CALID_INFO_SIZE);
							if(ret < 0)
								pr_err("[ANC_CAN_DEVICES] send calibration id faild, ret = %d\n", ret);
						} else if(canParamsBuffer[0] == 0xE7) {
							pr_info("[ANC_CAN_DEVICES] cpd software version %u.%u.%u\n", s_anc_diag.ver.s1,
												s_anc_diag.ver.s2,
												s_anc_diag.ver.s3);
							diag_info[0] = 0xE7;
							diag_info[1] = INTERFACA_VER;
							diag_info[2] = s_anc_diag.ver_resp;
							diag_info[3] = s_anc_diag.ver.s1;
							diag_info[4] = s_anc_diag.ver.s2;
							diag_info[5] = s_anc_diag.ver.s3;
							ret = send_diag_info(diag_info, VER_INFO_SIZE);
							if(ret < 0)
								pr_err("[ANC_CAN_DEVICES] send cpd software version faild, ret = %d\n", ret);
					} else if(canParamsBuffer[0] == 0xE8) {
						diag_info[0] = 0xE8;
						diag_info[1] = INTERFACA_VER;
						ret = get_anc_status(&anc_status);
						if(ret != 0) {
							diag_info[2] = RESP_FAILURE;
							pr_info("[ANC_CAN_DEVICES] get anc status faild\n");
						} else {
							diag_info[2] = RESP_SUCCESS;
						}
						diag_info[3] = anc_status;
						pr_info("[ANC_CAN_DEVICES] send anc status: %u\n", (uint32_t)anc_status);
						ret = send_diag_info(diag_info, ANCSTATUS_INFO_SIZE);
						if(ret < 0)
							pr_err("[ANC_CAN_DEVICES] send anc status faild, ret = %d\n", ret);
					} else if(canParamsBuffer[0] == 0xE9) {
						quad_id = canParamsBuffer[2];
						if(quad_id ==0  || quad_id > 4) {
							pr_err("[ANC_CAN_DEVICES] can parm quad id error, %u\n", (uint32_t)quad_id);
							continue;
						}
						diag_info[0] = 0xE9;
						diag_info[1] = INTERFACA_VER;
						ret = get_diver_cnt(&diver_errcnt, quad_id);
						if(ret != 0) {
							diag_info[2] = RESP_FAILURE;
							pr_err("[ANC_CAN_DEVICES] get divergence error counter faild\n");
						} else {
							diag_info[2] = RESP_SUCCESS;
						}
						diag_info[3] = diver_errcnt;
						pr_info("[ANC_CAN_DEVICES] send divergence error counter: 0x%x\n", (uint32_t)diver_errcnt);
						ret = send_diag_info(diag_info, DVRERRCNT_INFO_SIZE);
						if(ret < 0)
							pr_err("[ANC_CAN_DEVICES] send divergence error counter faild, ret = %d\n", ret);
						}
						else {
							pr_debug("[ANC_CAN_DEVICES] canParamsBuffe[0] = 0x%x\n", (uint32_t)canParamsBuffer[0]);
						}
				}
			}
		}
	}while(!kthread_should_stop());
	if (!IS_ERR(ioc_fd))
	{
		filp_close(ioc_fd,NULL);
	}

	return OK;
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DEVICE_NAME");
MODULE_AUTHOR("Kun Xing");
MODULE_VERSION("1.0");

module_init(anc_init);
module_exit(anc_exit);

