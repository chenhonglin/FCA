/*
 * Author: Scott Mackay <scott.mackay@harman.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include "linux/backing-dev.h"
#include <uapi/linux/earlypm.h>

#include <linux/wait.h> // Required for the wait queues
#include <linux/sched.h> // Required for task states (TASK_INTERRUPTIBLE etc ) 

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

/* Global variable for the device class */
static struct class *dev_class;


#define SUCCESS 0
#define FAILURE -1

static wait_queue_head_t actions_queue;
static wait_queue_head_t backlight_queue;
static wait_queue_head_t camera_delay_queue;
static wait_queue_head_t camera_gridlines_queue;
static wait_queue_head_t camera_pos_banner_queue;
static wait_queue_head_t camera_cancel_queue;
static wait_queue_head_t poc_control_queue;


static struct task_struct * iocReaderThread = NULL;
static int majorNumber;

// Maximum number of bytes that can be transferred between early and late evs
#define MAX_XFER_VALUES 64
static wait_queue_head_t xfer_ab_queue;
static volatile unsigned char num_xfer_ab_values = 0;
static volatile unsigned char xfer_ab_values[MAX_XFER_VALUES];
static wait_queue_head_t xfer_ba_queue;
static volatile unsigned char num_xfer_ba_values = 0;
static volatile unsigned char xfer_ba_values[MAX_XFER_VALUES];

// Queue up to 4 action requests from the VIP
#define MAX_ACTIONS 4
static volatile unsigned char current_actions[MAX_ACTIONS];

// Number of queued actions
static volatile int num_actions = 0;

// Keep a pointer to the first open call (that will receive action requests)
//  and any other open call (that will receive action requests once the first open is closed)
static void *actions_open_pointer = 0;
static void *pending_actions_pointer = 0;

// The maximum number of backlight commands sent to the VIP that can be saved for
//  returning to other processes via a read command
#define MAX_ENQUEUED_COMMANDS 16
static volatile int backlight_entries = 0;
static volatile unsigned char current_backlight_status[MAX_ENQUEUED_COMMANDS];

// Power-over-coax state for channel 0/1.
#define NUM_POC_CHANNELS 2
static volatile unsigned char poc_control[NUM_POC_CHANNELS] = {0xff,0xff};

// Set to 1 when there is a change in POC state available
static volatile int poc_change = 0;

/* Generic "device" structure for reading vehicle data */
struct vehicle_data_dev
{
	/* internal device id. also used to derive minor num */
	unsigned int id;

	/* character device name */
	const char  *name;

	/* character device struct */
	struct cdev  cdev; 

	/* sequence counter used by "poll" or blocking "read" to determine when
	 * new data available. Copy stored in file private data.  uint64_t
	 * chosen to prevent ever needing to worry about roll-over */
	uint64_t     seq;  

	/* lock to sync reader + ipc thread */
	struct mutex		 lock;  

	/* wait queue for block read. */
	wait_queue_head_t waitq; 

	/* Data pointer */
	void * data;

	/* Data size */
	size_t data_size;

	/* Pointer to custom write function [Optional] */
	ssize_t (*write)(struct file* filp, const char __user* buf, 
		size_t count, loff_t *f_pos);

	/* Pointer to custom read function [Optional] */
	ssize_t (*custom_read)(struct file* filp, char __user* buf, 
		size_t count, loff_t *f_pos);
};

struct vehicle_data_file_private {
	/* pointer to vehcile data structure */
	struct vehicle_data_dev *vdata_dev;

	/* sequence counter of last read value */
	uint64_t   seq;
};

enum vehicle_data_dev_id {
	VDATA_ID_CAMERA_CONFIG,
	VDATA_ID_VEHICLE_CAMERA_CONFIG,  /* new node to eventually depricate "camera config" */
	VDATA_ID_VEHICLE_SPEED,
	VDATA_ID_SHIFT_INFO,
	VDATA_ID_IGN_STATUS,
	VDATA_ID_CAMERA_ANGLE,
	VDATA_ID_BRANDING,
	VDATA_ID_ECALL_SOS,
	VDATA_ID_INTERIORCAM_LIGHT_REQ,
	VDATA_ID_CAMERA_DISP_STAT,
	VDATA_ID_CAMERA_DISP_REQ,
	VDATA_ID_SVC_STATUS,
	VDATA_ID_FFCM_CAM_DYNAMIC_GRIDLINE_STATE,
	VDATA_ID_CAMERA_TOUCH,
	VDATA_ID_LVDS_WALL_ASCM_ALERT_STAT,
	VDATA_ID_LIGHT_AMB_SENS,
	VDATA_ID_LVDS_VERBOSE,
	VDATA_ID_VIRTUAL_WALL,
	VDATA_ID_LVDS_COMMAND_RESP,
	VDATA_ID_LVDS_COMMAND_RQST,
	VDATA_ID_DIAGNOSTIC_STATUS,
};


/* Get vehicle data dev by ID. */
static struct vehicle_data_dev * get_vdata_dev(int id);
static struct vehicle_data_dev * vdata_start_update(int id);
static void vdata_end_update(struct vehicle_data_dev *vdata_dev);
static int vdata_available(struct vehicle_data_file_private *vdata_fp);

/* "static" Vehicle configuration data specific to FCA camera Proxi for
   Atlantis. This should not change within an ignition cycle */
static struct vehicle_camera_cfg_storage camera_cfg;

/* Storage for "raw" vehicle config data structure. This will contain and adapt
 * to version + size changes without making kernel code change */ 
static uint8_t vehicle_camera_cfg[256];

/* Vehicle speed as needed for RVC. */
static uint16_t vehicle_speed = 0xffff;

/* Shift Level Info */
static struct shift_lever_info shift_info;

/* Ignititon status signal */
static struct ignition_status ign_status;

/* Vehicle branding information needed by early animation/splash services */
static struct branding_info branding = {0};

/* ECALL Assist/SOS status needed by splash/animation services */
static struct ecall_assist_sos_state ecall_sos_state = {0};

static uint8_t camera_disp_stat = 0xff;
static uint8_t camera_disp_req  = 0xff;
static uint8_t svc_status[64];

/* LVDS Diagnostic Status information*/
static struct diagnosticStat{
    uint8_t camera_type;
    uint8_t lvds_connection_stat;
    uint8_t lvds_internal_err_stat;
    uint8_t lvds_external_err_stat;
    uint8_t video_signal_stat;
}diagStat;

/* For reading back last transmitted touch event */
static uint8_t camera_touch_data[7] = {0};
// Data related to LVDS communication
static struct lvds_wall_ascm_alert lvds_alerts = {0};
static struct lvds_amb_sens_day_lgt_md amb_sens_day_lgt_md = {0};
#define NUM_LVDS_PORTS 2
static volatile uint8_t lvds_verbose_control[NUM_LVDS_PORTS];
static volatile uint8_t virtual_wall_stat = 0xff;
#define BUFFER_LEN 64
static uint8_t lvdsCmdResp[BUFFER_LEN];
static uint8_t lvdsCmdRqst[BUFFER_LEN];

// Protects accessing above variable from multiple threads
static DEFINE_MUTEX(actions_lock);
static DEFINE_MUTEX(display_lock);
static DEFINE_MUTEX(camera_delay_lock);
static DEFINE_MUTEX(camera_gridlines_lock);
static DEFINE_MUTEX(camera_pos_banner_lock);
static DEFINE_MUTEX(camera_cancel_lock);
static DEFINE_MUTEX(poc_control_lock);

// Interface to the VIP IPC
#define BACKLIGHT_CONTROL_CHANNEL "/dev/ipc14"
static struct file *ioc_fd = 0;
#define IPC_MSG_SET_VIDEO_STATUS 0xE0
#define IPC_MSG_GET_DISPLAY_ACTION 0xE1
#define IPC_MSG_ANIMATION_STATUS   0xE5 /* SoC => IoC */
#define IPC_MSG_ECALL_ASSIST_SOS_STATUS   0xE6 /* SoC => IoC */
#define IPC_MSG_CAMERA_DELAY 0xF0
#define IPC_MSG_BANNER 0xF1
#define IPC_MSG_ANGLE 0xF2
#define IPC_MSG_CANCEL_BUTTON 0xF3
#define IPC_MSG_CAMERA_GRIDLINES 0xF4


// [Brian] New messages in RVC_IPC_SPEC_v1 from Ivan
#define IPC_MSG_PROXI_DATA       0xF5 /* DEPRECATING IN R1:R12:S3 migrate to 0xFE */
#define IPC_MSG_VEHICLE_SPEED    0xF6
#define IPC_MSG_SHIFT_INFO       0xF7
#define IPC_MSG_POC_CONTROL      0xF8
#define IPC_MSG_IGN_STATUS       0xFA
#define IPC_MSG_CAMERA_DISP_STAT 0xFB
#define IPC_MSG_CAMERA_DISP_REQ  0xFC
#define IPC_MSG_VEHICLE_CAMERA_CONFIG  0xFE
#define IPC_MSG_SVC_STATUS       0xFF
#define IPC_MSG_BRANDING         0xB0

//[sandeep] additional interface for ffcm gridline
#define IPC_MSG_FFCM_CAM_DYNAMIC_GRIDLINE_STATE  0x34 //forward facing camera
// message for CAM LIGHT Request
#define IPC_MSG_INTERIORCAM_LIGHT_REQ 0xFD
// messages related to LVDS communication
#define IPC_MSG_LVDS_WALL_ASCM_ALERT_STAT 0x30
#define IPC_MSG_LIGHT_AMB_SENS 0x31
#define IPC_MSG_VIRTUAL_WALL_REQ 0x32
// Message for LVDS Diagnostic status
#define IPC_MSG_DIAGNOSTIC_STATUS 0xCB

// Message to control the state of the VICS reset pin
#define IPC_MSG_VICS_RESET_CONTROL 0xD7
#define VICS_RESET_ACTIVE 0x00
#define VICS_RESET_INACTIVE 0x01

#define IPC_MSG_TOUCH 0xF9
#define INTERFACE_VERSION 0x00
static struct 
{
    unsigned char messageNumber;
    unsigned char interfaceVersion;
    unsigned char videoStatus;
} iocVideoStatusMessage;

// Protect access to the IPC interface
static DEFINE_MUTEX(ipc_lock);

#define DEVICE_NAME "early-pm"
#define BACKLIGHT_NODE_MINOR_NUMBER 0
#define BACKLIGHT_NODE_NAME "display_backlight"
#define ACTIONS_NODE_MINOR_NUMBER 1
#define ACTIONS_NODE_NAME "display_actions"

static const struct file_operations channel_fops;

#define DISPLAY_OFF              0
#define DISPLAY_ON_FULL          1
#define DISPLAY_ON_ANIMATION     2
#define DISPLAY_OFF_ANIMATION    3
#define DISPLAY_ON_SPLASH        4
#define DISPLAY_OFF_SPLASH       5
#define DISPLAY_ON_RVC           6
#define DISPLAY_OFF_RVC          7
#define DISPLAY_ON_SD_ANIMATION  8
#define DISPLAY_OFF_SD_ANIMATION 9
#define DISPLAY_RSE_HMI          0xa
#define DISPLAY_RSE_VIDEO        0xb
#define DISPLAY_RSE_OFF          0xc
#define DISPLAY_DTV_PRGM         0xd
#define DISPLAY_DTV_VIDEO        0xe
#define DISPLAY_DTV_CAMERA       0xf
#define DISPLAY_DTV_OFF          0x10
#define DISPLAY_RSE_ON_BLANK     0x11

int DisplayOn;
EXPORT_SYMBOL(DisplayOn);

static int mainDisplayOn        = 0;
static int animationDisplayOn   = 0;
static int splashDisplayOn      = 0;
static int rvcDisplayOn         = 0;
static int sdAnimationDisplayOn = 0;
static int rseDisplayOn         = 0;
static int dtvDisplayOn         = 0;

// If this is set to any value except for -1 then
//  a display setting is waiting to be sent to the VIP
//  once the IPC channel is openned
static volatile char pending_display_setting = -1;

static volatile unsigned char camera_delay = 255;
static volatile unsigned char camera_gridlines = 255;
static volatile unsigned char camera_pos_banner = 255;
static volatile unsigned char camera_cancel = 255;
static short camera_angle = 0xffff;
static volatile unsigned char ffcm_dynamic_guideline = 255;

/* Send ipc message. Acquires ipc_lock. */
static ssize_t earlypm_ipc_send(const char *buf, size_t len)
{
	unsigned long long offset = 0;
	ssize_t ret = -EINVAL;
	mm_segment_t originalFs;
	mutex_lock(&ipc_lock);
	if (0 != ioc_fd) {
		originalFs = get_fs();
		set_fs(get_ds());
		offset = 0;
		ret = vfs_write(ioc_fd, buf, len, &offset);
		set_fs(originalFs);
	}
	mutex_unlock(&ipc_lock);
	return ret;
}

/* Attempt to open ipc handle. Return 0 on success, -1 on err. */
static int earlypm_ipc_open(void)
{
	mm_segment_t originalFs;
	mutex_lock(&ipc_lock);
	originalFs = get_fs();
	set_fs(get_ds());
	ioc_fd = filp_open(BACKLIGHT_CONTROL_CHANNEL, O_RDWR | O_TRUNC, 0);
	set_fs(originalFs);
	if (IS_ERR(ioc_fd)) 
	{
		ioc_fd = 0;
	}
	mutex_unlock(&ipc_lock);
	return (ioc_fd != 0) ? 0 : -1;
}

//
// Thread that is used to continuously read messages from the VIP
//
static int IOCDataReader(void *data)
{
   struct
   {
      unsigned char messageNumber;
      unsigned char interfaceVersion;
      unsigned char payload[256];
   } iocRxMessage;

   mm_segment_t originalFs;
   unsigned long long offset = 0;
   struct vehicle_data_dev * vdata_dev;

   if (0 != ioc_fd)
   {
      // Get the current camera delay
      iocRxMessage.messageNumber    = IPC_MSG_CAMERA_DELAY;
      iocRxMessage.interfaceVersion = INTERFACE_VERSION;
      iocRxMessage.payload[0]       = 0;

      mutex_lock(&ipc_lock);

      originalFs = get_fs();
      set_fs(get_ds());
      offset = 0;
      vfs_write(ioc_fd, (char*)&iocRxMessage,\
                        3, &offset);

      set_fs(originalFs);

      mutex_unlock(&ipc_lock);
   }

   while(!kthread_should_stop())
   {
      if (0 == ioc_fd)
      {
         if (-1 == earlypm_ipc_open()){
             // Wait a while before attempting to open again
             msleep(100);
         }
         else
         {
            //Get vehicle camera config
            iocRxMessage.messageNumber    = IPC_MSG_VEHICLE_CAMERA_CONFIG;
            iocRxMessage.interfaceVersion = INTERFACE_VERSION;
            iocRxMessage.payload[0]       = 0;
            earlypm_ipc_send((char*)&iocRxMessage, 3);

            /* Get the current camera delay */
            iocRxMessage.messageNumber    = IPC_MSG_CAMERA_DELAY;
            iocRxMessage.interfaceVersion = INTERFACE_VERSION;
            iocRxMessage.payload[0]       = 0;
            earlypm_ipc_send((char*)&iocRxMessage, 3);

            /* Get disclaimer configuration info */
            iocRxMessage.messageNumber    = IPC_MSG_BRANDING;
            iocRxMessage.interfaceVersion = INTERFACE_VERSION;
            earlypm_ipc_send((char*)&iocRxMessage, 2);
         }
      }
      else
      {
         // Read a message from the VIP
         int result;
         int payload_len;

         originalFs = get_fs();
         set_fs(get_ds());
         mutex_lock(&ipc_lock);
         if (-1 != pending_display_setting)
         {
            // Send the pending display message to the vip
            iocVideoStatusMessage.messageNumber    = IPC_MSG_SET_VIDEO_STATUS;
            iocVideoStatusMessage.interfaceVersion = INTERFACE_VERSION;
            iocVideoStatusMessage.videoStatus      = pending_display_setting;
            offset = 0;

            result = vfs_write(ioc_fd, (char*)&iocVideoStatusMessage,\
                         sizeof(iocVideoStatusMessage), &offset);

            if(-1 != result)
            {
               // Forget about the pending setting if it was written to the VIP
               pending_display_setting = -1;
            }
         }
         mutex_unlock(&ipc_lock);

         offset = 0;
         result = vfs_read(ioc_fd, (char*)&iocRxMessage, sizeof(iocRxMessage), &offset);
         payload_len = result - 2;
         set_fs(originalFs);
         if (0 < result)
         {
            switch(iocRxMessage.messageNumber)
            {
               case IPC_MSG_GET_DISPLAY_ACTION:
                  // Convert the value into a display action
                  mutex_lock(&actions_lock);
                  if (num_actions < MAX_ACTIONS)
                  {
                     // Save the action at the end of the queue
                     current_actions[num_actions++] = iocRxMessage.payload[0];
                  }
                  else
                  {
                    // Replace the most recent action with this one
                    current_actions[MAX_ACTIONS-1] = iocRxMessage.payload[0];
                  }
                  mutex_unlock(&actions_lock);

                  //wake up all the tasks waiting on the actions_queue
                  wake_up_interruptible(&actions_queue);
                  break;
                  
               case IPC_MSG_CAMERA_DELAY:
                  mutex_lock(&camera_delay_lock);
                  camera_delay = iocRxMessage.payload[1];
                  mutex_unlock(&camera_delay_lock);
                  pr_info("%s: vip response : camera_delay = %d\n", __func__, camera_delay);

                  //wake up all the tasks waiting on the camera_delay_queue
                  wake_up_interruptible(&camera_delay_queue);
                  break;

               case IPC_MSG_CAMERA_GRIDLINES:
                  mutex_lock(&camera_gridlines_lock);
                  camera_gridlines = iocRxMessage.payload[1];
                  mutex_unlock(&camera_gridlines_lock);
                  pr_info("%s: vip response : camera_gridlines = %d\n", __func__, camera_gridlines);
                  
                  //wake up all the tasks waiting on the camera_gridlines_queue
                  wake_up_interruptible(&camera_gridlines_queue);
                  break;

               case IPC_MSG_FFCM_CAM_DYNAMIC_GRIDLINE_STATE:
                  if ((vdata_dev = vdata_start_update(VDATA_ID_FFCM_CAM_DYNAMIC_GRIDLINE_STATE))){
                     ffcm_dynamic_guideline = iocRxMessage.payload[1];
                     pr_info("%s: vip response : ffcm_dynamic_guideline = %d\n", __func__, ffcm_dynamic_guideline);
                     vdata_end_update(vdata_dev);
                  }
                  break;

               case IPC_MSG_BANNER:
                  mutex_lock(&camera_pos_banner_lock);
                  camera_pos_banner = 0;
                  if (iocRxMessage.payload[0])
                  {
                     camera_pos_banner |= 0x01;
                  }
                  if (iocRxMessage.payload[1])
                  {
                     camera_pos_banner |= 0x02;
                  }
                  mutex_unlock(&camera_pos_banner_lock);

                  //wake up all the tasks waiting on the camera_pos_banner_queue
                  wake_up_interruptible(&camera_pos_banner_queue);
                  break;
               case IPC_MSG_ANGLE:
                  if (payload_len == 2 && iocRxMessage.interfaceVersion == 0){
                     if((vdata_dev = vdata_start_update(VDATA_ID_CAMERA_ANGLE))){
                        uint16_t tmp = ((uint16_t)iocRxMessage.payload[1] << 8) + iocRxMessage.payload[0];
                        camera_angle = (short) tmp - 4096;
                        vdata_end_update(vdata_dev);
                     }
                  }
                  break;
               case IPC_MSG_CANCEL_BUTTON:
                  mutex_lock(&camera_cancel_lock);
                  camera_cancel = iocRxMessage.payload[0];
                  mutex_unlock(&camera_cancel_lock);

                  //wake up all the tasks waiting on the camera_cancel_queue
                  wake_up_interruptible(&camera_cancel_queue);
                  break;
               case IPC_MSG_PROXI_DATA:
                  if (payload_len == 3 && iocRxMessage.interfaceVersion == 0) {
                     struct vehicle_camera_cfg_atlmi *atlmi_cfg =
                     (struct vehicle_camera_cfg_atlmi *) &camera_cfg;
                     if((vdata_dev = vdata_start_update(VDATA_ID_CAMERA_CONFIG))){
                        atlmi_cfg->type = CAMCFG_FCA_ATL_MID;
                        atlmi_cfg->version = iocRxMessage.interfaceVersion;
                        atlmi_cfg->rear_view_camera_proxi = iocRxMessage.payload[0];
                        atlmi_cfg->gear_box_type_proxi    = iocRxMessage.payload[1];
                        atlmi_cfg->vehicle_line_configuration = iocRxMessage.payload[2];
                        vdata_dev->data_size = sizeof(*atlmi_cfg);
                        vdata_end_update(vdata_dev);
                     }
                  } else {
                     pr_err("%s: got malformed or unexpected camera proxi message!\n", __func__);
                  }
                  break;

               case IPC_MSG_VEHICLE_CAMERA_CONFIG:
                  if((vdata_dev = vdata_start_update(VDATA_ID_VEHICLE_CAMERA_CONFIG))){
                     /* Store as many bytes as possible w/o any in-kernel processing */
                     size_t copy_len = result < sizeof(vehicle_camera_cfg) ?
                     result : sizeof(vehicle_camera_cfg);

                     /* this should not be possible unless ipc msg frame size changes, but put
                     * in a warning here just in case. */
                     if (copy_len < result){
                        pr_warn("%s: discarding %d bytes of camera cfg message!\n", __func__,
                        (int) (result - copy_len));
                     }
                     memcpy(vehicle_camera_cfg, &iocRxMessage, copy_len);
                     vdata_dev->data_size = copy_len;
                     vdata_end_update(vdata_dev);
                  }
                  break;

               case IPC_MSG_VEHICLE_SPEED:
                  if (payload_len == 2 && iocRxMessage.interfaceVersion == 0){
                     if((vdata_dev = vdata_start_update(VDATA_ID_VEHICLE_SPEED))){
                        vehicle_speed = ((uint16_t) iocRxMessage.payload[0]) | \
                           (((uint16_t) iocRxMessage.payload[1]) << 8);
                        vdata_end_update(vdata_dev);
                     }
                  } else {
                          printk_ratelimited(KERN_ERR "%s: got bad vehicle speed msg\n", __func__);
                  }
                  break;

               case IPC_MSG_SHIFT_INFO:
                  if (payload_len >= 4 && iocRxMessage.interfaceVersion == 0){
                     if ((vdata_dev = vdata_start_update(VDATA_ID_SHIFT_INFO))){
                        shift_info.version = iocRxMessage.interfaceVersion;
                        shift_info.can_arch       = iocRxMessage.payload[0];
                        shift_info.shift_position = iocRxMessage.payload[1];
                        shift_info.reverse_gear   = iocRxMessage.payload[2];
                        shift_info.neutral_sw_sts = iocRxMessage.payload[3];
                        shift_info.awd_status     = payload_len > 4 ? iocRxMessage.payload[4]: 0;
                        vdata_end_update(vdata_dev);
                     }
                  } else {
                          printk_ratelimited(KERN_ERR "%s: got bad shift info\n", __func__);
                  }
                  break;

               case IPC_MSG_IGN_STATUS:
                  if (payload_len == 3 && iocRxMessage.interfaceVersion == 0){
                     if ((vdata_dev = vdata_start_update(VDATA_ID_IGN_STATUS))){
                        ign_status.version       = iocRxMessage.interfaceVersion;
                        ign_status.ign_sts       = iocRxMessage.payload[0];
                        ign_status.ign_fail_sts  = iocRxMessage.payload[1];
                        ign_status.can_arch      = iocRxMessage.payload[2];
                        vdata_end_update(vdata_dev);
                     }
                  } else {
                     printk_ratelimited(KERN_ERR "%s: got bad ign status\n", __func__);
                  }
                  break;

               case IPC_MSG_BRANDING:
                  /* Using payload_len >= 5 so that in future if we add another byte in the
                     end it will not cause regression, the new brand msg will work once we
                     add the handling for it.
                  */
                  if ( payload_len >= 5 ){
                     if ((vdata_dev = vdata_start_update(VDATA_ID_BRANDING))){
                        branding.version         = iocRxMessage.interfaceVersion;
                        branding.vehicle_brand   = iocRxMessage.payload[0];
                        branding.splash_type     = iocRxMessage.payload[1];
                        branding.special_package = iocRxMessage.payload[2];
                        branding.audio_brand     = iocRxMessage.payload[3];
                        branding.sxm             = iocRxMessage.payload[4];
                        if( payload_len >= 6 ) {
                           branding.srt_present = iocRxMessage.payload[5];
                        }
                        vdata_end_update(vdata_dev);
                     }
                  } else {
                        printk_ratelimited(KERN_ERR "%s: got bad branding info (len=%d, ver=%d)\n",
                        __func__, payload_len, iocRxMessage.interfaceVersion);
                  }
                  break;

               case IPC_MSG_POC_CONTROL:
                  {
                     int poc_port = 0;
                     pr_info("%s: poc_port %d => %d\n", __func__, poc_port, poc_control[poc_port]);
                     mutex_lock(&poc_control_lock);
                     for ( poc_port=0; poc_port < NUM_POC_CHANNELS; poc_port++){
                        if (poc_control[poc_port] != iocRxMessage.payload[poc_port]){
                           poc_control[poc_port] = iocRxMessage.payload[poc_port];
                           poc_change = 1;
                           pr_info("%s: poc_port %d => %d\n", __func__, poc_port, poc_control[poc_port]);
                        }
                     }
                     mutex_unlock(&poc_control_lock);
                     wake_up_interruptible(&poc_control_queue);
                  }
                  break;

               case IPC_MSG_ECALL_ASSIST_SOS_STATUS:
                  if ( payload_len == 1 ){
                      if ((vdata_dev = vdata_start_update(VDATA_ID_ECALL_SOS))){
                          ecall_sos_state.version     = iocRxMessage.interfaceVersion;
                          ecall_sos_state.call_status = iocRxMessage.payload[0];
                          printk(">>> %s, ecall status: 0x%x\n", __func__, ecall_sos_state.call_status);
                          vdata_end_update(vdata_dev);
                      }
                  } else {
                        printk_ratelimited(KERN_ERR "%s: got bad call status (len=%d, ver=%d)\n",
                        __func__, payload_len, iocRxMessage.interfaceVersion);
                  }
                  break;

               case IPC_MSG_SVC_STATUS:
                  if((vdata_dev = vdata_start_update(VDATA_ID_SVC_STATUS))){
                     size_t copy_len = result < sizeof(svc_status) ?
                     result : sizeof(svc_status);
                     if (copy_len < result){
                        pr_warn("%s: discarding %d bytes of svc status message!\n", __func__,
                        (int) (result - copy_len));
                     }
                     memcpy(svc_status, &iocRxMessage, copy_len);
                     vdata_dev->data_size = copy_len;
                     vdata_end_update(vdata_dev);
                  }
                  break;

               case IPC_MSG_LVDS_WALL_ASCM_ALERT_STAT:
                  if(result == (sizeof(lvds_alerts)+1)) {
                     if(lvds_alerts.wallAlert_InnerLeft != iocRxMessage.payload[0] ||
                      lvds_alerts.wallAlert_InnerRight != iocRxMessage.payload[1] ||
                      lvds_alerts.wallAlert_OuterLeft != iocRxMessage.payload[2] ||
                      lvds_alerts.wallAlert_OuterRight != iocRxMessage.payload[3] ||
                      lvds_alerts.ascm_Stat != iocRxMessage.payload[4] ) {
                        if((vdata_dev = vdata_start_update(VDATA_ID_LVDS_WALL_ASCM_ALERT_STAT))){
                           memcpy(&lvds_alerts, &iocRxMessage.interfaceVersion, sizeof(lvds_alerts));
                           vdata_dev->data_size = sizeof(lvds_alerts);
                           vdata_end_update(vdata_dev);
                        }
                     }
                  } else {
                      printk_ratelimited(KERN_ERR "%s: got bad wall alert message (len=%d, ver=%d)\n",
                        __func__, result, iocRxMessage.interfaceVersion);
                  }
                  break;

               case IPC_MSG_LIGHT_AMB_SENS:
                  if(result == (sizeof(amb_sens_day_lgt_md)+1)) {
                     if((vdata_dev = vdata_start_update(VDATA_ID_LIGHT_AMB_SENS))){
                        amb_sens_day_lgt_md.version = iocRxMessage.interfaceVersion;
                        amb_sens_day_lgt_md.amb_sens_val = iocRxMessage.payload[0];
                        vdata_dev->data_size = sizeof(amb_sens_day_lgt_md);
                        vdata_end_update(vdata_dev);
                     }
                  } else {
                     printk_ratelimited(KERN_ERR "%s: got bad light sens message (len=%d, ver=%d)\n",
                        __func__, result, iocRxMessage.interfaceVersion);
                  }
                  break;

               case IPC_MSG_VIRTUAL_WALL_REQ:
                  if(payload_len <= (sizeof(virtual_wall_stat)+1) && (vdata_dev = vdata_start_update(VDATA_ID_VIRTUAL_WALL))){
                     virtual_wall_stat = iocRxMessage.payload[1];
                     vdata_end_update(vdata_dev);
                  }
                  else{
                     printk_ratelimited(KERN_ERR "%s: got bad virtualWall message (expected:%d received=%d, ver=%d)\n",
                        __func__, (int)(sizeof(virtual_wall_stat)+1), payload_len, iocRxMessage.interfaceVersion);
                  }
                  break;

               default:
                  pr_info("%s: Unexpected spi-ipc message on ch14!\n", __func__);
                  // Log an error?
                  break;
            }
         }
         else
         {
            // Sleep for a while on an error so that we don't end up in a tight loop
            msleep(100);
         }
      }

      if (kthread_should_stop())
      {
         break;
      }
   }

   return 0;
}

//
// Function to open either the backlight or actions channel
//
static int channel_open(struct inode *inode, struct file *filp)
{
   // Allocate a single byte to be used to store the last read value
   filp->private_data = kzalloc(sizeof(unsigned char), GFP_KERNEL);
   if (filp->private_data)
   {
      // Initialize the last read value to a value that will never be read
      *((unsigned char *)filp->private_data) = 255;
   }
   else
   {
      return -ENOMEM;
   }

   return SUCCESS;
}

//
// Function to close either the backlight or actions channel
//
static int channel_close(struct inode *inode, struct file *filp)
{
   if(filp->private_data) 
   {
      // Free up the data allocated in the open command
      kfree(filp->private_data);
      filp->private_data = 0;
      return SUCCESS;
   }
   else
   {
      return -EINVAL;
   }
}

//
// Function to open the actions channel
//
static int actions_open(struct inode *inode, struct file *filp)
{
    int rc = SUCCESS;
    
   // Allocate a single byte to be used to store the last read value
   filp->private_data = kzalloc(sizeof(unsigned char), GFP_KERNEL);
   if (filp->private_data)
   {
      mutex_lock(&actions_lock);
      if (actions_open_pointer)
      {
         // If this is already open
         if (pending_actions_pointer)
         {
            // There are already two opens - do not allow another
             kfree(filp->private_data);
             filp->private_data = 0;
             rc = -EFAULT;
         }
         else
         {
            // Save the open call
            pending_actions_pointer = filp->private_data;
         }
      }
      else
      {
         // Mark this as the "primary" open
         actions_open_pointer = filp->private_data;
      }
      mutex_unlock(&actions_lock);
   }
   else
   {
      rc = -EFAULT;
   }
   
   if (filp->private_data)
   {
      // Initialize the last read value to a value that will never be read
      *((unsigned char *)filp->private_data) = 255;
   }

   return rc;
}

//
// Function to close the actions channel
//
static int actions_close(struct inode *inode, struct file *filp)
{
   if(filp->private_data) 
   {
      mutex_lock(&actions_lock);
      if (actions_open_pointer == filp->private_data)
      {
         // If there is a pending open then make it the primary
         if (pending_actions_pointer)
         {
             actions_open_pointer = pending_actions_pointer;
             pending_actions_pointer = 0;
             
             //wake up all the tasks waiting on the actions_queue
             wake_up_interruptible(&actions_queue);
         }
         else
         {
            // Forget about this open
            actions_open_pointer = 0;
         }
      }
      else if (pending_actions_pointer == filp->private_data)
      {
         // Forget about this open
         pending_actions_pointer = 0;
      }
      mutex_unlock(&actions_lock);
      
      // Free up the data allocated in the open command
      kfree(filp->private_data);
      filp->private_data = 0;
      return SUCCESS;
   }
   else
   {
      return -EINVAL;
   }
}

//
// Function to poll for actions from the VIP
//
static unsigned actions_poll(struct file *filp, struct poll_table_struct *pt)
{
    unsigned int mask = 0;

    if(filp->private_data) 
    {
        poll_wait(filp, &actions_queue, pt);
        mutex_lock(&actions_lock);
        // Only note that there is data available to the first open
        if (actions_open_pointer == filp->private_data)
        {
           if (num_actions > 0)
           {
               mask = POLLIN | POLLRDNORM;
           }
        }
        mutex_unlock(&actions_lock);
    }
    else
    {
        mask = POLLERR;
    }
    
    return mask;
}

//
// Perform a blocking read of the actions channel
// Will not return until a new action that is different from the 
//  last read action is commanded by the VIP
//
static ssize_t actions_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   int rc = 0;
   int result;
   unsigned char action;
   int i;

   if(actions_open_pointer == filp->private_data) 
   {
      // Wait for a new action
      while (0 != wait_event_interruptible(actions_queue, num_actions > 0))
      {
         // Keep waiting if the wait was interrupted
      }

      mutex_lock(&actions_lock);
      if (num_actions == 0)
      {
        rc = -EINVAL;
      }
      else
      {
         // Copy the new action to the caller's buffer
         action = '0' + current_actions[0];
 
         result = copy_to_user(buf, &action, 1);
      
         if (result) 
         {
            rc = -EFAULT;
         }
         else
         {
            // Remove the successfully transferred action from the queue
            --num_actions;
            for (i = 0; i < num_actions; ++i)
            {
               current_actions[i] = current_actions[i+1];
            }
            rc = 1;
         }
      }
      mutex_unlock(&actions_lock);
   }
   else
   {
      rc = -EINVAL;
   }

   return rc;
}

/* Write to this node to confirm when a display_action request has been
 * completed  */
static ssize_t actions_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
	mm_segment_t originalFs;
	unsigned long long offset = 0;
	int ret = 0;
	struct
	{
		unsigned char messageNumber;
		unsigned char interfaceVersion;
		unsigned char status;
	} iocAnimationSplashStatus;

	if (1 != count)
	{
		return -EFAULT;
	}

	// Get the action from the caller
	if (copy_from_user(&iocAnimationSplashStatus.status, buf, 1) != 0)
	{
		return -EFAULT;
	}

	iocAnimationSplashStatus.messageNumber    = IPC_MSG_ANIMATION_STATUS;
	iocAnimationSplashStatus.interfaceVersion = INTERFACE_VERSION;
	
        originalFs = get_fs();
        set_fs(get_ds());
        ret = vfs_write(ioc_fd, (char*)&iocAnimationSplashStatus,\
                        sizeof(iocAnimationSplashStatus), &offset);
        set_fs(originalFs);


	return count;
}
   
//
// loopback interface to get the last value written to the VIP
//
static unsigned int backlight_poll(struct file *filp, struct poll_table_struct *pt)
{
    unsigned int mask = 0;
    
    if(filp->private_data) 
    {
        poll_wait(filp, &backlight_queue, pt);
        mutex_lock(&display_lock);
        if (0 < backlight_entries)
        {
            mask = POLLIN | POLLRDNORM;
        }
        mutex_unlock(&display_lock);
    }
    else
    {
        mask = POLLERR;
    }
    
    return mask;
}

//
// Non-blocking loopback interface to get the last value written to the VIP
//
static ssize_t backlight_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   unsigned char status[MAX_ENQUEUED_COMMANDS];
   int result;

   if ((filp != NULL) && (filp->private_data) && (count > 0))
   {
      mutex_lock(&display_lock);
      
      if (backlight_entries <= 0)
      {
         count = 1;
         status[0] = current_backlight_status[0];
      }
      else
      {
         if (count > backlight_entries)
         {
            count = backlight_entries;
         }
         memcpy((void *)status, (void *)current_backlight_status, count);
         backlight_entries -= count;
         if (0 < backlight_entries)
         {
            memmove((void *)current_backlight_status, (void *)&current_backlight_status[count], backlight_entries);
         }
      }
      
      mutex_unlock(&display_lock);
      result = copy_to_user(buf, status, count);
      
      if (result) 
      {
         count = -EFAULT;
      }
   }
   else
   {
      count = -EINVAL;
   }

   return count;
}

//
// Helper function to determine what to send to the VIP
//
static unsigned char determineBacklightDisplayStatus(unsigned char status)
{
    unsigned char curStatus;
    
    switch (status)
    {
        case DISPLAY_OFF:
            mainDisplayOn = 0;
            break;
        case DISPLAY_ON_FULL:
            mainDisplayOn = 1;
            break;
        case DISPLAY_ON_ANIMATION:
            animationDisplayOn = 1;
            splashDisplayOn = 0;
            sdAnimationDisplayOn = 0;
            break;
        case DISPLAY_OFF_ANIMATION:
            animationDisplayOn = 0;
            break;
        case DISPLAY_ON_SPLASH:
            animationDisplayOn = 0;
            splashDisplayOn = 1;
            sdAnimationDisplayOn = 0;
            break;
        case DISPLAY_OFF_SPLASH:
            splashDisplayOn = 0;
            break;
        case DISPLAY_ON_RVC:
            rvcDisplayOn = 1;
            break;
        case DISPLAY_OFF_RVC:
            rvcDisplayOn = 0;
            break;
        case DISPLAY_ON_SD_ANIMATION:
            animationDisplayOn = 0;
            splashDisplayOn = 0;
            sdAnimationDisplayOn = 1;
            break;
        case DISPLAY_OFF_SD_ANIMATION:
            sdAnimationDisplayOn = 0;
            break;
        case DISPLAY_DTV_CAMERA:
        case DISPLAY_DTV_VIDEO:
        case DISPLAY_DTV_PRGM:
            dtvDisplayOn = status;
            break;
        case DISPLAY_DTV_OFF:
            dtvDisplayOn = 0;
            break;
	case DISPLAY_RSE_HMI:
	case DISPLAY_RSE_VIDEO:
	case DISPLAY_RSE_ON_BLANK:
	    rseDisplayOn = status;
	    break;
        case DISPLAY_RSE_OFF:
	    rseDisplayOn = 0;
	    break;
    }

    DisplayOn = mainDisplayOn;

    if (rvcDisplayOn)
        curStatus = DISPLAY_ON_RVC;
    else if (animationDisplayOn)
        curStatus = DISPLAY_ON_ANIMATION;
    else if (splashDisplayOn)
        curStatus = DISPLAY_ON_SPLASH;
    else if (sdAnimationDisplayOn)
        curStatus = DISPLAY_ON_SD_ANIMATION;
    else if (rseDisplayOn)
        curStatus = rseDisplayOn;
    else if (dtvDisplayOn)
        curStatus = dtvDisplayOn;
    else if (mainDisplayOn)
        curStatus = DISPLAY_ON_FULL;
    else
        curStatus = DISPLAY_OFF;

    pr_info("%s: state=%x\n; mn=%x,an=%x,sp=%x,rv=%x,sd=%x,rse=%x,dtv=%x",
	__func__, curStatus, mainDisplayOn, animationDisplayOn,
	splashDisplayOn,rvcDisplayOn, sdAnimationDisplayOn, rseDisplayOn,
	dtvDisplayOn );
    
    return curStatus;
}

static ssize_t backlight_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    unsigned char status;
    unsigned char derivedStatus;
    mm_segment_t originalFs;
    unsigned long long offset = 0;
    int ret = 0;

    // Get the action from the caller
    if (copy_from_user(&status, buf, 1) != 0)
    {
        return -EFAULT;
    }

    // Determing the backlight status based on the current settings an this new value
    derivedStatus = determineBacklightDisplayStatus(status);

    mutex_lock(&ipc_lock);
    if (0 != ioc_fd)
    {    
        iocVideoStatusMessage.messageNumber    = IPC_MSG_SET_VIDEO_STATUS;
        iocVideoStatusMessage.interfaceVersion = INTERFACE_VERSION;
        iocVideoStatusMessage.videoStatus      = derivedStatus;

        originalFs = get_fs();
        set_fs(get_ds());
        ret = vfs_write(ioc_fd, (char*)&iocVideoStatusMessage,\
                        sizeof(iocVideoStatusMessage), &offset);
        set_fs(originalFs);
        
        // Make sure that we don't send an "old" pending setting
        pending_display_setting = -1;
    }
    else
    {
        // Remember the value so that it can be sent once the IPC channel is available
        pending_display_setting = derivedStatus;
    }
    mutex_unlock(&ipc_lock);

    if(-1 == ret) 
    {
        return -EFAULT;
    }
   
    mutex_lock(&display_lock);
    if (MAX_ENQUEUED_COMMANDS > backlight_entries)
    {
        current_backlight_status[backlight_entries++] = ((derivedStatus & 0x0f) << 4) | (status & 0x0f);
    }
    else
    {
        // Remove the oldest entry
        memmove((void *)current_backlight_status, (void *)&current_backlight_status[1], MAX_ENQUEUED_COMMANDS-1);
        current_backlight_status[MAX_ENQUEUED_COMMANDS-1] = ((derivedStatus & 0x0f) << 4) | (status & 0x0f);
    }
    mutex_unlock(&display_lock);
    wake_up_interruptible(&backlight_queue);

    return 1;
}

//
// 
//
static unsigned int camera_delay_poll(struct file *filp, struct poll_table_struct *pt)
{
   unsigned int mask = 0;

    if(filp->private_data) 
    {
        unsigned char last_value = *((unsigned char *)filp->private_data);
        poll_wait(filp, &camera_delay_queue, pt);
        mutex_lock(&camera_delay_lock);
        if (last_value != camera_delay)
        {
            mask = POLLIN | POLLRDNORM;
            
        }
        mutex_unlock(&camera_delay_lock);
    }
    else
    {
        mask = POLLERR;
    }
    
    return mask;
}

//
// 
//
static ssize_t camera_delay_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   int result;

   mutex_lock(&camera_delay_lock);
   result = copy_to_user(buf, (const void *)(&camera_delay), sizeof(camera_delay));
   *((unsigned char *)filp->private_data) = camera_delay;
   mutex_unlock(&camera_delay_lock);     
   
   if(result)
   {
      return -EFAULT;
   }

   return sizeof(camera_delay);
}

static ssize_t camera_delay_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
   char msgBuf[] = {IPC_MSG_CAMERA_DELAY, INTERFACE_VERSION, 0x01, 0x00};
   int  sendLen = 3;
   
   //0x01 0xXX => set operation
   //0x00 0x00 => get operation
   if ( count < 1 || copy_from_user(&msgBuf[2], buf, 2) != 0){ //copy two bytes
      pr_err("%s: failure!!. count: %d\n", __func__, (int)count);
      return -EFAULT;
   }
   
   if(msgBuf[2] == 0x01) //SET operation expects 4 bytes, and GET expects 3 bytes.
      sendLen = 4;
   
   /* Request VIP to process this request*/
   earlypm_ipc_send(msgBuf, sendLen);
   
   return count;

}


static unsigned int camera_gridlines_poll(struct file *filp, struct poll_table_struct *pt)
{
   unsigned int mask = 0;

   if(filp->private_data) 
   {
      unsigned char last_value = *((unsigned char *)filp->private_data);
      poll_wait(filp, &camera_gridlines_queue, pt);
      mutex_lock(&camera_gridlines_lock);
      if (last_value != camera_gridlines)
      {
         mask = POLLIN | POLLRDNORM;
            
      }
      mutex_unlock(&camera_gridlines_lock);      
   }
   return mask;
}

//
// 
//
static ssize_t camera_gridlines_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   int result = 0;

   mutex_lock(&camera_gridlines_lock);
   result = copy_to_user(buf, (unsigned char *)&camera_gridlines, sizeof(camera_gridlines));
   *((unsigned char *)filp->private_data) = camera_gridlines;
   mutex_unlock(&camera_gridlines_lock);      
   
   if (result)
   {
      return -EFAULT;
   }

   return sizeof(camera_gridlines);
}

static ssize_t camera_gridlines_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
   char msgBuf[] = {IPC_MSG_CAMERA_GRIDLINES, INTERFACE_VERSION, 0x01, 0x00};
   int  sendLen = 3;
   
   //0x01 0xXX => set operation
   //0x00 0x00 => get operation
   if ( count < 1 || copy_from_user(&msgBuf[2], buf, 2) != 0){ //copy two bytes
      pr_err("%s: failure!!. count: %d\n", __func__, (int)count);
      return -EFAULT;
   }
   
   if(msgBuf[2] == 0x01) //SET operation expects 4 bytes, and GET expects 3 bytes.
      sendLen = 4;
   
   /* Request VIP to process this request*/
   earlypm_ipc_send(msgBuf, sendLen);
   
   return count;

}

//
// 
//
static unsigned int camera_pos_banner_poll(struct file *filp, struct poll_table_struct *pt)
{
   unsigned int mask = 0;

    if(filp->private_data) 
    {
        unsigned char last_value = *((unsigned char *)filp->private_data);
        poll_wait(filp, &camera_pos_banner_queue, pt);
        mutex_lock(&camera_pos_banner_lock);
        if (last_value != camera_pos_banner)
        {
            mask = POLLIN | POLLRDNORM;
        }
        mutex_unlock(&camera_pos_banner_lock);
    }
    else
    {
        mask = POLLERR;
    }
    
    return mask;
}

//
// 
//
static ssize_t camera_pos_banner_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   int read_bytes = 0;
   int result;

   if(filp->private_data) 
   {
      unsigned char last_value = *((unsigned char *)filp->private_data);

      // Wait for a new action
      while (0 != wait_event_interruptible(camera_pos_banner_queue, camera_pos_banner != last_value))
      {
         // Keep waiting if the wait was interrupted
      }
     
      // Copy the new action to the caller's buffer
      result = copy_to_user(buf, (const void *)(&camera_pos_banner), 1);
      
      if (result) 
      {
         return -EFAULT;
      }
      
      *((unsigned char *)filp->private_data) = camera_pos_banner;
      
      // return last_action to caller
      read_bytes = 1;
   }
   else
   {
      return -EINVAL;
   }

   return read_bytes;
}

//
// 
//
static unsigned int camera_cancel_poll(struct file *filp, struct poll_table_struct *pt)
{
   unsigned int mask = 0;

    if(filp->private_data) 
    {
        unsigned char last_value = *((unsigned char *)filp->private_data);
        poll_wait(filp, &camera_cancel_queue, pt);
        mutex_lock(&camera_cancel_lock);
        if (last_value != camera_cancel)
        {
            mask = POLLIN | POLLRDNORM;
        }
        mutex_unlock(&camera_cancel_lock);
    }
    else
    {
        mask = POLLERR;
    }
    
    return mask;
}

//
// 
//
static ssize_t camera_cancel_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   int read_bytes = 0;
   int result;

   if(filp->private_data) 
   {
      unsigned char last_value = *((unsigned char *)filp->private_data);
      
       // Wait for a new value
      while (0 != wait_event_interruptible(camera_cancel_queue, camera_cancel != last_value))
      {
         // Keep waiting if the wait was interrupted
      }
     
      // Copy the new value to the caller's buffer
      result = copy_to_user(buf, (const void *)(&camera_cancel), 1);
      
      if (result) 
      {
         return -EFAULT;
      }
      
      *((unsigned char *)filp->private_data) = camera_cancel;
      
      // return last_action to caller
      read_bytes = 1;
   }
   else
   {
      return -EINVAL;
   }

   return read_bytes;
}



//
// A write to this node causes a "cancel button pressed" message to be sent to the VIP
//
static ssize_t camera_cancel_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    mm_segment_t originalFs;
    unsigned long long offset = 0;
    int ret = 0;
    struct 
    {
        unsigned char messageNumber;
        unsigned char interfaceVersion;
        unsigned char cancelPressed;
    } iocCameraCancelMessage;
    
    mutex_lock(&ipc_lock);
    if (0 != ioc_fd)
    {    
        iocCameraCancelMessage.messageNumber    = IPC_MSG_CANCEL_BUTTON;
        iocCameraCancelMessage.interfaceVersion = INTERFACE_VERSION;
        iocCameraCancelMessage.cancelPressed    = 1;

        originalFs = get_fs();
        set_fs(get_ds());
        ret = vfs_write(ioc_fd, (char*)&iocCameraCancelMessage,\
                        sizeof(iocCameraCancelMessage), &offset);
        set_fs(originalFs);
    }
    mutex_unlock(&ipc_lock);

    if(-1 == ret) 
    {
        return -EFAULT;
    }
   
    return 1;
}

// Poll operation for PoC control (for notification of change by VIP)
static unsigned int poc_control_poll(struct file *filp, struct poll_table_struct *pt)
{
   unsigned int mask = 0;

    if(filp->private_data) 
    {
        poll_wait(filp, &poc_control_queue, pt);
        mutex_lock(&poc_control_lock);
        if (1 == poc_change)
        {
            mask = POLLIN | POLLRDNORM;
        }
        mutex_unlock(&poc_control_lock);
    }
    else
    {
        mask = POLLERR;
    }
    
    return mask;
}

//
// 
//
static ssize_t poc_control_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   int read_bytes = 0;
   int result;
   unsigned char poc_control_tmp[NUM_POC_CHANNELS];
   pr_info("%s: count=%d\n", __func__, (int)count);

   if(filp->private_data) 
   {
      if (2 != sizeof(poc_control))
      {
         return -EFAULT;
      }
   
      if (0 == (filp->f_flags & O_NONBLOCK))
      {
         if (0 != wait_event_interruptible(poc_control_queue, poc_change == 1))
         {
		 return -EINTR;
         }
      }

      mutex_lock(&poc_control_lock);
      memcpy(poc_control_tmp, (unsigned char*) poc_control, sizeof(poc_control));
      poc_change = 0;
      mutex_unlock(&poc_control_lock);

      // Copy the new value to the caller's buffer
      result = copy_to_user(buf, (const void *)(&poc_control), sizeof(poc_control));
      
      if (result) 
      {
         return -EFAULT;
      }
      
      // return size to caller
      read_bytes = sizeof(poc_control);
   }
   else
   {
      return -EINVAL;
   }

   return read_bytes;
}

//
// Write to this node to enable/disable Power-over-coax (PoC) for cameras
//
static ssize_t poc_control_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    mm_segment_t originalFs;
    unsigned long long offset = 0;
    int ret = 0;
    struct 
    {
        unsigned char messageNumber;
        unsigned char interfaceVersion;
        unsigned char request_write;
        unsigned char poc_control[NUM_POC_CHANNELS];
    } iocPocControlMessage;
    
    if (copy_from_user(&iocPocControlMessage.poc_control[0], buf, NUM_POC_CHANNELS) != 0){ 
       return -EFAULT;
    }

    mutex_lock(&ipc_lock);
    if (0 != ioc_fd)
    {    
        iocPocControlMessage.messageNumber    = IPC_MSG_POC_CONTROL;
        iocPocControlMessage.interfaceVersion = INTERFACE_VERSION;
        iocPocControlMessage.request_write    = 1;

        originalFs = get_fs();
        set_fs(get_ds());
        ret = vfs_write(ioc_fd, (char*)&iocPocControlMessage,
                        sizeof(iocPocControlMessage), &offset);
        set_fs(originalFs);
    }
    mutex_unlock(&ipc_lock);

    if(-1 == ret) 
    {
        return -EFAULT;
    }
   
    return count;
}

//
// Write to this node to enable/disable vics reset line
//
static ssize_t vics_reset_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    mm_segment_t originalFs;
    unsigned long long offset = 0;
    int ret = 0;
    struct 
    {
        unsigned char messageNumber;
        unsigned char interfaceVersion;
        unsigned char vics_decoder_state;
    } iocVicsResetMessage;
    
    if (0 == count) {
        return 0;
    }
    
    // Only use the first byte even if multiple bytes are written
    if (copy_from_user(&iocVicsResetMessage.vics_decoder_state, buf, 1) != 0){ 
        return -EFAULT;
    }
    
    // Value can be either 0, 1, '0', or '1'
    // The ASCII values are just to make testing easier
    if ((0 == iocVicsResetMessage.vics_decoder_state) ||
        ('0' == iocVicsResetMessage.vics_decoder_state)) {
        iocVicsResetMessage.vics_decoder_state = VICS_RESET_ACTIVE;
    } else if ((1 == iocVicsResetMessage.vics_decoder_state) ||
        ('1' == iocVicsResetMessage.vics_decoder_state)) {
        iocVicsResetMessage.vics_decoder_state = VICS_RESET_INACTIVE;
    } else {
        ret = 1;
    }
    
    if (0 == ret) {
        iocVicsResetMessage.messageNumber    = IPC_MSG_VICS_RESET_CONTROL;
        iocVicsResetMessage.interfaceVersion = INTERFACE_VERSION;

        mutex_lock(&ipc_lock);
        if (0 != ioc_fd) {
            originalFs = get_fs();
            set_fs(get_ds());
            ret = vfs_write(ioc_fd, (char*)&iocVicsResetMessage,
                            sizeof(iocVicsResetMessage), &offset);
            set_fs(originalFs);
        } else {
           ret = -1;
        }
        mutex_unlock(&ipc_lock);
    }

    if(-1 == ret) 
    {
        return -EFAULT;
    }
   
    return count;
}


static ssize_t camera_touch_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    mm_segment_t originalFs;
    unsigned long long offset = 0;
    int ret = 0;

    struct 
    {
        unsigned char messageNumber;
        unsigned char interfaceVersion;
        unsigned char touchMessage[7];
    } iocCameraTouchMessage;
    
    if (4 != count && count != 7)
    {
      return -EFAULT;
    }

    // Get the action from the caller
    if (copy_from_user(&iocCameraTouchMessage.touchMessage[0], buf, count) != 0)
    {
        return -EFAULT;
    }

    mutex_lock(&ipc_lock);
    if (0 != ioc_fd)
    {    
        iocCameraTouchMessage.messageNumber    = IPC_MSG_TOUCH;
        iocCameraTouchMessage.interfaceVersion = (count == 4 ? 0 : 1);

        originalFs = get_fs();
        set_fs(get_ds());
        ret = vfs_write(ioc_fd, (char*)&iocCameraTouchMessage,\
                        count + 2, &offset);
        set_fs(originalFs);
    }
    mutex_unlock(&ipc_lock);

    if(-1 == ret) 
    {
        return -EIO;
    }
    else 
    {
	/* Update anybody read()/poll()ing /dev/camera_touch */
	struct vehicle_data_dev *vdata_dev;
	if ((vdata_dev = vdata_start_update(VDATA_ID_CAMERA_TOUCH))){
		memcpy(camera_touch_data,
			&iocCameraTouchMessage.touchMessage[0], 
			count);
		vdata_end_update(vdata_dev);
	}
    }
   
    return count;
}

/***
 * Custom "read" function for vehicle_camera_config to try retreiving latest available
 * data from the VIP.
 */
static ssize_t vehicle_camera_config_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
	struct vehicle_data_file_private *vdata_fp = filp->private_data;
	struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev;
	char msgBuf[] = {IPC_MSG_VEHICLE_CAMERA_CONFIG, INTERFACE_VERSION};
	int  data_ready = 1;
	int  ret = -EFAULT;

	if (count < vdata_dev->data_size){
		return -EINVAL;
	}

	if ( !(filp->f_flags & O_NONBLOCK) ){
		long timeout_jiff = msecs_to_jiffies(25);
		int  retries = 10; // 10x retries (after 25ms) will delay up to
		                  // 250ms total

		/* Get latest seq number to make sure returned data is from
		 * AFTER the call to read. */
		mutex_lock(&vdata_fp->vdata_dev->lock);
		vdata_fp->seq = vdata_fp->vdata_dev->seq;
		mutex_unlock(&vdata_fp->vdata_dev->lock);

		do {
			/* Request VIP to updated branding info */
			earlypm_ipc_send(msgBuf, sizeof(msgBuf));

			/* Wait for response */
			data_ready = wait_event_interruptible_timeout(vdata_dev->waitq,
			    vdata_available(vdata_fp), timeout_jiff);
		} while (!data_ready && retries--);
		pr_debug("%s : data_ready=%d, retries=%d\n", __func__, data_ready, retries);
	}

	if ( data_ready ){
		int copy_result;
		mutex_lock(&vdata_dev->lock);
		copy_result = copy_to_user(buf, vdata_dev->data, vdata_dev->data_size);
		if (copy_result == 0){
			vdata_fp->seq = vdata_dev->seq;
		}
		mutex_unlock(&vdata_dev->lock);

		if ( !copy_result ){
			ret = vdata_dev->data_size;
		}
	} else {
		pr_warn("%s : Timeout getting vehicle_camera_config from VIP!\n", __func__);
		ret = -EIO;
	}

	return ret;
}

/***
 * Custom "read" function for branding_info to try retreiving latest available
 * data from the VIP.
 */
static ssize_t branding_info_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
	struct vehicle_data_file_private *vdata_fp = filp->private_data;
	struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev; 
	char msgBuf[] = {IPC_MSG_BRANDING, INTERFACE_VERSION};
	int  data_ready = 1;
	int  ret = -EFAULT;

	if (count < vdata_dev->data_size){
		return -EINVAL;
	}

	if ( !(filp->f_flags & O_NONBLOCK) ){
		long timeout_jiff = msecs_to_jiffies(25);
		int  retries = 3; // 3x retries (after 25ms) will delay up to
		                  // 100ms total

		/* Get latest seq number to make sure returned data is from
		 * AFTER the call to read. */
		mutex_lock(&vdata_fp->vdata_dev->lock);
		vdata_fp->seq = vdata_fp->vdata_dev->seq;
		mutex_unlock(&vdata_fp->vdata_dev->lock);

		do {
			/* Request VIP to updated branding info */
			earlypm_ipc_send(msgBuf, sizeof(msgBuf));

			/* Wait for response */
			data_ready = wait_event_interruptible_timeout(vdata_dev->waitq,
			    vdata_available(vdata_fp), timeout_jiff);
		} while (!data_ready && retries--);
		pr_debug("%s : data_ready=%d, retries=%d\n", __func__, data_ready, retries);
	}

	if ( data_ready ){
		int copy_result;
		mutex_lock(&vdata_dev->lock);
		copy_result = copy_to_user(buf, vdata_dev->data, vdata_dev->data_size);
		if (copy_result == 0){
			vdata_fp->seq = vdata_dev->seq;
		}
		mutex_unlock(&vdata_dev->lock);

		if ( !copy_result ){
			ret = vdata_dev->data_size;
		}
	} else {
		pr_warn("%s : Timeout getting branding_info from VIP!\n", __func__);
		ret = -EIO;
	}

	return ret;
}

/* Write to 'branding_info' node to force refreshing
   latest available values. */
static ssize_t branding_info_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
	unsigned char value;
	char msgBuf[] = {IPC_MSG_BRANDING, INTERFACE_VERSION};
	/* Get user write value */
	if ( count < 1 || copy_from_user(&value, buf, 1) != 0)
	{
		return -EFAULT;
	}

	if ( value == 1 || value == '1') {
		pr_info("%s: request branding_info update\n", __func__);
		earlypm_ipc_send(msgBuf, sizeof(msgBuf));
		return count;
	} else {
		return -EINVAL;
	}
}


/*  Currently IOC is not supporting this request.
 *  Write to 'ecall_assist_sos' node to request current call status.
 *  Note: To be used only if IOC supports SOC request status
*/
static ssize_t ecall_assist_sos_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
        unsigned char value = -1;
        char msgBuf[] = {IPC_MSG_ECALL_ASSIST_SOS_STATUS, INTERFACE_VERSION, 0x01};

        /* Get user write request value */
        if ( count < 1 || copy_from_user(&value, buf, 1) != 0){
                pr_err("%s: failure!!. count: %d\n", __func__, (int)count);
                return -EFAULT;
        }

        if ( value == 0x1 ) { //As per ipc spec, request value for Ecall is 0x01
                pr_info("%s: request ecall_assist_sos status \n", __func__);
                earlypm_ipc_send(msgBuf, sizeof(msgBuf));
                return count;
        } else {
                pr_err("TODO: handle this failure case!!\n");
                return -EINVAL;
        }
}

/*  'write' funtion to send Camera Light request meesages to IOC
*/
static ssize_t lvds_cam_ctrl_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
   char uData = 0;
   ssize_t ret = 0;
   char msgBuf[] = {IPC_MSG_INTERIORCAM_LIGHT_REQ, INTERFACE_VERSION, 0x00};

   if ( count < 1 ){
      pr_err("%s: failure!!. data size not supported : %d\n", __func__, (int)count);
      return -EFAULT;
   }

   /* Get user write request value */
   if ( copy_from_user(&uData, buf, 1) != 0){
      pr_err("%s: copy_from_user failed!!. count: %d\n", __func__, (int)count);
      return -EFAULT;
   }

   msgBuf[2] = uData;
   ret = earlypm_ipc_send(msgBuf, sizeof(msgBuf));

   if(ret < 0)
   {
      pr_err("%s: failure!! IPC write failed, count = %d, ret = %d\n", __func__, (int)count, (int)ret);
      return -EIO;
   }

   return count;
}

static ssize_t camera_disp_stat_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
	struct vehicle_data_dev *vdata_dev;
        char msgBuf[] = {IPC_MSG_CAMERA_DISP_STAT, INTERFACE_VERSION, 0x00};

        if ( count < 1 || copy_from_user(&msgBuf[2], buf, 1) != 0){
                pr_err("%s: failure!!. count: %d\n", __func__, (int)count);
                return -EFAULT;
        }

	/* Update internal state of last written value so other processes can
	 * read back this value. This signal is otherwise one-way. */
	if ((vdata_dev = vdata_start_update(VDATA_ID_CAMERA_DISP_STAT))){
		camera_disp_stat = msgBuf[2];
		vdata_end_update(vdata_dev);
	}

	earlypm_ipc_send(msgBuf, sizeof(msgBuf));
	return count;
}

static ssize_t camera_disp_req_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
	struct vehicle_data_dev *vdata_dev;
        char msgBuf[] = {IPC_MSG_CAMERA_DISP_REQ, INTERFACE_VERSION, 0x00};

        if ( count < 1 || copy_from_user(&msgBuf[2], buf, 1) != 0){
                pr_err("%s: failure!!. count: %d\n", __func__, (int)count);
                return -EFAULT;
        }

	/* Update internal state of last written value so other processes can
	 * read back this value. This signal is otherwise one-way. */
	if ((vdata_dev = vdata_start_update(VDATA_ID_CAMERA_DISP_REQ))){
		camera_disp_req = msgBuf[2];
		vdata_end_update(vdata_dev);
	}

	earlypm_ipc_send(msgBuf, sizeof(msgBuf));
	return count;
}
   
static ssize_t ffcm_dynamic_guideline_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
   char msgBuf[] = {IPC_MSG_FFCM_CAM_DYNAMIC_GRIDLINE_STATE, INTERFACE_VERSION, 0x01, 0x00};
   int  sendLen = 3;
   
   //0x01 0xXX => set operation
   //0x00 0x00 => get operation
   if ( count < 1 || copy_from_user(&msgBuf[2], buf, 2) != 0){ //copy two bytes
      printk(KERN_ERR "%s: failure!!. count: %d\n", __func__, (int)count);
      return -EFAULT;
   }

   if(msgBuf[2] == 0x01) //SET operation expects 4 bytes, and GET expects 3 bytes.
      sendLen = 4;
   
   /* Request VIP to process this request*/
   earlypm_ipc_send(msgBuf, sendLen);
   
   return count;

}

static ssize_t ffcm_dynamic_guideline_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   struct vehicle_data_file_private *vdata_fp = filp->private_data;
   struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev;
   int result = 0;

   mutex_lock(&vdata_dev->lock);
   result = copy_to_user(buf, (unsigned char *)&ffcm_dynamic_guideline, sizeof(ffcm_dynamic_guideline));
   if (result == 0){
		vdata_fp->seq = vdata_dev->seq;
	}
   printk(KERN_ERR "%s: returned %d\n", __func__, ffcm_dynamic_guideline);
   mutex_unlock(&vdata_dev->lock);
   
   if(result){
      pr_err("%s: failure!!: %d\n", __func__, result);
      return -EFAULT;
   }
   return sizeof(ffcm_dynamic_guideline);
}

static ssize_t amb_sens_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
	struct vehicle_data_file_private *vdata_fp = filp->private_data;
	struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev; 
	char msgBuf[] = {IPC_MSG_LIGHT_AMB_SENS, INTERFACE_VERSION};
	int  data_ready = 1;
	int  ret = -EFAULT;

	if (count < vdata_dev->data_size){
		return -EINVAL;
	}

	if ( (!(filp->f_flags & O_NONBLOCK)) || vdata_fp->vdata_dev->seq == 0/*first read call*/){ 
		long timeout_jiff = msecs_to_jiffies(25);
		int  retries = 3; // 3x retries (after 25ms) will delay up to 100ms total

		/* Get latest seq number to make sure returned data is from
		 * AFTER the call to read. */
		mutex_lock(&vdata_fp->vdata_dev->lock);
		vdata_fp->seq = vdata_fp->vdata_dev->seq;
		mutex_unlock(&vdata_fp->vdata_dev->lock);

		do {
			/* Request VIP to updated branding info */
			earlypm_ipc_send(msgBuf, sizeof(msgBuf));

			/* Wait for response */
			data_ready = wait_event_interruptible_timeout(vdata_dev->waitq,
			    vdata_available(vdata_fp), timeout_jiff);
		} while (!data_ready && retries--);
		pr_debug("%s : data_ready=%d, retries=%d\n", __func__, data_ready, retries);
	}

	if ( data_ready ){
		int copy_result;
		mutex_lock(&vdata_dev->lock);
		copy_result = copy_to_user(buf, vdata_dev->data, vdata_dev->data_size);
		if (copy_result == 0){
			vdata_fp->seq = vdata_dev->seq;
		}
		mutex_unlock(&vdata_dev->lock);

		if ( !copy_result ){
			ret = vdata_dev->data_size;
		}
	} else {
		pr_warn("%s : Timeout getting amb sens info from VIP!\n", __func__);
		ret = -EIO;
	}

	return ret;
}

static ssize_t wall_alert_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
	struct vehicle_data_file_private *vdata_fp = filp->private_data;
	struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev; 
	char msgBuf[] = {IPC_MSG_LVDS_WALL_ASCM_ALERT_STAT, INTERFACE_VERSION};
	int  data_ready = 1;
	int  ret = -EFAULT;

	if (count < vdata_dev->data_size){
		return -EINVAL;
	}

	if ( (!(filp->f_flags & O_NONBLOCK)) || vdata_fp->vdata_dev->seq == 0/*first read call*/){ 
		long timeout_jiff = msecs_to_jiffies(25);
		int  retries = 3; // 3x retries (after 25ms) will delay up to 100ms total

		/* Get latest seq number to make sure returned data is from
		 * AFTER the call to read. */
		mutex_lock(&vdata_fp->vdata_dev->lock);
		vdata_fp->seq = vdata_fp->vdata_dev->seq;
		mutex_unlock(&vdata_fp->vdata_dev->lock);

		do {
			/* Request VIP to updated branding info */
			earlypm_ipc_send(msgBuf, sizeof(msgBuf));

			/* Wait for response */
			data_ready = wait_event_interruptible_timeout(vdata_dev->waitq,
			    vdata_available(vdata_fp), timeout_jiff);
		} while (!data_ready && retries--);
		pr_debug("%s : data_ready=%d, retries=%d\n", __func__, data_ready, retries);
	}

	if ( data_ready ){
		int copy_result;
		mutex_lock(&vdata_dev->lock);
		copy_result = copy_to_user(buf, vdata_dev->data, vdata_dev->data_size);
		if (copy_result == 0){
			vdata_fp->seq = vdata_dev->seq;
		}
		mutex_unlock(&vdata_dev->lock);

		if ( !copy_result ){
			ret = vdata_dev->data_size;
		}
	} else {
		pr_warn("%s : Timeout getting wall_alerts info from VIP!\n", __func__);
		ret = -EIO;
	}

	return ret;
}


/*  SRS:1014135 : provide command line inteface to enable verbose logs of cambcc module
*/
static ssize_t lvds_verbose_control_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
   struct vehicle_data_dev *vdata_dev;
   uint8_t val[NUM_LVDS_PORTS] = {0};
   uint8_t Camport = 0;
   if ( count < 2 || copy_from_user(val, buf, NUM_LVDS_PORTS) != 0){
      pr_err("%s: failure!!. count: %d\n", __func__, (int)count);
      return -EFAULT;
   }

   if(val[0] > NUM_LVDS_PORTS || val[0] <= 0){
      pr_err("%s: not supported command for Camport value 0x%x!!\n", __func__, (int)val[0]);
      return -EIO;
   }
   if((val[1] != 1) && (val[1] != 0)){
      pr_err("%s: not supported command for Camport value 0x%x and settting 0x%x!!\n", __func__, (int)val[0], (int)val[1]);
      return -EIO;
   }
   Camport = val[0];
   if ((vdata_dev = vdata_start_update(VDATA_ID_LVDS_VERBOSE))){
      lvds_verbose_control[Camport-1] = val[1];
      vdata_end_update(vdata_dev);
   }
   return count;
}

static ssize_t virtual_wall_stat_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   struct vehicle_data_file_private *vdata_fp = filp->private_data;
   struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev;
   char msgBuf[] = {IPC_MSG_VIRTUAL_WALL_REQ, INTERFACE_VERSION, 0x00};
   int  data_ready = 1;

   if (vdata_fp->vdata_dev->seq == 0){ /*for first read call*/
      long timeout_jiff = msecs_to_jiffies(25);
      int  retries = 4; // 4x retries (after 25ms) will delay up to 100ms total

      /* Get latest seq number to make sure returned data is from
       * AFTER the call to read. */
      mutex_lock(&vdata_fp->vdata_dev->lock);
      vdata_fp->seq = vdata_fp->vdata_dev->seq;
      mutex_unlock(&vdata_fp->vdata_dev->lock);

      do {
         /* Request VIP to updated branding info */
         earlypm_ipc_send(msgBuf, sizeof(msgBuf));

         /* Wait for response */
         data_ready = wait_event_interruptible_timeout(vdata_dev->waitq,vdata_available(vdata_fp),timeout_jiff);
      } while (!data_ready && retries--);
      pr_debug("%s : data_ready=%d, retries=%d\n", __func__, data_ready, retries);
   }

   if ( data_ready ){
      int copy_result;
      if ( count > vdata_dev->data_size) {
         count = vdata_dev->data_size;
      }
      mutex_lock(&vdata_dev->lock);
      copy_result = copy_to_user(buf, vdata_dev->data, count);
      if (copy_result == 0){
         vdata_fp->seq = vdata_dev->seq;
      }
      mutex_unlock(&vdata_dev->lock);

      if ( copy_result )
         return -EFAULT;
      else return count;
   } else {
      pr_warn("%s : Timeout getting virtual wall info from VIP!\n", __func__);
      return -EIO;
   }
}

static ssize_t virtual_wall_stat_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
   char msgBuf[] = {IPC_MSG_VIRTUAL_WALL_REQ, INTERFACE_VERSION, 0x01, 0x00};
   int  sendLen = 3;

   //0x01 0xXX => set operation
   //0x00 0x00 => get operation
   if ( count < 2 || copy_from_user(&msgBuf[2], buf, 2) != 0){ //copy two bytes
      pr_err("%s: failure!!. count: %d not supported\n", __func__, (int)count);
      return -EFAULT;
   }

   if(msgBuf[2] == 0x01) //SET operation expects 4 bytes, and GET expects 3 bytes.
      sendLen = 4;

   /* Request VIP to process this request*/
   earlypm_ipc_send(msgBuf, sendLen);

   return count;
}

static ssize_t ldvsRqst_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    struct vehicle_data_file_private *vdata_fp = filp->private_data;
    struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev;
    int copy_result = 1;
    if ( count > (sizeof(uint8_t) * BUFFER_LEN)){
       pr_err("%s: failure!!. data count: %d more than supported count %d\n", __func__, (int)count, (int)(sizeof(uint8_t) * BUFFER_LEN));
       return -EFAULT;
    }
    if ((vdata_dev = vdata_start_update(VDATA_ID_LVDS_COMMAND_RQST))){
       copy_result = copy_from_user(vdata_dev->data,buf,count);
       vdata_end_update(vdata_dev);
    }
    if(copy_result)
       return -EFAULT;
    else
       return count;
}

static ssize_t diagnosticStat_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    int  sendLen = (sizeof(struct diagnosticStat) + 2);//5 byte payload + 2 byte header (MSD ID + INTERFACE_VERSION)
    char msgBuf[10] = {IPC_MSG_DIAGNOSTIC_STATUS, INTERFACE_VERSION, 0x00, 0x00, 0x00, 0x00, 0x00};

    memset(&diagStat, 0, sizeof(struct diagnosticStat));
    if ( count < 1 || copy_from_user(&diagStat, buf, sizeof(struct diagnosticStat)) != 0){
        printk(KERN_ERR "%s: failure!!. count: %d\n", __func__, (int)count);
        return -EFAULT;
    }
    msgBuf[2] = diagStat.camera_type;
    msgBuf[3] = diagStat.lvds_connection_stat;
    msgBuf[4] = diagStat.lvds_internal_err_stat;
    msgBuf[5] = diagStat.lvds_external_err_stat;
    msgBuf[6] = diagStat.video_signal_stat;
    /* Request VIP to process this request*/
    earlypm_ipc_send(msgBuf, sendLen);

    return count;
}

static ssize_t ldvsResp_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    struct vehicle_data_file_private *vdata_fp = filp->private_data;
    struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev;
    int copy_result = 1;
    if ( count > (sizeof(uint8_t) * BUFFER_LEN)){
       pr_err("%s: failure!!. data count: %d more than supported count %d\n", __func__, (int)count, (int)(sizeof(uint8_t) * BUFFER_LEN));
       return -EFAULT;
    }
    if ((vdata_dev = vdata_start_update(VDATA_ID_LVDS_COMMAND_RESP))){
       copy_result = copy_from_user(vdata_dev->data,buf,count);
       vdata_end_update(vdata_dev);
    }
    if(copy_result)
       return -EFAULT;
    else
       return count;
}

static const struct file_operations fops_backlight = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .poll         = backlight_poll,
   .read         = backlight_read,
   .write        = backlight_write,
   .release      = channel_close,
};

static const struct file_operations fops_actions = {
   .owner        = THIS_MODULE,
   .open         = actions_open,
   .poll         = actions_poll, // temp!!!
   .read         = actions_read,
   .write        = actions_write,
   .release      = actions_close,
};

static const struct file_operations fops_camera_delay = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .poll         = camera_delay_poll,
   .read         = camera_delay_read,
   .write        = camera_delay_write,
   .release      = channel_close,
};

static const struct file_operations fops_camera_gridlines = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .poll         = camera_gridlines_poll,
   .read         = camera_gridlines_read,
   .write        = camera_gridlines_write,
   .release      = channel_close,
};

static const struct file_operations fops_camera_pos_banner = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .poll         = camera_pos_banner_poll,
   .read         = camera_pos_banner_read,
   .release      = channel_close,
};

static const struct file_operations fops_camera_cancel = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .poll         = camera_cancel_poll,
   .read         = camera_cancel_read,
   .write        = camera_cancel_write,
   .release      = channel_close,
};

static const struct file_operations fops_poc_control = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .poll         = poc_control_poll,
   .read         = poc_control_read,
   .write        = poc_control_write,
   .release      = channel_close,
};

static const struct file_operations fops_vics_reset = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .write        = vics_reset_write,
   .release      = channel_close,
};

//
// 
//
static unsigned int xfer_ab_poll(struct file *filp, struct poll_table_struct *pt)
{
    unsigned int mask = 0;
    
    if(filp->private_data) 
    {
        poll_wait(filp, &xfer_ba_queue, pt);
        if (0 != num_xfer_ba_values)
        {
            mask = POLLIN | POLLRDNORM;
        }
    }
    else
    {
        mask = POLLERR;
    }
    
    return mask;
}

//
// 
//
static ssize_t xfer_ab_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   int read_bytes = -EFAULT;

   if(filp->private_data) 
   {
      // Wait for a new action
      while (0 != wait_event_interruptible(xfer_ba_queue, 0 != num_xfer_ba_values))
      {
         // Keep waiting if the wait was interrupted
      }
      
      if (count > num_xfer_ba_values)
      {
          count = num_xfer_ba_values;
      }
      
      if (0 == copy_to_user(buf, (const void *)(&xfer_ba_values), count))
      {
         num_xfer_ba_values = 0;
         read_bytes = count;
      }
   }

   return read_bytes;
}

//
// 
//
static ssize_t xfer_ab_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    // Get the action from the caller
    if (MAX_XFER_VALUES < count)
    {
        return -EFAULT;
    }

    if (copy_from_user((void *)(&xfer_ab_values), buf, count) != 0)
    {
        return -EFAULT;
    }
    num_xfer_ab_values = count;
    
    wake_up_interruptible(&xfer_ab_queue);    
  
    return count;
}

//
// 
//
static unsigned int xfer_ba_poll(struct file *filp, struct poll_table_struct *pt)
{
    unsigned int mask = 0;
    
    if(filp->private_data) 
    {
        poll_wait(filp, &xfer_ab_queue, pt);
        if (0 != num_xfer_ab_values)
        {
            mask = POLLIN | POLLRDNORM;
        }
    }
    else
    {
        mask = POLLERR;
    }
    
    return mask;
}

//
// 
//
static ssize_t xfer_ba_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
   int read_bytes = -EFAULT;

   if(filp->private_data) 
   {
      // For non-blocking read just return the last values
      if (0 == (filp->f_flags & O_NONBLOCK))
      {
         // Wait for a new action
         while (0 != wait_event_interruptible(xfer_ab_queue, 0 != num_xfer_ab_values))
         {
            // Keep waiting if the wait was interrupted
         }
      }
      
      if (0 == copy_to_user(buf, (const void *)&xfer_ab_values, 1))
      {
         read_bytes = 1;
         num_xfer_ab_values = 0;
      }
   }

   return read_bytes;
}

//
// 
//
static ssize_t xfer_ba_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
    // Get the action from the caller
    if (MAX_XFER_VALUES < count)
    {
        return -EFAULT;
    }

    if (copy_from_user((void *)(&xfer_ba_values), buf, count) != 0)
    {
        return -EFAULT;
    }
    
    num_xfer_ba_values = count;
    
    wake_up_interruptible(&xfer_ba_queue);    
  
    return count;
}

static const struct file_operations fops_early_to_late = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .poll         = xfer_ab_poll,
   .read         = xfer_ab_read,
   .write        = xfer_ab_write,
   .release      = channel_close,
};

static const struct file_operations fops_late_to_early = {
   .owner        = THIS_MODULE,
   .open         = channel_open,
   .poll         = xfer_ba_poll,
   .read         = xfer_ba_read,
   .write        = xfer_ba_write,
   .release      = channel_close,
};

struct earlypm_dev{
   unsigned int minor;
   const char   *name;
   struct cdev node_cdev;
   const struct file_operations *fops;
};

struct earlypm_dev devlist[] = { /* list of minor devices */
   { BACKLIGHT_NODE_MINOR_NUMBER, BACKLIGHT_NODE_NAME, {{0}}, &fops_backlight},
   { ACTIONS_NODE_MINOR_NUMBER,   ACTIONS_NODE_NAME,   {{0}}, &fops_actions},
   { 2,                           "camera_delay",      {{0}}, &fops_camera_delay},
   { 3,                           "camera_pos_banner", {{0}}, &fops_camera_pos_banner},
   { 4,                           "camera_cancel",     {{0}}, &fops_camera_cancel},
   { 5,                           "early_to_late_evs", {{0}}, &fops_early_to_late},
   { 6,                           "late_to_early_evs", {{0}}, &fops_late_to_early},
   { 7,                           "camera_gridlines",  {{0}}, &fops_camera_gridlines},
   { 8,                           "poc_control",       {{0}}, &fops_poc_control},
   { 9,                           "vics_reset",        {{0}}, &fops_vics_reset},
};

#define NUM_NODES (sizeof(devlist)/sizeof(devlist[0]))

struct vehicle_data_dev vdata_devlist [] = {
	{ .id        = VDATA_ID_CAMERA_CONFIG,
	  .name      = "camera_config",
	  .data      = (void*) &camera_cfg,
	  .data_size = sizeof(camera_cfg),
	  .seq       = 0
	}, 
	{ .id        = VDATA_ID_VEHICLE_CAMERA_CONFIG,
	  .name      = "vehicle_camera_config",
	  .data      = (void*) vehicle_camera_cfg,
	  .data_size = sizeof(vehicle_camera_cfg),
	  .custom_read = vehicle_camera_config_read,
	  .seq       = 0
	}, 
	{ .id        = VDATA_ID_VEHICLE_SPEED,
	  .name      = "vehicle_speed",
	  .data      = (void*) &vehicle_speed,
	  .data_size = sizeof(vehicle_speed),
	  .seq       = 0
	}, 
	{ .id        = VDATA_ID_SHIFT_INFO,
	  .name      = "shift_info",
	  .data      = (void*) &shift_info,
	  .data_size = sizeof(shift_info),
	  .seq       = 0
	}, 
	{ .id        = VDATA_ID_IGN_STATUS,
	  .name      = "ignition_status",
	  .data      = (void*) &ign_status,
	  .data_size = sizeof(ign_status),
	  .seq       = 0
	},
	{ .id        = VDATA_ID_CAMERA_ANGLE,
	  .name      = "camera_angle",
	  .data      = (void*) &camera_angle,
	  .data_size = sizeof(camera_angle),
	  .seq       = 0
	},
	{ .id        = VDATA_ID_BRANDING,
	  .name      = "branding_info",
	  .data      = (void*) &branding,
	  .data_size = sizeof(branding),
	  .seq       = 0,
	  .write     = branding_info_write,
	  .custom_read      = branding_info_read
	},
	{ .id        = VDATA_ID_ECALL_SOS,
	  .name      = "ecall_assist_sos",
	  .data      = (void*) &ecall_sos_state,
	  .data_size = sizeof(ecall_sos_state),
	  .seq       = 0,
	  .write     = ecall_assist_sos_write,
	},
	{ .id        = VDATA_ID_INTERIORCAM_LIGHT_REQ,
	  .name      = "interiorCam_Light_req",
	  .write     = lvds_cam_ctrl_write,
	},
	{ .id        = VDATA_ID_CAMERA_DISP_STAT,
	  .name      = "camera_disp_stat",
	  .data      = (void*) &camera_disp_stat,
	  .data_size = sizeof(camera_disp_stat),
	  .seq       = 0,
	  .write     = camera_disp_stat_write,
	},
	{ .id        = VDATA_ID_CAMERA_DISP_REQ,
	  .name      = "camera_disp_req",
	  .data      = (void*) &camera_disp_req,
	  .data_size = sizeof(camera_disp_req),
	  .seq       = 0,
	  .write     = camera_disp_req_write,
	},
	{ .id        = VDATA_ID_SVC_STATUS,
	  .name      = "camera_svc_status",
	  .data      = (void*) &svc_status,
	  .data_size = sizeof(svc_status),
	  .seq       = 0,
	},
    { .id        = VDATA_ID_FFCM_CAM_DYNAMIC_GRIDLINE_STATE,
	  .name      = "ffcm_dynamic_guideline",
	  .data      = (void*) &ffcm_dynamic_guideline,
	  .data_size = sizeof(ffcm_dynamic_guideline),
	  .seq       = 0,
	  .write     = ffcm_dynamic_guideline_write,
	  .custom_read      = ffcm_dynamic_guideline_read,
	},
	{ .id        = VDATA_ID_CAMERA_TOUCH,
	  .name      = "camera_touch",
	  .data      = (void*) &camera_touch_data,
	  .data_size = sizeof(camera_touch_data),
	  .seq       = 0,
	  .write     = camera_touch_write,
	},
	{ .id        = VDATA_ID_LVDS_WALL_ASCM_ALERT_STAT,
	  .name      = "lvds_wall_ascm_stat",
	  .data      = (void*) &lvds_alerts,
	  .data_size = sizeof(lvds_alerts),
	  .seq       = 0,
	  .custom_read  = wall_alert_read,
	},
	{ .id        = VDATA_ID_LIGHT_AMB_SENS,
	  .name      = "amb_sens_day_lgt_md",
	  .data      = (void*) &amb_sens_day_lgt_md,
	  .data_size = sizeof(amb_sens_day_lgt_md),
	  .seq       = 0,
	  .custom_read  = amb_sens_read,
	},
	{ .id        = VDATA_ID_LVDS_VERBOSE,
	  .name      = "lvds_verbose_control",
	  .data      = (void*) &lvds_verbose_control,
	  .data_size = sizeof(lvds_verbose_control),
	  .seq       = 0,
	  .write     = lvds_verbose_control_write,
	},
	{ .id        = VDATA_ID_VIRTUAL_WALL,
	  .name      = "virtual_wall_stat",
	  .data      = (void*) &virtual_wall_stat,
	  .data_size = sizeof(virtual_wall_stat),
	  .seq       = 0,
	  .custom_read  = virtual_wall_stat_read,
	  .write     = virtual_wall_stat_write,
	},
	{ .id        = VDATA_ID_LVDS_COMMAND_RESP,
	  .name      = "lvds_cmd_resp",
	  .data      = (void*) &lvdsCmdResp,
	  .data_size = sizeof(lvdsCmdResp),
	  .seq       = 0,
	  .write     = ldvsResp_write,
	},
	{ .id        = VDATA_ID_LVDS_COMMAND_RQST,
	  .name      = "lvds_cmd_rqst",
	  .data      = (void*) &lvdsCmdRqst,
	  .data_size = sizeof(lvdsCmdRqst),
	  .seq       = 0,
	  .write     = ldvsRqst_write,
	},
    { .id        = VDATA_ID_DIAGNOSTIC_STATUS,
	  .name      = "diagnostic_stat",
	  .data      = (void*) &diagStat,
	  .data_size = sizeof(struct diagnosticStat),
	  .seq       = 0,
	  .write     = diagnosticStat_write,
	},
};

#define MIN_VDATA_MINORNUM (NUM_NODES)
#define NUM_VDATA_NODES (sizeof(vdata_devlist)/sizeof(vdata_devlist[0]))


// Function to open a generic "vehicle data" node
static int vdata_open(struct inode *inode, struct file *filp)
{

	struct vehicle_data_dev  *vdata_dev;
	struct vehicle_data_file_private *vdata_fp;
	vdata_dev = container_of(inode->i_cdev, struct vehicle_data_dev, cdev);
	pr_info("opening vehicle data node: %s\n", vdata_dev->name);

	vdata_fp = kzalloc(sizeof (struct vehicle_data_file_private), GFP_KERNEL);
	if ( vdata_fp )
	{
		vdata_fp->seq       = 0;
		vdata_fp->vdata_dev = vdata_dev;
		filp->private_data = vdata_fp;
	} else {
		return -ENOMEM;
	}
	return SUCCESS;
}

// Function to vehicle data node
static int vdata_close(struct inode *inode, struct file *filp)
{
   if(filp->private_data) 
   {
      // Free up the data allocated in the open command
      kfree(filp->private_data);
      filp->private_data = 0;
      return SUCCESS;
   }
   else
   {
      return -EINVAL;
   }
}

/* Helper for poll and non-blocking read to know when data available */
static int vdata_available(struct vehicle_data_file_private *vdata_fp)
{
	int result = 0;
	mutex_lock(&vdata_fp->vdata_dev->lock);
	result = (vdata_fp->seq != vdata_fp->vdata_dev->seq);
	mutex_unlock(&vdata_fp->vdata_dev->lock);
	return result;
}

/* Poll function for generic "vehicle data" nodes */
static unsigned int vdata_poll(struct file *filp, struct poll_table_struct *pt)
{
	struct vehicle_data_file_private *vdata_fp = filp->private_data;
	struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev;
	unsigned int mask = 0;
	poll_wait(filp, &vdata_dev->waitq, pt);
	if ( vdata_available(vdata_fp) ) {
		mask = POLLIN | POLLRDNORM;
	}
	return mask;
}

/***
 * Read function for generic vehicle data nodes.
 * Anything using this fops.read() routine will have the following behavior:
 *
 * blocking reads will wait for a change in data since the last time read() was
 * called.
 *
 * non-blocking reads will turn the latest available value
 *
 * If underlying data value changes multiple times between sequential read
 * calls, intermediate values are dropped, not queued.
 */
static ssize_t vdata_read(struct file *filp, char __user *buf,
      size_t count, loff_t *f_pos)
{
	struct vehicle_data_file_private *vdata_fp = filp->private_data;
	struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev; 
	int copy_result;

	if (vdata_dev->custom_read){
		return vdata_dev->custom_read(filp, buf, count, f_pos);
	}

	if ( !(filp->f_flags & O_NONBLOCK) ){
		if ( 0 != wait_event_interruptible(vdata_dev->waitq, 
				vdata_available(vdata_fp)) ){
			return -EINTR;
		}
	} 

	mutex_lock(&vdata_dev->lock);
	/* Check length while lock is held since some nodes update data length
	   when new message arrives. Holding vdata_dev lock guarantees the length
	   is consistent with what's actually present in a buffer */
	if ( count > vdata_dev->data_size) {
		/* Only fill userspace buffer w/ available bytes */
		count = vdata_dev->data_size; 
	} else if (count < vdata_dev->data_size){
		/* If user requests LESS THAN available bytes, log a warning,
		   but allow an incomplete read. This allows for easier future
		   expansion (assuming new bytes are appended) */
		pr_info("%s: incomplete read (len=%d, avail=%d)\n", __func__,
			(int) count, (int) vdata_dev->data_size);
	}

	copy_result = copy_to_user(buf, vdata_dev->data, count);
	if (copy_result == 0){
		vdata_fp->seq = vdata_dev->seq;
	}
	mutex_unlock(&vdata_dev->lock);

	if ( copy_result ){
		return -EFAULT;
	}

	return count;
}

static ssize_t vdata_write(struct file *filp, const char __user *buf,
      size_t count, loff_t *f_pos)
{
	struct vehicle_data_file_private *vdata_fp = filp->private_data;
	struct vehicle_data_dev *vdata_dev = vdata_fp->vdata_dev; 
	if (vdata_dev->write){
		return vdata_dev->write(filp, buf, count, f_pos);
	} else {
		return -EINVAL;
	}
}


/**
 * Prolog func for starting update of a vehicle data parameter/structure. This
 * puts all the locking & thread synchronization in one place.
 */
static struct vehicle_data_dev * vdata_start_update(int id)
{
	struct vehicle_data_dev* dev = NULL;
	if ( (dev = get_vdata_dev(id)) ){
		mutex_lock(&dev->lock);
	}
	return dev;
}

static void vdata_end_update(struct vehicle_data_dev *dev)
{
	if (dev){
		dev->seq++;
		mutex_unlock(&dev->lock);
		wake_up_interruptible(&dev->waitq);
	}
}

/***
 */
static struct vehicle_data_dev * get_vdata_dev(int id)
{
	int i;
	for(i=0; i < NUM_VDATA_NODES; ++i){
		if ( vdata_devlist[i].id == id )
			return &vdata_devlist[i];
	}
	return NULL;
}

static const struct file_operations fops_vdata = {
   .owner        = THIS_MODULE,
   .open         = vdata_open,
   .poll         = vdata_poll,
   .read         = vdata_read,
   .write        = vdata_write,
   .release      = vdata_close,
};


static int register_channels(void)
{
   int init_result = 0;
   dev_t devt;
   int i;
   //int register_err;

   majorNumber = register_chrdev(0, DEVICE_NAME, &channel_fops);

   if (0 > majorNumber) 
   {
      return FAILURE;
   }

   dev_class = class_create(THIS_MODULE, DEVICE_NAME);
   if (IS_ERR(dev_class))
   {
      unregister_chrdev(majorNumber, DEVICE_NAME);
      return FAILURE;
   }
   
   // Initialize the nodes
   for (i = 0; i < NUM_NODES; i++) 
   {
      // Initialize the node
      cdev_init(&devlist[i].node_cdev, devlist[i].fops);
      devt = MKDEV(majorNumber, devlist[i].minor);
      init_result = cdev_add(&devlist[i].node_cdev, devt, 1);
      if (init_result < 0) 
      {
         // remove the device
         device_destroy(dev_class, MKDEV(majorNumber, 0));     
      
         // unregister the device class
         class_unregister(dev_class);                          
      
         // remove the device class
         class_destroy(dev_class);                             
      
         // unregister the major number
         unregister_chrdev(majorNumber, DEVICE_NAME);            
   
         return FAILURE;
      }
   
      if (IS_ERR(device_create(dev_class, NULL, MKDEV(majorNumber,
            devlist[i].minor), NULL, devlist[i].name)))
      {
         // remove the device
         device_destroy(dev_class, MKDEV(majorNumber, 0));     
      
         // unregister the device class
         class_unregister(dev_class);                          
      
         // remove the device class
         class_destroy(dev_class);                             
      
         // unregister the major number
         unregister_chrdev(majorNumber, DEVICE_NAME);            
   
         return FAILURE;
      }
   }

   for (i = 0; i < NUM_VDATA_NODES; i++)  {
      unsigned int minorNumber = vdata_devlist[i].id + MIN_VDATA_MINORNUM;
      mutex_init(&vdata_devlist[i].lock);
      init_waitqueue_head(&vdata_devlist[i].waitq);
      cdev_init(&vdata_devlist[i].cdev, &fops_vdata);
      devt = MKDEV(majorNumber, minorNumber);
      init_result = cdev_add(&vdata_devlist[i].cdev, devt, 1);
      if (init_result < 0) {
	      pr_err("TODO: handle this failure case!!\n");
      }

      if (IS_ERR(device_create(dev_class, NULL, MKDEV(majorNumber,
            minorNumber), NULL, vdata_devlist[i].name))){
	      pr_err("TODO: handle this failure case!!\n");
      }
   }
   
   return SUCCESS;
}


static int __init r1_backlight_init(void)
{
   int ret = 0;
   
   mutex_init(&actions_lock);
   mutex_init(&display_lock);
   mutex_init(&ipc_lock);
   mutex_init(&camera_delay_lock);
   mutex_init(&camera_gridlines_lock);
   mutex_init(&camera_pos_banner_lock);
   mutex_init(&camera_cancel_lock);
   init_waitqueue_head(&actions_queue);
   init_waitqueue_head(&backlight_queue);
   init_waitqueue_head(&camera_delay_queue);
   init_waitqueue_head(&camera_gridlines_queue);
   init_waitqueue_head(&camera_pos_banner_queue);
   init_waitqueue_head(&camera_cancel_queue);
   init_waitqueue_head(&xfer_ab_queue);
   init_waitqueue_head(&xfer_ba_queue);
   init_waitqueue_head(&poc_control_queue);


   iocReaderThread = kthread_run(IOCDataReader, 0, "IOCDataReader");
   if (iocReaderThread)
   {
      if (SUCCESS != register_channels())
      {
         printk(KERN_ERR "[R1 Backlight] Failed to create nodes\n");
         kthread_stop(iocReaderThread);
         iocReaderThread = 0;
         ret = -1;
      }
   }
   else
   {
      printk(KERN_ERR "[R1 Backlight] Failed to create IOC reader thread\n");
      ret = -1;
   }

   return SUCCESS;
}

static void __exit r1_backlight_exit(void)
{
  
   // Kill the data reader thread
   if(iocReaderThread)
   {
      kthread_stop(iocReaderThread);
      iocReaderThread = 0;
   }

   // Close the interface with the VIP
   if (ioc_fd)
   {
      filp_close(ioc_fd, NULL);
      ioc_fd = 0;
   }

   if (0 <= majorNumber)
   {
      // remove the device
      device_destroy(dev_class, MKDEV(majorNumber, 0));     
      
      // unregister the device class
      class_unregister(dev_class);                          
      
      // remove the device class
      class_destroy(dev_class);                             
      
      // unregister the major number
      unregister_chrdev(majorNumber, DEVICE_NAME);            
   }
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DEVICE_NAME);
MODULE_AUTHOR("Scott Mackay");
MODULE_VERSION("0.1");

module_init(r1_backlight_init);
module_exit(r1_backlight_exit);
