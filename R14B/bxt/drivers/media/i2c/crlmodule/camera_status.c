/*  
 *  camera_status.c
 */
//#define DEBUG
#include <linux/module.h>	/* Needed by all modules */
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/init.h>      // included for __init and __exit macros
#include <linux/errno.h>
#include <linux/fs.h> /*this is the file structure, file open read close */
#include <linux/cdev.h> /* this is for character device, makes cdev avilable*/
#include <linux/uaccess.h> /*this is for copy_user vice vers*/
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/wait.h> // Required for the wait queues
#include <linux/poll.h>

#define DEVICE_NAME "camera_status"

struct cdev mcdev_camera;			/*this is the name of my char driver that i will be registering*/
static struct class *cl;
int major_number_camera;			/* will store the major number extracted by dev_t*/
dev_t dev_num_camera;				/*will hold the major number that the kernel gives*/
static int camera_type = -1;
static wait_queue_head_t queue;

int check_camera_status(int *camera_status,int camera_type);

static int camera_status_open(struct inode *inode, struct file *filp)
{
	unsigned int minor = iminor(inode);
#ifdef DEBUG
	printk("camera_status: Open succesfully. minor: %d \n", minor);
#endif
	return 0;
}

static int camera_status_close(struct inode *inode, struct file *filp)
{
	int ret = 0;
#ifdef DEBUG
	printk("camera_status: device has been closed.\n");
#endif
	return ret;
}


static ssize_t camera_status_read(struct file *fp, char *buff, size_t length, loff_t *ppos)
{
	int ret = 0;
	unsigned int camera_status = 0;

	pr_debug("camera_status_read: length=%zu, *ppos=%d\n",
		length, (int) *ppos);

	/* If file positition is != 0, return 0 indicating we're at the EOF */
	if (*ppos != 0){
		pr_debug("camera_status_read: *ppos != 0, ret=0\n");
		return 0;
	}

	/* Only allow reading if user buffer is large enough for capturing
	 * camera_status atomically */
	if (length < sizeof(camera_status)){
		pr_debug("camera_status_read: length (%zu) to short. Required %zu\n",
			length, sizeof(camera_status));
		return -EINVAL;
	}

	ret = check_camera_status(&camera_status,camera_type);
	if (ret) {
		printk("error in reading camera_status\n");
		return ret;
	}
#ifdef DEBUG
	printk("camera_status_read = 0x%x\n", camera_status);
#endif
	if(copy_to_user(buff, &camera_status, sizeof(unsigned int))){
		printk("camera_status: copy to user failed!!!\n");
		return -EFAULT;
	}

	/* On successful read, update filepos & return the actual length read
	   to behave like a normal char device */
	*ppos += sizeof(camera_status);
	return sizeof(camera_status);
}

static ssize_t camera_status_write(struct file *fp, const char *buff, size_t length, loff_t *ppos)
{
#ifdef DEBUG
	printk("camera_status : device has been written: %d\n");
#endif
	if(copy_from_user(&camera_type,buff,sizeof(int))){ 
		printk("camera_status: copy from user failed!!!\n");
                return -EFAULT;
	}
	return 0;
}

struct file_operations fops_camera = { 
	.owner = THIS_MODULE,			/*prevents unloading when operations are in use*/
	.open = camera_status_open,		/*to open the device*/
	.write = camera_status_write,	/*to write to the device*/
	.read = camera_status_read,		/*to read the device*/
	.release = camera_status_close,	/*to close the device*/
};

static int __init camera_status_init(void)
{
	int rc = -1;
	int ret = 0;
	extern unsigned char oem_camera_type; // from platform data

	init_waitqueue_head(&queue);

	camera_type = oem_camera_type;
	pr_info("camera_status: set camera_type=oem_camera_type (%d)\n",
		__FILE__, __LINE__, oem_camera_type);

	ret = -1;
	ret = alloc_chrdev_region(&dev_num_camera, 0, 1, DEVICE_NAME);
	if(ret < 0) {
		printk("camera_status :failed to allocate major num\n");
		return ret;
	}
	if ((cl = class_create( THIS_MODULE, "camera_status_dev" )) == NULL ){
		printk("%s: camera_status: Class creation failed\n");
		unregister_chrdev_region( dev_num_camera, 1 );
		return -1;
	}
	if (device_create(cl, NULL, dev_num_camera, NULL, "camera_status") == NULL ){
		printk("camera_status: Device creation failed\n");
		class_destroy(cl);
		unregister_chrdev_region(dev_num_camera, 1);
		return -1;
	}


	major_number_camera = MAJOR(dev_num_camera);
	cdev_init(&mcdev_camera, &fops_camera);
	mcdev_camera.owner = THIS_MODULE;

	/*we have created and initialized our cdev structure.
	  Now we need to add it to the kernel*/
	ret = cdev_add(&mcdev_camera,dev_num_camera,1);
	if(ret < 0) {
		printk("camera_status : device adding to the kernel failed\n");
		device_destroy( cl, dev_num_camera);
		class_destroy( cl );
		unregister_chrdev_region(dev_num_camera, 1);
		return ret;
	}
	printk("camera_status: device additin to the kernel succesful\n");

	/* 
	 * A non 0 return means init_module failed; module can't be loaded. 
	 */
	return 0;
}

static void __exit camera_status_exit(void)
{
	cdev_del(&mcdev_camera); /*removing the structure that we added previously*/
	device_destroy(cl, dev_num_camera);
	class_destroy(cl);
	unregister_chrdev_region(dev_num_camera,1);
}

module_init(camera_status_init);
module_exit(camera_status_exit);

MODULE_AUTHOR("Harman India, <Pratik.Bhatt@harman.com>");
MODULE_DESCRIPTION("CAMERA STATUS MODULE"); 
MODULE_LICENSE("GPL");

