/*  
 *  native_touch.c
 */
#include <linux/module.h>	/* Needed by all modules */
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/init.h>      // included for __init and __exit macros
#include <linux/errno.h>
#include<linux/fs.h> /*this is the file structure, file open read close */
#include<linux/cdev.h> /* this is for character device, makes cdev avilable*/
#include<linux/uaccess.h> /*this is for copy_user vice vers*/
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/wait.h> // Required for the wait queues
#include <linux/poll.h>


#define DEVICE_NAME "native_touch"

static int pm=0;
struct cdev mcdev;			/*this is the name of my char driver that i will be registering*/
static struct class *cl;
int major_number;			/* will store the major number extracted by dev_t*/
int ret;					/*used to return values*/
dev_t dev_num;				/*will hold the major number that the kernel gives*/

static wait_queue_head_t queue;


unsigned int event_path_global;
EXPORT_SYMBOL(event_path_global);

static unsigned int last_native_x_pos;

struct touch_info
{
	unsigned int x_pos;
	unsigned int y_pos;
	unsigned int z_pos;
};

struct touch_info ntouch;


void update_native_touch_coordinates(unsigned int native_x_pos, unsigned int native_y_pos, unsigned int native_z_pos)
{
   static int last_touch_was_release = 0;
   
   // Make sure that we do not swamp the system with continuous touch releases
   if ((native_x_pos == 0) && (native_y_pos == 0))
   {
      if (last_touch_was_release)
      {
         return;
      }
      else
      {
         last_touch_was_release = 1;
      }
   }
   else
   {
      last_touch_was_release = 0;
   }
	ntouch.y_pos = native_y_pos;
	ntouch.z_pos = native_z_pos;
	ntouch.x_pos = native_x_pos;   
   wake_up_interruptible(&queue);
}
EXPORT_SYMBOL(update_native_touch_coordinates);

static int native_touch_open(struct inode *inode, struct file *filp)
{
	unsigned int minor = iminor(inode);
        ntouch.x_pos = -1;
        ntouch.y_pos = -1;
        ntouch.z_pos = -1;
#ifdef DEBUG
	printk("native_touch: Open succesfully. minor: %d \n", minor);
#endif
	return 0;
}

static int native_touch_close(struct inode *inode, struct file *filp)
{
#ifdef DEBUG
    printk("native_touch: device has been closed.\n");
#endif
    // Wake up any blocked calls
    ntouch.x_pos = -2;
    ntouch.y_pos = -2;
    ntouch.z_pos = -2;

    wake_up_interruptible(&queue);
    
    // Revert back to standard touch interface
    event_path_global = 0;
    return ret;
}

static ssize_t native_touch_read(struct file *fp, char *buff, size_t length, loff_t *ppos)
{
#ifdef DEBUG
    printk("native_touch: device has been read x=%x y=%d z=%d \n", ntouch.x_pos, ntouch.y_pos, ntouch.z_pos);
#endif
 
      // For non-blocking read just return the last values
      if (0 == (fp->f_flags & O_NONBLOCK))
      {
         // Wait for a new action
         while (0 != wait_event_interruptible(queue, (-1 != ntouch.x_pos)))
         {
            // Keep waiting if the wait was interrupted
         }
      }


	if(copy_to_user(buff, &ntouch, sizeof(struct touch_info))){
		printk("native_touch: copy to user failed!!!\n");
		return -EFAULT;
	}
	ntouch.x_pos = -1;
	ntouch.y_pos = -1;
	ntouch.z_pos = -1;
	last_native_x_pos = ntouch.x_pos;
   return length;
}

static ssize_t native_touch_write(struct file *fp, const char *buff, size_t length, loff_t *ppos)
{

	if (copy_from_user(&event_path_global, buff, sizeof(int))) {
		printk("copy from user failed!!!\n");
		return -EFAULT;
	}
#ifdef DEBUG
    printk("native_touch : device has been written: %d\n", event_path_global);
#endif
    return length;
}



#if 0
int native_touch_ioctl(struct file *filp, unsigned int ioctl_cmd, unsigned long arg)
{
     
	printk("native_touch : device ioctl: cmd: %d\n", ioctl_cmd);

	return 0;
}
#endif
static unsigned int native_touch_poll(struct file *filp, struct poll_table_struct *pt)
{
    unsigned int mask = 0;
    
    poll_wait(filp, &queue, pt);
    if (-1 != ntouch.x_pos)
    {
        mask = POLLIN | POLLRDNORM;
    }
    
    return mask;
}



struct file_operations fops = { 
    .owner = THIS_MODULE, /*prevents unloading when operations are in use*/
    .open = native_touch_open,  /*to open the device*/
    .write = native_touch_write, /*to write to the device*/
    .read = native_touch_read, /*to read the device*/
    .poll = native_touch_poll, /* to poll for new input */
    .release = native_touch_close, /*to close the device*/
//    .unlocked_ioctl = native_touch_ioctl,
};

static int __init native_touch_init(void)
{
	int rc = -1;

	init_waitqueue_head(&queue);

	ret = -1;
	ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
	if(ret < 0) {
		printk("native_touch :failed to allocate major num\n");
		return ret;
	}
	if ((cl = class_create( THIS_MODULE, "native_touch_dev" )) == NULL ){
		printk("%s: native_touch: Class creation failed\n");
		unregister_chrdev_region( dev_num, 1 );
		return -1;
	}
	if (device_create(cl, NULL, dev_num, NULL, "native_touch") == NULL ){
		printk("native_touch: Device creation failed\n");
		class_destroy(cl);
		unregister_chrdev_region(dev_num, 1);
		return -1;
	}

	major_number = MAJOR(dev_num);
	cdev_init(&mcdev, &fops);
	mcdev.owner = THIS_MODULE;

	/*we have created and initialized our cdev structure.
	  Now we need to add it to the kernel*/
	ret = cdev_add(&mcdev,dev_num,1);
	if(ret < 0) {
		printk("native_touch : device adding to the kernel failed\n");
		device_destroy( cl, dev_num);
		class_destroy( cl );
		unregister_chrdev_region(dev_num, 1);
		return ret;
	}
	printk("native_touch: device additin to the kernel succesful\n");

	/* 
	 * A non 0 return means init_module failed; module can't be loaded. 
	 */
	return 0;
}

static void __exit native_touch_exit(void)
{
	cdev_del(&mcdev); /*removing the structure that we added previously*/
	device_destroy(cl, dev_num);
	class_destroy(cl);
	unregister_chrdev_region(dev_num,1);
}

module_init(native_touch_init);
module_exit(native_touch_exit);

MODULE_AUTHOR("Harman India, <sathishkumar.selvaraj2@harman.com>");
MODULE_DESCRIPTION("NATIVE TOUCH MODULE"); 
MODULE_LICENSE("GPL");

