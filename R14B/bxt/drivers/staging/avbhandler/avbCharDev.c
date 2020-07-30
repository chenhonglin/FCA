/*****************************************************************************
 * Project         HARMAN AVB Stack(2.0)
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 * Secrecy Level   STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          avbCharDev.c
 * @author        Siva Kumar Mellemputi
 *
 * AVB Character device driver file
 */

/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/
#include "osIncludes.h"
#include "main.h"

/*---------------------------------------------------------------------
 * DEFINES
 *--------------------------------------------------------------------*/
#define CHAR_DEV_NAME "avbdev"
#define CLASS_NAME  "avb"

#ifdef CONFIG_INCREASED_AVB_MAPPING_SIZE
#define MAX_MEM_USERS  20         // max number of applications that can use memory simultaneously
#else
#define MAX_MEM_USERS  6         // max number of applications that can use memory simultaneously
#endif

/*---------------------------------------------------------------------
 * EXTERNAL
 *--------------------------------------------------------------------*/
extern t_LocalUnchachedMemory avbBuf;
/*---------------------------------------------------------------------
 * STRUCTURES
 *--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * CONST
 *--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * VARIABLES
 *--------------------------------------------------------------------*/
static uint32_t majorNum = 0;
static int8_t inuse = 0;
static struct class*  avbCharClass  = NULL;
struct device* avbCharDevice = NULL;

/*---------------------------------------------------------------------
 * FUNCTION PROTOTYPES
 *--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * FUNCTION IMPLEMENTATION
 *--------------------------------------------------------------------*/
/**
 * Function to handle open operation from userspace and validate access 
 * returns:	EXIT_SUCCESS on success
 */
static int avbDevOpen(struct inode *inode, struct file *file)
{
	if (inuse >= MAX_MEM_USERS) 
	{
		logError("%s: %s: Device(%s) busy", APP_NAME, __FUNCTION__, CHAR_DEV_NAME);
		return -EBUSY;
	}
	try_module_get(THIS_MODULE);
	inuse++;
	logDebug2("%s: Open operation invoked",APP_NAME);
	return EXIT_SUCCESS;
}

/**
 * Function to handle release operations from userspace
 * returns:	EXIT_SUCCESS on success
 */
static int avbDevRelease(struct inode *inode, struct file *file)
{
	module_put(THIS_MODULE);
	inuse--;
	logDebug2("%s: Close operation invoked",APP_NAME);
	return EXIT_SUCCESS;
}

/**
 * Function to provide physical address of AVB memory
 * returns:	size (number of bytes copied) on success
 */
static ssize_t avbDevRead(struct file *file, char *buf, size_t lbuf, loff_t *offset)
{
   int err = 0;
   ssize_t size = sizeof(uint64_t);
	
   if(buf == NULL)
	{
		logError("%s: %s: ERROR: NULL pointer input", APP_NAME, __FUNCTION__);
		return EINVAL;
	}
	if(lbuf < size)
	{
		logError("%s: %s: ERROR: Insufficient buffer size", APP_NAME, __FUNCTION__);
		return EINVAL;		
	}
	
	err = copy_to_user(buf, (char *)(&avbBuf.physBuffer), size);
   if (err==0)
	{
      logDebug2("%s: Sent PHYSICAL address(0x%08x%08x) to the user", APP_NAME,(uint32_t)(avbBuf.physBuffer>>32), (uint32_t)avbBuf.physBuffer);
   }
   else 
	{
      logError("%s: %s: ERROR: Failed to send PHYSICAL address to the user", APP_NAME, __FUNCTION__);
      return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
   }
	return size;
}

/**
 * Function to map AVB memory to userspace
 * returns:	Success.
 */
static int avbMemMap(struct file *file, struct vm_area_struct* vma)
{
	logDebug2("%s: mmap() requested", APP_NAME);
	return dma_mmap_coherent(avbCharDevice, vma, avbBuf.buffer, avbBuf.physBuffer, avbBuf.size);
}

static struct file_operations avbCharDevfOps = {
	.owner = THIS_MODULE,
	.open = avbDevOpen,
	.read = avbDevRead,
	.mmap = avbMemMap,
	.release = avbDevRelease
};

/**
 * Function to create and register AVB character device
 * returns:	EXIT_SUCCESS on success, EXIT_FAILURE on failure
 */
int createAvbDev(void)
{
   // Try to dynamically allocate a major number for the device
	majorNum = register_chrdev (0 , CHAR_DEV_NAME ,	&avbCharDevfOps);
	if(majorNum < 0)
	{
		logError("%s: %s: Cannot register AVB character device", APP_NAME, __FUNCTION__);
		return EXIT_FAILURE;
	}
	
   // Register the device class
   avbCharClass = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(avbCharClass))
	{
      unregister_chrdev(majorNum, CHAR_DEV_NAME);
      logError("%s: %s: Failed to register device class", APP_NAME, __FUNCTION__);
      return EXIT_FAILURE;
   }
   logDebug1("%s: device class registered correctly", APP_NAME);

   // Register the device driver
   avbCharDevice = device_create(avbCharClass, NULL, MKDEV(majorNum, 0), NULL, CHAR_DEV_NAME);
   if (IS_ERR(avbCharDevice))
	{
      class_destroy(avbCharClass);
      unregister_chrdev(majorNum, CHAR_DEV_NAME);
      logError("%s: %s: Failed to create the device", APP_NAME, __FUNCTION__);
      return EXIT_FAILURE;
   }
   logDebug1("%s: device class created correctly", APP_NAME);
	logInfo("%s: Registered AVB character device(%s) with Major no. - %d ", APP_NAME, CHAR_DEV_NAME ,majorNum);
	
	return EXIT_SUCCESS;
}

/**
 * Function to unregister/destroy AVB character device
 * return void
 */
void destroyAvbDev(void)
{
	if(!IS_ERR(avbCharDevice))
	{
		device_destroy(avbCharClass, MKDEV(majorNum, 0));     // remove the device		
	}
	else
	{
		logError("%s: %s: Device(%s) not created", APP_NAME, __FUNCTION__, CHAR_DEV_NAME);
	}
   if (!IS_ERR(avbCharClass))
	{
		class_unregister(avbCharClass);                       // unregister the device class
		class_destroy(avbCharClass);									// remove the device class
	}
	else
	{
		logError("%s: %s: Class(%s) not registered", APP_NAME, __FUNCTION__, CLASS_NAME);
	}
	if(majorNum <= 0)
	{
	  unregister_chrdev(majorNum, CHAR_DEV_NAME);
	}
	else
	{
		logError("%s: %s: Device(%s) not registered", APP_NAME, __FUNCTION__, CHAR_DEV_NAME);
	}
	
	logInfo("%s: Unregistered character driver (%s)..\n", APP_NAME, CHAR_DEV_NAME);
	return;
}

