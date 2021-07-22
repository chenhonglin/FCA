/*****************************************************************************
 * Project         HARMAN AVB Stack(2.0)
 *
 * (c) copyright   2018
 * Company         Harman International
 *                 All rights reserved
 * Secrecy Level   STRICTLY CONFIDENTIAL
 *****************************************************************************/
/**
 * @file          main.c
 * @author        Siva Kumar Mellemputi (ioctl support)
 *                Keshavamurthy Sannegowda (AVB memory management)
 *                Ramakumar Badipatla (AVB memory management)
 *
 * Kernel module to allocate AVB memory
 */

/*---------------------------------------------------------------------
 * INCLUDES
 *--------------------------------------------------------------------*/
#include "osIncludes.h"
#include "main.h"

/*---------------------------------------------------------------------
 * DEFINES
 *--------------------------------------------------------------------*/
/* Linux kernel module inits. */
MODULE_AUTHOR("Harman");
MODULE_DESCRIPTION(APP_NAME);
MODULE_LICENSE("GPL");

#ifdef CONFIG_INCREASED_AVB_MAPPING_SIZE
#define AVB_DMA_ADDRESS_BITS    32
#define AVB_DEFAULT_MEM_SIZE    0x400000 //mapping 4MB
#else
#define AVB_DMA_ADDRESS_BITS    32
#define AVB_DEFAULT_MEM_SIZE    0x200000 //mapping 4MB
#endif


/*---------------------------------------------------------------------
 * EXTERNAL
 *--------------------------------------------------------------------*/
extern struct device* avbCharDevice;

/*---------------------------------------------------------------------
 * STRUCTURES
 *--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * CONST
 *--------------------------------------------------------------------*/

/*---------------------------------------------------------------------
 * VARIABLES
 *--------------------------------------------------------------------*/
int32_t v = -1;
int32_t t = -1;
int32_t size = -1;

module_param(v, int, 0);
MODULE_PARM_DESC(v,"variable to set verbose level");
module_param(t, int, 0);
MODULE_PARM_DESC(t,"variable to set trace output channel");
module_param(size, int, 0);
MODULE_PARM_DESC(v,"variable to set the AVB memory size");

TTrace* TRACE_DEFAULT; 				/**< global trace instance (used as default) */
uint8_t traceChannel=TRACE_CHANNEL_PRINTF;
uint32_t verbose=TRACE_LEVEL_INFO;
uint32_t avbMemSize=AVB_DEFAULT_MEM_SIZE;

uint64_t currTs64 = 0;
DEFINE_LOCAL_UNCHACHED_MEMORY( avbBuf, AVB_DEFAULT_MEM_SIZE )

/*---------------------------------------------------------------------
 * FUNCTION PROTOTYPES
 *--------------------------------------------------------------------*/
int32_t createAvbMem(uint32_t memSize );
void destroyAvbMem( void );
void optScanCommandLine( int32_t argc, char* const* argv );

 /*---------------------------------------------------------------------
 * FUNCTION IMPLEMENTATION
 *--------------------------------------------------------------------*/
 
/**
 * Function to reserve AVB memory
 * returns:	0 on success
 */
int32_t createAvbMem(uint32_t memSize )
{
	avbBuf.size=memSize;
	
	#if 0
	/* Set required bit-addressing for DMA memory */
	if(dma_set_mask(NULL,DMA_BIT_MASK(AVB_DMA_ADDRESS_BITS))==0)
	{
	   dma_set_coherent_mask(NULL,DMA_BIT_MASK(AVB_DMA_ADDRESS_BITS));
	   logInfo("%s: DMA mask is SET for %d-bit addressing ! ", APP_NAME,AVB_DMA_ADDRESS_BITS);
	}
	else
	{
		logError("%s: Failed to SET DMA mask for %d-bit addressing ! ", APP_NAME,AVB_DMA_ADDRESS_BITS);
	}
	#endif
	
	avbBuf.buffer = dma_zalloc_coherent(NULL, avbBuf.size, &avbBuf.physBuffer, GFP_KERNEL);
	if( avbBuf.buffer == NULL )
	{
		logError("%s: Failed to allocate AVB Memory ", APP_NAME);
		return EXIT_FAILURE;
	}
	logInfo("%s: AVB memory allocated at 0x%08x%08x, size-0x%08x ", APP_NAME,(uint32_t)(avbBuf.physBuffer >> 32),(uint32_t)(avbBuf.physBuffer),avbBuf.size);
	return EXIT_SUCCESS;
}

/**
 * Function to free AVB memory
 * returns:	void
 */
void destroyAvbMem( void )
{
	if(avbBuf.buffer != NULL)
	{
		dma_free_coherent(NULL, avbBuf.size, &avbBuf.buffer, GFP_KERNEL);
	}
}

/**
 * Main function for Initializations
 * returns:	void
 */
static void initThread(void)
{
	uint32_t res;
	
	TRACE_DEFAULT = traceInit(_SLOGC_SYSTEM, 0, TRACE_CHANNEL_PRINTF);
	if (NULL == TRACE_DEFAULT)
	{
		logError("%s: Failed to initialize trace ", APP_NAME);
	}

	optScanCommandLine( 0, NULL );
	traceSetChannel(TRACE_DEFAULT, traceChannel);
	traceSetLevel(TRACE_DEFAULT, verbose );

	res = createAvbDev();
	if ( EXIT_FAILURE == res )
	{
		logError("%s: Failed to create avb device ", APP_NAME);
	}
	
	res = createAvbMem(avbMemSize);
	if ( EXIT_FAILURE == res )
	{
		logError("%s: Failed to reserve avb memory ", APP_NAME);
	}
	
	return;
}

static int __init thread_init(void)
{
	initThread();
	return 0;
}

static void __exit thread_exit(void)
{
	destroyAvbMem();
	destroyAvbDev();
	
	if(TRACE_DEFAULT != NULL)
	{
		traceDestroy( TRACE_DEFAULT );
		TRACE_DEFAULT = NULL;
	}
}

module_init(thread_init);
module_exit(thread_exit);
