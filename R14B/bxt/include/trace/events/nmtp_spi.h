#undef TRACE_SYSTEM
#define TRACE_SYSTEM nmtp_spi

#if !defined(INCLUDE_TRACE_EVENTS_NMTP_SPI_H_) || defined(TRACE_HEADER_MULTI_READ)
#define INCLUDE_TRACE_EVENTS_NMTP_SPI_H_

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(nmtp,

	TP_PROTO(u32 fifoId, char *name, u8 mf, u8 segment_order,
			int fifo_length),

	TP_ARGS(fifoId, name, mf, segment_order, fifo_length),

	TP_STRUCT__entry(
		__field(	u32,        	fifoId		)
		__field(	char *,		name		)
		__field(	u8,		mf	)
		__field(	u8,		segment_order	)
		__field(	int,		fifo_length	)
	),

	TP_fast_assign(
		__entry->fifoId = fifoId;
		__entry->name = name;
		__entry->mf = mf;
		__entry->segment_order = segment_order;
		__entry->fifo_length = fifo_length;
		
	),

	TP_printk("nmtp_fill_kfifo: fifoId %d :%s  MF= %d, SO %d fifoLen %x",
	__entry->fifoId, __entry->name, __entry->mf, __entry->segment_order,
								__entry->fifo_length)
);

DEFINE_EVENT(nmtp, nmtp_fill_kfifo,

	TP_PROTO(u32 fifoId, char *name, u8 mf, u8 segment_order,
			int fifo_length),
	TP_ARGS(fifoId, name, mf, segment_order, fifo_length)

);

#endif /* INCLUDE_TRACE_EVENTS_NMTP_SPI_H_ */

/* This part must be outside protection */
#include <trace/define_trace.h>
