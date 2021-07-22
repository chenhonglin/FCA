/***
  This file is part of PulseAudio.

  Copyright 2004-2008 Lennart Poettering

  PulseAudio is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published
  by the Free Software Foundation; either version 2.1 of the License,
  or (at your option) any later version.

  PulseAudio is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with PulseAudio; if not, see <http://www.gnu.org/licenses/>.
***/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <pulse/rtclock.h>
#include <pulse/timeval.h>
#include <pulse/xmalloc.h>

#include <pulsecore/i18n.h>
#include <pulsecore/macro.h>
#include <pulsecore/sink.h>
#include <pulsecore/module.h>
#include <pulsecore/core-util.h>
#include <pulsecore/modargs.h>
#include <pulsecore/log.h>
#include <pulsecore/thread.h>
#include <pulsecore/thread-mq.h>
#include <pulsecore/rtpoll.h>

#include <avbMaif.h>

#define DEFAULT_DEVICE "default"

#define AVB_FRAME_SIZE  	(64)   /* 1 frame == 64 samples */
#define AVB_HW_ENABLE		1
#define AVB_PCM_DUMP_ENABLE	0

PA_MODULE_AUTHOR("Lennart Poettering");
PA_MODULE_DESCRIPTION(_("Clocked NULL sink"));
PA_MODULE_VERSION(PACKAGE_VERSION);
PA_MODULE_LOAD_ONCE(false);
PA_MODULE_USAGE(
        "sink_name=<name of sink> "
        "sink_properties=<properties for the sink> "
        "format=<sample format> "
        "rate=<sample rate> "
        "channels=<number of channels> "
        "channel_map=<channel map>");

#define DEFAULT_SINK_NAME "null"
#define BLOCK_USEC (PA_USEC_PER_SEC * 2)

struct userdata {
    pa_core *core;
    pa_module *module;
    pa_sink *sink;

    pa_thread *thread;
    pa_thread_mq thread_mq;
    pa_rtpoll *rtpoll;

    pa_usec_t block_usec;
    pa_usec_t timestamp;

    pa_memchunk memchunk;
    size_t frame_size;
    int talker_index;
    int talker_wridx;
    int talker_wrcnt;
    int talker_buf_num;
    uint16_t *txMaifTlkrBufPtr[AVTP_NUM_PAYLOADBLOCKS_RINGBUF];

};

typedef enum {
        AVB_DEVICE_INVALID = -1,
        AVB_DEVICE_AVB_VRM = 0,
        AVB_DEVICE_AVB_MIM,
        AVB_DEVICE_AVB_MAX,
}pa_avb_dev_t;

static const char* const valid_modargs[] = {
    "sink_name",
    "sink_properties",
    "format",
    "rate",
    "channels",
    "channel_map",
    NULL
};
void pa__done(pa_module*m);

#if AVB_PCM_DUMP_ENABLE
#define DUMP_DATA_START		(8*1000)
#define DUMP_DATA_LENGTH	(6*1000)
#define DUMP_DATA_NAME	"/mnt/pulseaudio/pcm-avb-sink"
typedef enum {
        AVB_DUMP_FE = 0,
        AVB_DUMP_BE,
        AVB_DUMP_MAX,
}pa_avb_dump_t;
static int pfile_pcm[AVB_DEVICE_AVB_MAX][AVB_DUMP_MAX] = {{-1,-1},{-1,-1}};

static int audio_file_open(int *pfile, char*name, int isRead)
{
    if(*pfile == -1)
    {
        if(isRead)
        {
               *pfile = open(name, O_RDONLY, 0);
        }
        else
        {
               *pfile = open(name, O_RDWR|O_CREAT|O_TRUNC, 0);
         }
    }
    if(*pfile == -1)
    {
       printf("%s: open file %s fail!\n", __FUNCTION__,name);
       return -1;
    }
    return 0;
}

static int audio_file_close(int *pfile)
{
    close(*pfile);
    *pfile = -1;

    return 0;
}

static int audio_file_read(int *pfile, uint16_t *buffer, int size)
{
    int reads;
    if(*pfile == -1)
    {
       printf("%s: file doesn't exist!\n", __FUNCTION__);
       return -1;
    }

    reads = read(*pfile, (char *)buffer, size);
    return reads;
}

static int audio_file_write(int *pfile, uint16_t *buffer, int size)
{
    int writes;
    if(*pfile == -1)
    {
       printf("%s: file doesn't exist!\n", __FUNCTION__);
       return -1;
    }

    writes = write(*pfile, (char *)buffer, size);
    return writes;
}
static void pa_avb_dump_data(int count, int talker,pa_avb_dump_t type,uint16_t *buffer, int size)
{
        int *pfile;
        char name[128];

        pfile = &pfile_pcm[talker][type];
        if(count==DUMP_DATA_START) //starts from 10s after starts up
        {
                if(-1 == *pfile)
                {
                        sprintf(name, "%s_%d_%d.raw", DUMP_DATA_NAME,talker,type);
                        audio_file_open(pfile,name, 0);
                        pa_log_error("--------%s:%s file open!\n", __func__,name);
                }
        }
        if(*pfile != -1)
        {
                audio_file_write(pfile, buffer,size);
        }
        if(count == (DUMP_DATA_START + DUMP_DATA_LENGTH))  //dump for 30s
        {
                audio_file_close(pfile);
                pa_log_error("--------%s:%s file close!\n", __func__,name);
        }
}
#endif

static int writeAvbTxBuf(struct userdata *u,int frames, uint16_t* audBuf)
{
    int isample = 0,chn;
    int icnt = 0;
    uint16_t data_le = 0;
    uint16_t data_be = 0;
    uint16_t pushPktCnt = 0;
    uint16_t *destBuf = NULL;
    size_t frame_size;

    frame_size =pa_frame_size(&u->sink->sample_spec);
#if AVB_PCM_DUMP_ENABLE
    u->talker_wrcnt++;
    pa_avb_dump_data(u->talker_wrcnt,u->talker_index,AVB_DUMP_FE,audBuf,frames*pa_frame_size(&u->sink->sample_spec));
#endif

#if AVB_HW_ENABLE
        if(frames%AVB_FRAME_SIZE != 0){
                pa_log_error("%s: error unsupported frames (%d)!\n", __func__, frames);
                return -1;
        }
        else{
                pushPktCnt = frames/AVB_FRAME_SIZE;
                if(pushPktCnt > u->talker_buf_num)
                {
                        pa_log_error("%s: avb buffer is not enough buf num=%d!\n", __func__, u->talker_buf_num);
                }
                for(icnt=0; icnt<pushPktCnt; icnt++){
                        destBuf = u->txMaifTlkrBufPtr[u->talker_wridx];
                        for(isample=0; isample<AVB_FRAME_SIZE; isample++){
                                for(chn = 0; chn< u->sink->sample_spec.channels;chn++){
                                        data_le = audBuf[icnt*AVB_FRAME_SIZE*u->sink->sample_spec.channels+isample*u->sink->sample_spec.channels+chn];
                                        destBuf[isample*u->sink->sample_spec.channels+chn] = data_le;
                                }
                       }
#if AVB_PCM_DUMP_ENABLE
                        pa_avb_dump_data(u->talker_wrcnt,u->talker_index,AVB_DUMP_BE,destBuf,AVB_FRAME_SIZE*pa_frame_size(&u->sink->sample_spec));
#endif
                        u->talker_wridx=(u->talker_wridx+1)%(u->talker_buf_num);
                        incAudioIFaceTalkerCounter1722Blocks(u->talker_index);

                }
        }
#endif
    return 0;
}

static void avb_write(struct userdata *u) {
        void *p;
        size_t frames;
    if (u->memchunk.length <= 0){
        pa_sink_render(u->sink, u->sink->thread_info.max_request, &u->memchunk);
    }
     pa_assert(u->memchunk.length > 0);

     frames = u->memchunk.length / u->frame_size;
     p = pa_memblock_acquire(u->memchunk.memblock);
     writeAvbTxBuf(u,frames,(uint16_t *)((const uint8_t*) p + u->memchunk.index));
     pa_memblock_release(u->memchunk.memblock);

     pa_memblock_unref(u->memchunk.memblock);
     pa_memchunk_reset(&u->memchunk);
}
static int sink_process_msg(
        pa_msgobject *o,
        int code,
        void *data,
        int64_t offset,
        pa_memchunk *chunk) {

    struct userdata *u = PA_SINK(o)->userdata;

    switch (code) {
        case PA_SINK_MESSAGE_SET_STATE:

            if (PA_PTR_TO_UINT(data) == PA_SINK_RUNNING)
                u->timestamp = pa_rtclock_now();

            switch (PA_PTR_TO_UINT(data)) {
                case PA_SINK_SUSPENDED: {
                    pa_assert(PA_SINK_IS_OPENED(u->sink->thread_info.state));
                    pa_log_info("#PA_SINK_SET SUSPENDED,idx=%d.\r\n",u->talker_index);
                    setAudioIFaceTalkerState(u->talker_index,SST_IDLE);
                    break;
                }
                case PA_SINK_IDLE:
                case PA_SINK_RUNNING: {
                    if (u->sink->thread_info.state == PA_SINK_SUSPENDED) {
                        pa_log_info("#PA_SINK_SET RUNNING,idx=%d.\r\n",u->talker_index);
                        setAudioIFaceTalkerState(u->talker_index,SST_TALKING);
                    }
                    break;
                }
                case PA_SINK_UNLINKED:
                case PA_SINK_INIT:
                case PA_SINK_INVALID_STATE:
                default:
                break;
            }
            break;

        case PA_SINK_MESSAGE_GET_LATENCY: {
            pa_usec_t now;
            now = pa_rtclock_now();
            *((pa_usec_t*) data) = u->timestamp > now ? u->timestamp - now : 0ULL;

            return 0;
        }
        case PA_SINK_MESSAGE_ADD_DATA:{
            if(PA_SINK_IS_OPENED(u->sink->thread_info.state))
            {
                avb_write(u);
            }
            break;
        }
        default:
                break;
    }

    return pa_sink_process_msg(o, code, data, offset, chunk);
}

static void sink_update_requested_latency_cb(pa_sink *s) {
    struct userdata *u;
    size_t nbytes;

    pa_sink_assert_ref(s);
    pa_assert_se(u = s->userdata);

    u->block_usec = pa_sink_get_requested_latency_within_thread(s);

    if (u->block_usec == (pa_usec_t) -1)
        u->block_usec = s->thread_info.max_latency;

    nbytes = pa_usec_to_bytes(u->block_usec, &s->sample_spec);
    pa_sink_set_max_rewind_within_thread(s, nbytes);
    pa_sink_set_max_request_within_thread(s, nbytes);
}

static void process_rewind(struct userdata *u) {

    pa_assert(u);
    /*if avb provide interface to get buffer avb driver buffer info, then avb can support rewind.*/
    pa_sink_process_rewind(u->sink, 0);
}

static void thread_func(void *userdata) {
    struct userdata *u = userdata;

    pa_assert(u);

    pa_log_debug("Thread starting up");

    pa_thread_mq_install(&u->thread_mq);

    u->timestamp = pa_rtclock_now();

    for (;;) {
        int ret;
        if (PA_UNLIKELY(u->sink->thread_info.rewind_requested))
            process_rewind(u);
        /* Hmm, nothing to do. Let's sleep */
        if ((ret = pa_rtpoll_run(u->rtpoll)) < 0)
            goto fail;

        if (ret == 0)
            goto finish;
    }

fail:
    /* If this was no regular exit from the loop we have to continue
     * processing messages until we received PA_MESSAGE_SHUTDOWN */
    pa_asyncmsgq_post(u->thread_mq.outq, PA_MSGOBJECT(u->core), PA_CORE_MESSAGE_UNLOAD_MODULE, u->module, 0, NULL, NULL);
    pa_asyncmsgq_wait_for(u->thread_mq.inq, PA_MESSAGE_SHUTDOWN);

finish:
    pa_log_debug("Thread shutting down");
}
pa_avb_dev_t pa_parse_avb_device(const char *sink) {
    pa_assert(sink);

    if (strstr(sink, "vrm"))
    {
        return AVB_DEVICE_AVB_VRM;
    }
    else if (strstr(sink, "mim"))
    {
        return AVB_DEVICE_AVB_MIM;
    }
    else
    {
    	return AVB_DEVICE_INVALID;
    }
}

int pa__init(pa_module*m) {
    struct userdata *u = NULL;
    pa_sample_spec ss;
    pa_channel_map map;
    pa_modargs *ma = NULL;
    pa_sink_new_data data;
    size_t nbytes;
    const char *sink_name;
    #ifdef MAIF_64
    uint64_t addrBlocks[AVTP_NUM_PAYLOADBLOCKS_RINGBUF]={0};
    #else
    uint32_t addrBlocks[AVTP_NUM_PAYLOADBLOCKS_RINGBUF]={0};
    #endif
    uint32_t numBlocks=0;
    uint8_t j=0;
    char *e;

    pa_assert(m);

    if (!(ma = pa_modargs_new(m->argument, valid_modargs))) {
        pa_log("Failed to parse module arguments.");
        goto fail;
    }

    ss = m->core->default_sample_spec;
    map = m->core->default_channel_map;
    if (pa_modargs_get_sample_spec_and_channel_map(ma, &ss, &map, PA_CHANNEL_MAP_DEFAULT) < 0) {
        pa_log("Invalid sample format specification or channel map");
        goto fail;
    }


    m->userdata = u = pa_xnew0(struct userdata, 1);
    u->core = m->core;
    u->module = m;
    u->rtpoll = pa_rtpoll_new();
    pa_thread_mq_init(&u->thread_mq, m->core->mainloop, u->rtpoll);


    if (sink_name = pa_modargs_get_value(ma, "sink_name", NULL))
    {
        u->talker_index = pa_parse_avb_device(sink_name);
        if(u->talker_index == AVB_DEVICE_INVALID)
        {
            pa_log("Invalid AVB device\r\n");
            goto fail;
        }
        pa_log_debug("#avb sink =%d. name =%s  load.\r\n",u->talker_index,sink_name);
    }
    u->talker_wrcnt = 0;
    u->talker_wridx = 0;
    u->talker_buf_num = 0;



#if AVB_HW_ENABLE
    /* Create MAIF in the reserved physical memory */
   if( 0 != createAudioIFace(MAIF_CFG_OWNER,OCMC1_START,OCMC1_SIZE,0))
   {
       pa_log_error("%s: createAudioIFace failed!\n", __func__);
       goto fail;
   }
   else
   {
    /* Get the virtual addresses of all the payload blocks for each talker stream.*/
       u->talker_buf_num=getAudioIFaceBufferPtrTalker( u->talker_index, &addrBlocks[0] );
       for(j=0;j<u->talker_buf_num;j++)
       {
              u->txMaifTlkrBufPtr[j]=(uint16_t*)addrBlocks[j];
       }
       pa_log_debug("%s: createAudioIFace successful! buf_num=%d.\r\n", __func__,u->talker_buf_num);
   }
#endif

    pa_sink_new_data_init(&data);
    data.driver = __FILE__;
    data.module = m;
    pa_sink_new_data_set_name(&data, pa_modargs_get_value(ma, "sink_name", DEFAULT_SINK_NAME));
    pa_sink_new_data_set_sample_spec(&data, &ss);
    pa_sink_new_data_set_channel_map(&data, &map);
    e = pa_sprintf_malloc("avb Output %s", data.name);
    pa_proplist_sets(data.proplist, PA_PROP_DEVICE_DESCRIPTION,e);
    pa_proplist_sets(data.proplist, PA_PROP_DEVICE_CLASS, "abstract");
    pa_xfree(e);
    if (pa_modargs_get_proplist(ma, "sink_properties", data.proplist, PA_UPDATE_REPLACE) < 0) {
        pa_log("Invalid properties");
        pa_sink_new_data_done(&data);
        goto fail;
    }

    u->sink = pa_sink_new(m->core, &data, PA_SINK_LATENCY|PA_SINK_DYNAMIC_LATENCY);
    pa_sink_new_data_done(&data);

    if (!u->sink) {
        pa_log("Failed to create sink object.");
        goto fail;
    }

    u->sink->parent.process_msg = sink_process_msg;
    u->sink->update_requested_latency = sink_update_requested_latency_cb;
    u->sink->userdata = u;

    pa_sink_set_asyncmsgq(u->sink, u->thread_mq.inq);
    pa_sink_set_rtpoll(u->sink, u->rtpoll);

    u->block_usec = BLOCK_USEC;
    nbytes = pa_usec_to_bytes(u->block_usec, &u->sink->sample_spec);
    pa_sink_set_max_rewind(u->sink, nbytes);
    pa_sink_set_max_request(u->sink, nbytes);
    u->frame_size = pa_frame_size(&ss);

    if (!(u->thread = pa_thread_new("avb-sink", thread_func, u))) {
        pa_log("Failed to create thread.");
        goto fail;
    }

    pa_sink_set_latency_range(u->sink, 0, BLOCK_USEC);

    pa_sink_put(u->sink);

    pa_modargs_free(ma);

    return 0;

fail:
    if (ma)
        pa_modargs_free(ma);

    pa__done(m);

    return -1;
}

int pa__get_n_used(pa_module *m) {
    struct userdata *u;

    pa_assert(m);
    pa_assert_se(u = m->userdata);

    return pa_sink_linked_by(u->sink);
}

void pa__done(pa_module*m) {
    struct userdata *u;

    pa_assert(m);

    if (!(u = m->userdata))
        return;

    if (u->sink)
        pa_sink_unlink(u->sink);

    if (u->thread) {
        pa_asyncmsgq_send(u->thread_mq.inq, NULL, PA_MESSAGE_SHUTDOWN, NULL, 0, NULL);
        pa_thread_free(u->thread);
    }

    pa_thread_mq_done(&u->thread_mq);

    if (u->sink)
        pa_sink_unref(u->sink);

    if (u->rtpoll)
        pa_rtpoll_free(u->rtpoll);

    pa_xfree(u);
}
