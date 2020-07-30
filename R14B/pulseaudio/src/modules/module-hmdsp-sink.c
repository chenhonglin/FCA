/***
    This file is part of PulseAudio.

    Copyright 2015 Harman COC_Media

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

#include <pulse/gccmacro.h>
#include <pulse/xmalloc.h>
#include <pulse/rtclock.h>
#include <pulse/timeval.h>

#include <pulsecore/namereg.h>
#include <pulsecore/sink.h>
#include <pulsecore/mix.h>
#include <pulsecore/module.h>
#include <pulsecore/core-util.h>
#include <pulsecore/modargs.h>
#include <pulsecore/log.h>
#include <pulsecore/sample-util.h>
#include <pulsecore/thread.h>
#include <pulsecore/thread-mq.h>
#include <pulsecore/protocol-native.h>
#include <pulsecore/pstream-util.h>
#include <pulsecore/core-error.h>

//#include <execinfo.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <dlfcn.h>
#include <signal.h>
#include <string.h>

#include <pulse/ext-dsp-framework.h>
#include "module-hmdsp-sink-symdef.h"
#include "module-hmdsp-sink-config.h"

/* Define to the version of this package. */
#define HMDSP_VERSION (__DATE__)
#define FADE_IN_MS    (64)

#define MEMBLOCKQ_MAXLENGTH (1*1024*1024)
#define MAX_MIX_CHANNELS (32)
#define MIX_BUFFER_LENGTH (PA_PAGE_SIZE)
#define CONVERT_BUFFER_LENGTH (PA_PAGE_SIZE)

PA_MODULE_AUTHOR("COC Media");
PA_MODULE_DESCRIPTION(_("Harman CrossBar"));
PA_MODULE_VERSION(HMDSP_VERSION);
PA_MODULE_LOAD_ONCE(false);
PA_MODULE_USAGE(
        _("sink_name=<name for the sink> "
          "sink_properties=<properties for the sink> "
          "master=<name of sink to filter> "
          "rate=<sample rate> "
          "channels=<number of channels> "
          "channel_map=<channel map> "
          "use_volume_sharing=<yes or no> "
          "force_flat_volume=<yes or no> "
          "fade_in_ms=<ms for fade in process> "
        ));

typedef struct
{
   int fd;
   char path[256];
   unsigned int n_bytes;
   int8_t *data;  //data buffer
   unsigned int saved;
} vsink_dump_t;

struct userdata {
    pa_core *core;
    pa_module *module;

    /* FIXME: Uncomment this and take "autoloaded" as a modarg if this is a filter */
    /* bool autoloaded; */

    pa_sink *sink;
    pa_sink_input *sink_input;

    pa_memblockq *memblockq;

    bool auto_desc;
    unsigned channels;

    /* Called when an audio post processing is requested */
    int (*sink_streams_APP)(pa_mix_info channels[], unsigned nstreams, void *data, size_t length,
                const pa_sample_spec *spec, const pa_cvolume *volume, bool mute, struct userdata *u);

    /*audio processing dynamic library API*/
    void *pDlopen_handle;
    int (*ap_create)(void);
    int (*ap_apply)(void* pPafAudioData);
    int (*ap_msg_write)(int MsgSize, void *payload);
    int (*ap_msg_read)(int Type, void *px_Message_Params);

    pa_native_protocol *protocol;
    int dumping;
    vsink_dump_t prepro_dump[INPUT_SRC_CNT];
    vsink_dump_t postpro_dump[SINK_CNT];
    PAF_AudioFrame paf_prepro; /*store de-interleaved data, and format int32 5Q27*/
    pa_thread *rx_thread;
    int fade_in_frame;
};

static const char* const valid_modargs[] = {
    "sink_name",
    "sink_properties",
    "master",
    "rate",
    "channels",
    "channel_map",
    "use_volume_sharing",
    "force_flat_volume",
    "fade_in_ms",
    NULL
};


#if 0
static void hmdsp_stream_backtrace(void)
{
    int j,nptrs;
#define SIZE 100
    void *buffer[SIZE];
    char **strings;
    nptrs = backtrace(buffer,SIZE);
    printf("backtrace()returned %d addresses\n", nptrs);
    strings = backtrace_symbols(buffer, nptrs);
    if(strings == NULL){
        perror("backtrace_symbols");
        exit(EXIT_FAILURE);
    }
    for(j=0; j<nptrs; j++)
        printf("%s\n",strings[j]);
    free(strings);
}
#endif


static input_audio_config sources[] = {
   // source name                   pos     channels
    { "Media",                      0,      CH_2},
    { "pcmTTS_p",                   2,      CH_1},
    { "AudibleFeedback",            3,      CH_1},
    { "MixPrompt",                  4,      CH_1},
    { "MutexPrompt",                5,      CH_2},
    { "Dirana_Tuner_Chime_SXM_Aux", 7,      CH_8},
    { "Dirana_Tuner",               7,      CH_2},
    { "Dirana_Chime",               9,      CH_2},
    { "Dirana_SXM",                 11,     CH_2},
    { "Dirana_Aux",                 13,     CH_2},
    { "BT_PHONE_DL",                15,     CH_1},
    { "TCP_PHONE_DL",               16,     CH_1},
    { "pcmChime_p",                 17,     CH_1},
    { "RSI_IN1",                    18,     CH_2},
    { "RSI_IN2",                    20,     CH_2},
    { "iAP2",                       22,     CH_2},
};

#define SOURCE_CNT    (sizeof(sources)/sizeof(input_audio_config))

int get_device_config(pa_mix_info info, input_audio_config* cfg)
{
    const char *media_name;
    unsigned int i;

    media_name = pa_proplist_gets(((pa_sink_input *)(info.userdata))->proplist, PA_PROP_MEDIA_NAME);

    if(media_name == NULL) {
        pa_log_error("No available source name!");
        return 0;
    }

    for(i=0; i<SOURCE_CNT; i++){
        if(strstr(media_name, sources[i].name)){
            *cfg = sources[i];
            return i;
        }
    }
    return -1;
}


static void dump_complete_callback(pa_mainloop_api *a, pa_time_event *e,
                    const struct timeval *t, void *userdata)
{
    unsigned int i;
    int32_t ret;

    struct userdata *u = userdata;

    u->dumping = 0;

    pa_log_debug("Time is up, save data to file...");

    for(i=0;i<INPUT_SRC_CNT;i++){
        if(u->prepro_dump[i].data){
            pa_log_debug("prepro%d: %d bytes save to fd %d", i, u->prepro_dump[i].saved, u->prepro_dump[i].fd);
            ret = write(u->prepro_dump[i].fd, u->prepro_dump[i].data, u->prepro_dump[i].saved);
            pa_xfree(u->prepro_dump[i].data);
            u->prepro_dump[i].data = NULL;
        }
        u->prepro_dump[i].saved = 0;
        pa_close(u->prepro_dump[i].fd);
    }

    for(i=0;i<SINK_CNT;i++){
        if(u->postpro_dump[i].data){
            pa_log_debug("postpro%d: %d bytes -> fd %d", i, u->postpro_dump[i].saved, u->postpro_dump[i].fd);
            ret = write(u->postpro_dump[i].fd, u->postpro_dump[i].data, u->postpro_dump[i].saved);
            pa_xfree(u->postpro_dump[i].data);
            u->postpro_dump[i].data = NULL;
        }
        u->postpro_dump[i].saved = 0;
        pa_close(u->postpro_dump[i].fd);
    }

    pa_log_debug("save completed");
}

static pa_dump_error_code_t dump_to_file_start(struct userdata *u, uint32_t seconds, char *dump_path)
{
    int i;
    struct tm *ptm;
    long ts;
    int samples;
    int n_bytes;
    int result;
    char file[256];

    if(u->dumping == 1){
        pa_log_warn("pulseaudio is already in dumping state");
        return PA_DUMP_ERR_BUSY;
    }

    ts = time(NULL);
    ptm = localtime(&ts);


    result = access(dump_path, R_OK|W_OK);
    if (result !=0){
        pa_log_warn("Dump folder doesn't esixt, errorno=%d, %s\n", errno, pa_cstrerror(errno));

        /*if dir does not exist, need create one*/
        if (pa_make_secure_dir(dump_path, 0760, (uid_t) -1, (gid_t) -1, true) < 0){
            pa_log_error("Failed to create dump directory (%s): %s", dump_path, pa_cstrerror(errno));
            return PA_DUMP_ERR_ACCESS;
        }
    }else{
        pa_log_debug("Successed to create dump directory (%s)", dump_path);
    }

    /*pre-calc dump memory*/
    samples = seconds * u->sink->sample_spec.rate;
    for(i=0;i<INPUT_SRC_CNT;i++){

        sprintf(file, "%s/audiodump-%02d-%dch-prepro-%ds.pcm", dump_path, i, sources[i].chs,seconds);
        u->prepro_dump[i].n_bytes = samples * sizeof(int16_t) * sources[i].chs;

	if ((u->prepro_dump[i].fd  = pa_open_cloexec(file, O_CREAT|O_RDWR, 0760)) < 0) {
	    pa_log_error("Failed to create dump file %s: %s", file, pa_cstrerror(errno));
	    return PA_DUMP_ERR_ACCESS;
	}

        pa_log_debug("create dump file %s [fd=%d]", file, u->prepro_dump[i].fd);
        u->prepro_dump[i].saved = 0;
    }

    pa_log_debug("create %d output dump files", SINK_CNT);
    for(i=0;i<SINK_CNT;i++){

        sprintf(file, "%s/audiodump-%02d-%dch-postpro-%ds.pcm", dump_path, i, u->sink->sample_spec.channels, seconds);
        u->postpro_dump[i].n_bytes = samples * sizeof(int16_t) * u->sink->sample_spec.channels;

        if((u->postpro_dump[i].fd = pa_open_cloexec(file, O_CREAT|O_RDWR, 0666)) < 0) {
            pa_log_error("create dump file %s failed", file);
            return PA_DUMP_ERR_ACCESS;
        }

        pa_log_debug("create dump file %s [fd=%d]", file, u->postpro_dump[i].fd);

        u->postpro_dump[i].saved = 0;
    }

    pa_core_rttime_new(u->core, pa_rtclock_now() + seconds*PA_USEC_PER_SEC, dump_complete_callback, u);

    u->dumping = 1;

    return PA_DUMP_OK;
}


static int extension_cb(pa_native_protocol *p, pa_module *m, pa_native_connection *c, uint32_t tag, pa_tagstruct *t)
{
    pa_tagstruct *reply = NULL;
    uint16_t data_index;
    uint16_t ret;
    uint32_t msg_type = MSG_INVALID;
    t_Tagged_MSG_Params msg;
    uint32_t dump_time = 0;
    char *dump_path = NULL;
    struct userdata *u;

    pa_assert(p);
    pa_assert(m);
    pa_assert(c);
    pa_assert(t);

    u = m->userdata;

    pa_tagstruct_getu32(t, &msg_type);

    if(msg_type == MSG_DUMP_CMD){

        pa_log_debug("start dump\n");
        pa_tagstruct_getu32(t, &dump_time);
        pa_tagstruct_gets(t, &dump_path);
        pa_log_debug("dump time %ds dump path %s\n", dump_time, dump_path);
        reply = pa_tagstruct_new(NULL,0);
        pa_tagstruct_putu32(reply, PA_COMMAND_REPLY);
        pa_tagstruct_putu32(reply, tag);
        pa_tagstruct_putu8(reply, dump_to_file_start(u, dump_time, dump_path));

        pa_pstream_send_tagstruct(pa_native_connection_get_pstream(c), reply);
    }else if (msg_type == MSG_AM_CMD){
        if (pa_tagstruct_getu32(t, (uint32_t *)&msg.e_AC_Destination) < 0 ||
            pa_tagstruct_getu32(t, (uint32_t *)&msg.e_Msg_Destination) < 0 ||
            pa_tagstruct_getu32(t, (uint32_t *)&msg.e_Asp_Module) < 0 ||
            pa_tagstruct_getu32(t, (uint32_t *)&msg.x_AM_Params.Session_ID) < 0 ||
            pa_tagstruct_getu32(t, (uint32_t *)&msg.x_AM_Params.Cmd_ID) < 0 ||
            pa_tagstruct_getu32(t, (uint32_t *)&msg.x_AM_Params.Cmd_Type) < 0 ||
            pa_tagstruct_getu32(t, (uint32_t *)&msg.x_AM_Params.InstID) < 0 ||
            pa_tagstruct_getu32(t, (uint32_t *)&msg.x_AM_Params.Length) < 0) {
                goto fail;
        }

        pa_log_debug("e_AC_Destination    : 0x%x", msg.e_AC_Destination);
        pa_log_debug("e_Msg_Destination   : 0x%x", msg.e_Msg_Destination);
        pa_log_debug("e_Asp_Module        : 0x%x", msg.e_Asp_Module);

        pa_log_debug("SessionID : 0x%x", msg.x_AM_Params.Session_ID);
        pa_log_debug("CmdID     : 0x%x", msg.x_AM_Params.Cmd_ID);
        pa_log_debug("CmdType   : 0x%x", msg.x_AM_Params.Cmd_Type);
        pa_log_debug("InstID    : 0x%x", msg.x_AM_Params.InstID);
        pa_log_debug("Length    : 0x%x", msg.x_AM_Params.Length);

        for(data_index=0; data_index<msg.x_AM_Params.Length; data_index++){
            pa_tagstruct_getu8(t, &msg.x_AM_Params.Data[data_index]);
            pa_log_debug("Data[%d]    : 0x%x", data_index, msg.x_AM_Params.Data[data_index]);
        }

        /*send cmd to DSP*/
        ret = u->ap_msg_write(msg.x_AM_Params.Length, &msg);
        reply = pa_tagstruct_new(NULL,0);
        pa_tagstruct_putu32(reply, PA_COMMAND_REPLY);
        pa_tagstruct_putu32(reply, tag);
        pa_tagstruct_putu8(reply, 0x1);

        pa_log_debug("send command status %d back",ret);
        pa_pstream_send_tagstruct(pa_native_connection_get_pstream(c), reply);
    }else{
        pa_log_error("unknow msg type");
    }
    return 0;

fail:
    pa_log_error("%s,%s,%d\n",__FILE__,__func__,__LINE__);
    if (reply)
        pa_tagstruct_free(reply);

    return -1;
}

typedef enum pcm_fade_state
{
   OMAP_PCM_FADED_OUT = 0,
   OMAP_PCM_FADED_IN = 1
} pcm_fade_state_e;

/* support 16 bit. Support fade in/out */
static inline int32_t pa_sink_input_fade(void *dst, void*src, short frames, int fade_frame_total,
                                            short channels, int step, int frame_proced, pcm_fade_state_e fade)
{
    int i = 0;
    int front_left, front_right,rear_left,rear_right;
    short factor = 65536/fade_frame_total;
    short *pSRC = (short *) src;
    short *pDST = (short *) dst;
    short frames_tobe_proc;

    pa_assert(fade_frame_total <= 65536);

    if(frame_proced >= fade_frame_total){
        step = 0;
        frame_proced = -1;
        return -1;
    } else if(frame_proced+frames > fade_frame_total){
        frames_tobe_proc = fade_frame_total - frame_proced;
    } else
        frames_tobe_proc = frames;

    /* -------------Two channels case was not verified-------------*/
    if(channels == 2){
        /* For fade out, we retransmit the last 256 samples valid frames and the remaining part use silence data.
           for fade in, we just modify the volume of the frames, ramp up the whole current frames */
        if(fade == OMAP_PCM_FADED_OUT){
            for(i=0; i<256; i++){
                front_left  = pSRC[i*2  ] * (65536-step);
                front_right = pSRC[i*2+1] * (65536-step);
                pDST[i]     = ((front_left & 0xFFFF0000) | (front_right >> 16));
                step +=factor;
            }
        }else {
            for(i=0; i<256; i++)    {
                front_left  = pSRC[i*2  ] * step;
                front_right = pSRC[i*2+1] * step;
                pDST[i]     = ((front_left & 0xFFFF0000) | (front_right >> 16));
                step +=factor;
            }
        }
    }

    /* -------------Four channels fade out case was not verified-------------*/
    else if(channels == 4){
        if(fade == OMAP_PCM_FADED_OUT){
            for(i=0; i<256; i++){
                pDST[i*4+0] = (pSRC[i*4+0] * (65536-step))>>16;
                pDST[i*4+1] = (pSRC[i*4+1] * (65536-step))>>16;
                pDST[i*4+2] = (pSRC[i*4+2] * (65536-step))>>16;
                pDST[i*4+3] = (pSRC[i*4+3] * (65536-step))>>16;

                step +=factor;
            }
        }else {
            for(i=0; i<frames_tobe_proc; i++){
                pDST[i*4+0] = (pSRC[i*4+0] * step)>>16;
                pDST[i*4+1] = (pSRC[i*4+1] * step)>>16;
                pDST[i*4+2] = (pSRC[i*4+2] * step)>>16;
                pDST[i*4+3] = (pSRC[i*4+3] * step)>>16;

                step +=factor;
            }
            frame_proced += frames;
        }
    }

    return 0;
}

/* Called from thread context */
static void pa_sink_input_peek_module(pa_sink_input *i, size_t slength /* in sink bytes */,
                                                 pa_mix_info *info, struct userdata *u) {
    bool do_volume_adj_here, need_volume_factor_sink;
    bool volume_is_norm;
    size_t block_size_max_sink, block_size_max_sink_input;
    size_t ilength;
    size_t ilength_full;
    pa_memchunk *chunk;
    pa_cvolume *volume;

    pa_sink_input_assert_ref(i);
    pa_sink_input_assert_io_context(i);
    pa_assert(PA_SINK_INPUT_IS_LINKED(i->thread_info.state));
    pa_assert(pa_frame_aligned(slength, &i->sink->sample_spec));
    pa_assert(chunk = &info->chunk);
    pa_assert(volume = &info->volume);
    pa_assert(info);
    pa_assert(u);

#ifdef SINK_INPUT_DEBUG
    pa_log_debug("peek ");
#endif

    block_size_max_sink_input = i->thread_info.resampler ?
        pa_resampler_max_block_size(i->thread_info.resampler) :
        pa_frame_align(pa_mempool_block_size_max(i->core->mempool), &i->sample_spec);

    block_size_max_sink = pa_frame_align(pa_mempool_block_size_max(i->core->mempool), &i->sink->sample_spec);

    /* Default buffer size */
    if (slength <= 0)
        slength = pa_frame_align(CONVERT_BUFFER_LENGTH, &i->sink->sample_spec);

    if (slength > block_size_max_sink)
        slength = block_size_max_sink;

    if (i->thread_info.resampler) {
        ilength = pa_resampler_request(i->thread_info.resampler, slength);

        if (ilength <= 0)
            ilength = pa_frame_align(CONVERT_BUFFER_LENGTH, &i->sample_spec);
    } else
        ilength = slength;

    /* Length corresponding to slength (without limiting to
     * block_size_max_sink_input). */
    ilength_full = ilength;

    if (ilength > block_size_max_sink_input)
        ilength = block_size_max_sink_input;

    /* If the channel maps of the sink and this stream differ, we need
     * to adjust the volume *before* we resample. Otherwise we can do
     * it after and leave it for the sink code */

    do_volume_adj_here = !pa_channel_map_equal(&i->channel_map, &i->sink->channel_map);
    volume_is_norm = pa_cvolume_is_norm(&i->thread_info.soft_volume) && !i->thread_info.muted;
    need_volume_factor_sink = !pa_cvolume_is_norm(&i->volume_factor_sink);

    while (!pa_memblockq_is_readable(i->thread_info.render_memblockq) ||
           (pa_memblockq_is_readable(i->thread_info.render_memblockq) && pa_memblockq_get_length(i->thread_info.render_memblockq)<slength))
    {
        pa_memchunk tchunk;
        int pop_result;

        /* There's nothing in our render queue. We need to fill it up
         * with data from the implementor. */

        if (i->thread_info.state == PA_SINK_INPUT_CORKED ||
            (pop_result = i->pop(i, ilength, &tchunk)) < 0) {

            /* OK, we're corked or the implementor didn't give us any
             * data, so let's just hand out silence */
            pa_atomic_store(&i->thread_info.drained, 1);

            pa_memblockq_seek(i->thread_info.render_memblockq, (int64_t) slength, PA_SEEK_RELATIVE, true);
            i->thread_info.playing_for = 0;
            if (i->thread_info.underrun_for != (uint64_t) -1) {
                i->thread_info.underrun_for += ilength_full;
                i->thread_info.underrun_for_sink += slength;
            }
            break;
        }

        pa_atomic_store(&i->thread_info.drained, 0);

        pa_assert(tchunk.length > 0);
        pa_assert(tchunk.memblock);

        i->thread_info.underrun_for = 0;
        i->thread_info.underrun_for_sink = 0;
        i->thread_info.playing_for += tchunk.length;

        while (tchunk.length > 0) {
            pa_memchunk wchunk;
            bool nvfs = need_volume_factor_sink;

            wchunk = tchunk;
            pa_memblock_ref(wchunk.memblock);

            if (wchunk.length > block_size_max_sink_input)
                wchunk.length = block_size_max_sink_input;

            /* It might be necessary to adjust the volume here */
            if (do_volume_adj_here && !volume_is_norm) {
                pa_memchunk_make_writable(&wchunk, 0);

                if (i->thread_info.muted) {
                    pa_silence_memchunk(&wchunk, &i->thread_info.sample_spec);
                    nvfs = false;

                } else if (!i->thread_info.resampler && nvfs) {
                    pa_cvolume v;

                    /* If we don't need a resampler we can merge the
                     * post and the pre volume adjustment into one */

                    pa_sw_cvolume_multiply(&v, &i->thread_info.soft_volume, &i->volume_factor_sink);
                    pa_volume_memchunk(&wchunk, &i->thread_info.sample_spec, &v);
                    nvfs = false;

                } else
                    pa_volume_memchunk(&wchunk, &i->thread_info.sample_spec, &i->thread_info.soft_volume);
            }

            if (!i->thread_info.resampler) {

                if (nvfs) {
                    pa_memchunk_make_writable(&wchunk, 0);
                    pa_volume_memchunk(&wchunk, &i->sink->sample_spec, &i->volume_factor_sink);
                }

                pa_memblockq_push_align(i->thread_info.render_memblockq, &wchunk);
            } else {
                pa_memchunk rchunk;
                pa_resampler_run(i->thread_info.resampler, &wchunk, &rchunk);

#ifdef SINK_INPUT_DEBUG
                pa_log_debug("pushing %lu", (unsigned long) rchunk.length);
#endif

                if (rchunk.memblock) {

                    if (nvfs) {
                        pa_memchunk_make_writable(&rchunk, 0);
                        pa_volume_memchunk(&rchunk, &i->sink->sample_spec, &i->volume_factor_sink);
                    }

                    pa_memblockq_push_align(i->thread_info.render_memblockq, &rchunk);
                    pa_memblock_unref(rchunk.memblock);
                }
            }

            pa_memblock_unref(wchunk.memblock);

            tchunk.index += wchunk.length;
            tchunk.length -= wchunk.length;
        }

        pa_memblock_unref(tchunk.memblock);
    }

    pa_assert_se(pa_memblockq_peek_fixed_size(i->thread_info.render_memblockq, slength, chunk) >= 0);

    pa_assert(chunk->length > 0);
    pa_assert(chunk->memblock);

#if 0
    {
        void *frame_current = pa_memblock_acquire_chunk(chunk);

         if(i->thread_info.frame_proced >= 0){
            int16_t frames = chunk->length/(u->channels*sizeof(int16_t));
            pa_sink_input_fade(frame_current, frame_current, frames, u->fade_in_frame,
                                u->channels, i->thread_info.step, i->thread_info.frame_proced, OMAP_PCM_FADED_IN);
        }
        pa_memblock_release(chunk->memblock);
     }
#endif

#ifdef SINK_INPUT_DEBUG
    pa_log_debug("peeking %lu", (unsigned long) chunk->length);
#endif

    if (chunk->length > block_size_max_sink)
        chunk->length = block_size_max_sink;

    /* Let's see if we had to apply the volume adjustment ourselves,
     * or if this can be done by the sink for us */

    if (do_volume_adj_here)
        /* We had different channel maps, so we already did the adjustment */
        pa_cvolume_reset(volume, i->sink->sample_spec.channels);
    else if (i->thread_info.muted)
        /* We've both the same channel map, so let's have the sink do the adjustment for us*/
        pa_cvolume_mute(volume, i->sink->sample_spec.channels);
    else
        *volume = i->thread_info.soft_volume;
}


/* Called from IO thread context */
static unsigned fill_mix_info_module(pa_sink *s, size_t *length, pa_mix_info *info, unsigned maxinfo, struct userdata *u) {
    pa_sink_input *i;
    unsigned n = 0;
    void *state = NULL;
    size_t mixlength = *length;

    pa_sink_assert_ref(s);
    pa_sink_assert_io_context(s);
    pa_assert(info);

    while ((i = pa_hashmap_iterate(s->thread_info.inputs, &state, NULL)) && maxinfo > 0) {
        pa_sink_input_assert_ref(i);

        pa_sink_input_peek_module(i, *length, info, u);

        if (mixlength == 0 || info->chunk.length < mixlength)
            mixlength = info->chunk.length;

      if (pa_memblock_is_silence(info->chunk.memblock)) {
         void *ptr = pa_memblock_acquire_chunk(&info->chunk);
         memset(ptr, 0, *length);
         pa_memblock_release(info->chunk.memblock);
        }

        info->userdata = pa_sink_input_ref(i);

        pa_assert(info->chunk.memblock);
        pa_assert(info->chunk.length > 0);

        info++;
        n++;
        maxinfo--;
    }

    if (mixlength > 0)
        *length = mixlength;

    return n;
}

/* Called from IO thread context */
static void inputs_drop(pa_sink *s, pa_mix_info *info, unsigned n, pa_memchunk *result) {
    pa_sink_input *i;
    void *state;
    unsigned p = 0;
    unsigned n_unreffed = 0;

    pa_sink_assert_ref(s);
    pa_sink_assert_io_context(s);
    pa_assert(result);
    pa_assert(result->memblock);
    pa_assert(result->length > 0);

    /* We optimize for the case where the order of the inputs has not changed */

    PA_HASHMAP_FOREACH(i, s->thread_info.inputs, state) {
        unsigned j;
        pa_mix_info* m = NULL;

        pa_sink_input_assert_ref(i);

        /* Let's try to find the matching entry info the pa_mix_info array */
        for (j = 0; j < n; j ++) {

            if (info[p].userdata == i) {
                m = info + p;
                break;
            }

            p++;
            if (p >= n)
                p = 0;
        }

        /* Drop read data */
        pa_sink_input_drop(i, result->length);

        if (s->monitor_source && PA_SOURCE_IS_LINKED(s->monitor_source->thread_info.state)) {

            if (pa_hashmap_size(i->thread_info.direct_outputs) > 0) {
                void *ostate = NULL;
                pa_source_output *o;
                pa_memchunk c;

                if (m && m->chunk.memblock) {
                    c = m->chunk;
                    pa_memblock_ref(c.memblock);
                    pa_assert(result->length <= c.length);
                    c.length = result->length;

                    pa_memchunk_make_writable(&c, 0);
                    pa_volume_memchunk(&c, &s->sample_spec, &m->volume);
                } else {
                    c = s->silence;
                    pa_memblock_ref(c.memblock);
                    pa_assert(result->length <= c.length);
                    c.length = result->length;
                }

                while ((o = pa_hashmap_iterate(i->thread_info.direct_outputs, &ostate, NULL))) {
                    pa_source_output_assert_ref(o);
                    pa_assert(o->direct_on_input == i);
                    pa_source_post_direct(s->monitor_source, o, &c);
                }

                pa_memblock_unref(c.memblock);
            }
        }

        if (m) {
            if (m->chunk.memblock) {
                pa_memblock_unref(m->chunk.memblock);
                pa_memchunk_reset(&m->chunk);
            }

            pa_sink_input_unref(m->userdata);
            m->userdata = NULL;

            n_unreffed += 1;
        }
    }

    /* Now drop references to entries that are included in the
     * pa_mix_info array but don't exist anymore */

    if (n_unreffed < n) {
        for (; n > 0; info++, n--) {
            if (info->userdata)
                pa_sink_input_unref(info->userdata);
            if (info->chunk.memblock)
                pa_memblock_unref(info->chunk.memblock);
        }
    }

    if (s->monitor_source && PA_SOURCE_IS_LINKED(s->monitor_source->thread_info.state))
        pa_source_post(s->monitor_source, result);
}


/*
    convert_to_processing_work_format
    1 channels remap
    2 deinterleave
    3 16bit ->32bit
    4 1Q31 -> 5Q27
    5 put data to the right place
*/
static void paf_install(const void *src, void *dst[],
                        unsigned src_channels, unsigned dst_channels, unsigned n)
{
    unsigned c;

    pa_assert(src);
    pa_assert(dst);
    pa_assert(src_channels > 0);
    pa_assert(dst_channels > 0);
    pa_assert(n > 0);

    for (c = 0; c < dst_channels; c++) {
        unsigned j;
        const int16_t *s;
        int32_t *d;

        s = (int16_t *) src + c;
        d = (int32_t *)dst[c];

        for (j = 0; j < n; j ++) {
            /* if stream opened by multi-clients, simple mix here */
            *d += (int32_t)(((int32_t) *(int16_t *)s) << 12);

            s = (int16_t*) s + src_channels;
            d = (int32_t*) d + 1;
        }
    }
}


static void paf_extract(const void *src[], unsigned channels, void *dst, unsigned n)
{

    unsigned c;

    pa_assert(src);
    pa_assert(dst);
    pa_assert(channels > 0);
    pa_assert(n > 0);

    for (c = 0; c < channels; c++) {
        unsigned j;
        const int32_t *s;
        int16_t *d;

        s = (int32_t *) src[c];
        d = (int16_t *) dst + c;

        for (j = 0; j < n; j ++) {
            *d = (int16_t)(*s >> 12);

            s = (int32_t*) s + 1;
            d = (int16_t*) d + channels;
        }

    }
}

static int app_mix_count=0;

static void APP_mix(pa_mix_info streams[], unsigned int nstreams, unsigned channels, int16_t *data, unsigned length, struct userdata *u) {
    unsigned int i,j;
    unsigned int samples, frames;
    input_audio_config cfg;
    PAF_AudioData *p_data[INPUT_CH_CNT];

    samples = length/sizeof(int16_t)/channels;
    frames = samples/FRAME_SIZE;
    memset(&cfg, 0, sizeof(input_audio_config));

    if(samples%FRAME_SIZE != 0){
        pa_log_warn("warning: got an abnormal sample [%d]", samples);
    }

    /*clear buffer is necessary*/
    for(i=0; i<INPUT_CH_CNT; i++){
        memset(u->paf_prepro.data.sample[i],0,samples*sizeof(int32_t)*1);
    }

    for(i=0; i<nstreams; i++) {
        if(get_device_config(streams[i], &cfg) >= 0){
            /*install data to processing PAF struct */
            paf_install(streams[i].ptr, (void *)&(u->paf_prepro.data.sample[cfg.pos]), channels, cfg.chs, samples);
        }
    }

    for(i=0; i<INPUT_CH_CNT; i++){
        p_data[i] = u->paf_prepro.data.sample[i];
    }

    /*processing each 1 frame=64 samples*/
    for(i=0;i<frames;i++){
        for(j=0; j<INPUT_CH_CNT; j++){
            u->paf_prepro.data.sample[j] = (int32_t *)(p_data[j]) + FRAME_SIZE*i;
        }

        /*do processing*/
        u->ap_apply(&(u->paf_prepro));

        /*cpoy processed data out*/
        paf_extract((void *)&(u->paf_prepro.data.sample[0]), channels, data+FRAME_SIZE*i*channels, FRAME_SIZE);
    }

    for(i=0; i<INPUT_CH_CNT; i++){
        u->paf_prepro.data.sample[i] = p_data[i];
    }

    /* Check whether need to be dumped */
    if(u->dumping){
        /* bankup audio stream for preprocess, before crossbar */
        for(i=0; i<nstreams; i++) {
            int32_t source_index = get_device_config(streams[i], &cfg);
            if(source_index < 0)
                continue;

            pa_log_warn("source_index = %d, channels = %d", source_index, cfg.chs);
            /*check memory whether has malloced*/
            if(u->prepro_dump[source_index].data == NULL)
                u->prepro_dump[source_index].data = pa_xmalloc0(u->prepro_dump[source_index].n_bytes);

            if(u->prepro_dump[source_index].saved + samples*sizeof(int16_t)*cfg.chs <= u->prepro_dump[source_index].n_bytes){
                int16_t *dst = (int16_t*)(u->prepro_dump[source_index].data + u->prepro_dump[source_index].saved);
                int16_t *src = (int16_t*)(streams[i].ptr);
                size_t i;

                if(cfg.chs == CH_1){
                    for(i=0; i<samples; i++){
                        dst[i] = src[i*u->sink->sample_spec.channels];
                    }
                    u->prepro_dump[source_index].saved += samples*sizeof(int16_t);
                }else if(cfg.chs == CH_2){
                    for(i=0; i<samples; i++){
                        dst[i*CH_2+0] = src[i*u->sink->sample_spec.channels+0];
                        dst[i*CH_2+1] = src[i*u->sink->sample_spec.channels+1];
                    }
                    u->prepro_dump[source_index].saved += samples*sizeof(int16_t)*CH_2;
                }else if(cfg.chs == CH_8){
                    memcpy(dst, src, samples*sizeof(int16_t)*u->sink->sample_spec.channels);
                    u->prepro_dump[source_index].saved += samples*sizeof(int16_t)*CH_8;
                }
            }
        }
        /* bankup audio stream for postprocess, after crossbar */
        for(i=0; i<SINK_CNT; i++) {

            /*check memory whether has malloced*/
            if(u->postpro_dump[i].data == NULL)
                u->postpro_dump[i].data = pa_xmalloc0(u->postpro_dump[i].n_bytes);

            if(u->postpro_dump[i].saved + samples*sizeof(int16_t)*u->sink->sample_spec.channels <= u->postpro_dump[i].n_bytes) {
                int8_t *dst = (int8_t*)(u->postpro_dump[i].data + u->postpro_dump[i].saved);
                int8_t *src = (int8_t*)(data) + i;

                memcpy(dst, src, samples*sizeof(int16_t)*u->sink->sample_spec.channels);

                u->postpro_dump[i].saved += samples*sizeof(int16_t)*u->sink->sample_spec.channels;
            }
        }
    }
}

static int sink_streams_APP(
        pa_mix_info streams[],
        unsigned nstreams,
        void *data,
        size_t length,
        const pa_sample_spec *spec,
        const pa_cvolume *volume,
        bool mute,
        struct userdata *u) {

    pa_cvolume full_volume;
    unsigned k;
    char st[128];

    pa_assert(streams);
    pa_assert(data);
    pa_assert(length);
    pa_assert(spec);

    if (!volume)
        volume = pa_cvolume_reset(&full_volume, spec->channels);

    if (mute || pa_cvolume_is_muted(volume)) {
        pa_silence_memory(data, length, spec);
        return length;
    }

    for (k = 0; k < nstreams; k++) {
        pa_assert(length <= streams[k].chunk.length);
        streams[k].ptr = pa_memblock_acquire_chunk(&streams[k].chunk);
    }

    APP_mix(streams, nstreams, spec->channels, data, length, u);

    for (k = 0; k < nstreams; k++)
        pa_memblock_release(streams[k].chunk.memblock);

    return length;
}


/* Called from I/O thread context */
static int sink_process_msg_cb(pa_msgobject *o, int code, void *data, int64_t offset, pa_memchunk *chunk) {
    struct userdata *u = PA_SINK(o)->userdata;

    switch (code) {

        case PA_SINK_MESSAGE_GET_LATENCY:

            /* The sink is _put() before the sink input is, so let's
             * make sure we don't access it in that time. Also, the
             * sink input is first shut down, the sink second. */
            if (!PA_SINK_IS_LINKED(u->sink->thread_info.state) ||
                !PA_SINK_INPUT_IS_LINKED(u->sink_input->thread_info.state)) {
                *((pa_usec_t*) data) = 0;
                return 0;
            }

            *((pa_usec_t*) data) =

                /* Get the latency of the master sink */
                pa_sink_get_latency_within_thread(u->sink_input->sink) +

                /* Add the latency internal to our sink input on top */
                pa_bytes_to_usec(pa_memblockq_get_length(u->sink_input->thread_info.render_memblockq), &u->sink_input->sink->sample_spec);

            return 0;
    }

    return pa_sink_process_msg(o, code, data, offset, chunk);
}

/* Called from main context */
static int sink_set_state_cb(pa_sink *s, pa_sink_state_t state) {
    struct userdata *u;

    pa_sink_assert_ref(s);
    pa_assert_se(u = s->userdata);

    if (!PA_SINK_IS_LINKED(state) ||
        !PA_SINK_INPUT_IS_LINKED(pa_sink_input_get_state(u->sink_input)))
        return 0;

    pa_sink_input_cork(u->sink_input, state == PA_SINK_SUSPENDED);
    return 0;
}

/* Called from I/O thread context */
static void sink_request_rewind_cb(pa_sink *s) {
    struct userdata *u;

    pa_sink_assert_ref(s);
    pa_assert_se(u = s->userdata);

    if (!PA_SINK_IS_LINKED(u->sink->thread_info.state) ||
        !PA_SINK_INPUT_IS_LINKED(u->sink_input->thread_info.state))
        return;

    /* Just hand this one over to the master sink */
    pa_sink_input_request_rewind(u->sink_input,
                                 s->thread_info.rewind_nbytes +
                                 pa_memblockq_get_length(u->memblockq), true, false, false);
}

/* Called from I/O thread context */
static void sink_update_requested_latency_cb(pa_sink *s) {
    struct userdata *u;

    pa_sink_assert_ref(s);
    pa_assert_se(u = s->userdata);

    if (!PA_SINK_IS_LINKED(u->sink->thread_info.state) ||
        !PA_SINK_INPUT_IS_LINKED(u->sink_input->thread_info.state))
        return;

    /* Just hand this one over to the master sink */
    pa_sink_input_set_requested_latency_within_thread(
            u->sink_input,
            pa_sink_get_requested_latency_within_thread(s));
}

/* Called from main context */
static void sink_set_volume_cb(pa_sink *s) {
    struct userdata *u;

    pa_sink_assert_ref(s);
    pa_assert_se(u = s->userdata);

    if (!PA_SINK_IS_LINKED(pa_sink_get_state(s)) ||
        !PA_SINK_INPUT_IS_LINKED(pa_sink_input_get_state(u->sink_input)))
        return;

    pa_sink_input_set_volume(u->sink_input, &s->real_volume, s->save_volume, true);
}

/* Called from main context */
static void sink_set_mute_cb(pa_sink *s) {
    struct userdata *u;

    pa_sink_assert_ref(s);
    pa_assert_se(u = s->userdata);

    if (!PA_SINK_IS_LINKED(pa_sink_get_state(s)) ||
        !PA_SINK_INPUT_IS_LINKED(pa_sink_input_get_state(u->sink_input)))
        return;

    pa_sink_input_set_mute(u->sink_input, s->muted, s->save_muted);
}

static void thread_func_event_rx(void *userdata) {
    struct userdata *u = userdata;
    pa_client *client;
    pa_core *c;
    uint32_t idx;
    int i;
    pa_proplist *data = NULL;
    t_Tagged_MSG_Params msg;

    pa_assert(u);

    pa_log_debug("EVENT RX Thread starting up");

    c = u->core;
    idx = PA_IDXSET_INVALID;

    for (;;) {
        pa_log_debug("Loop");
        u->ap_msg_read(0,&msg);
        pa_log_debug("read a event");
        pa_log_debug("AC_Dest   : %d", msg.e_AC_Destination);
        pa_log_debug("Msg_Dest  : 0x%x", msg.e_Msg_Destination);
        pa_log_debug("Asp       : 0x%x", msg.e_Asp_Module);
        pa_log_debug("SessionID : 0x%x", msg.x_AM_Params.Session_ID);
        pa_log_debug("CmdID     : 0x%x", msg.x_AM_Params.Cmd_ID);
        pa_log_debug("CmdType   : 0x%x", msg.x_AM_Params.Cmd_Type);
        pa_log_debug("InstID    : 0x%x", msg.x_AM_Params.InstID);
        pa_log_debug("Length    : 0x%x", msg.x_AM_Params.Length);
        for(i=0; i<msg.x_AM_Params.Length; i++){
            pa_log_debug("Data[%d]   : 0x%x", i, msg.x_AM_Params.Data[i]);
        }

        data = pa_proplist_new();

        //pa_log("clients: %s", pa_client_list_to_string(u->core));
        PA_IDXSET_FOREACH(client, c->clients, idx) {
            //pa_log("APPLICATION_ID: %s", pa_proplist_gets(client->proplist, PA_PROP_APPLICATION_NAME));
            char *temp_prop_audio_manager = pa_proplist_gets(client->proplist, PA_PROP_AUDIO_MANAGER);
            pa_assert(temp_prop_audio_manager);
            if((pa_proplist_contains(client->proplist, PA_PROP_AUDIO_MANAGER)) &&
                    !strcmp(temp_prop_audio_manager, "true")) {
                pa_log_debug("sending event to %s",pa_proplist_gets(client->proplist, PA_PROP_APPLICATION_NAME));
                pa_proplist_set(data, PA_PROP_DSP_EVENT, &msg, sizeof(msg)- sizeof(msg.x_AM_Params.Data) + msg.x_AM_Params.Length);
                pa_client_send_event(client,DSP_EVENT,data);
            }
        }
        pa_proplist_free(data);
    }

    pa_log_debug("Thread event rx shutting down");
}

static void pa_app_sink_render(struct userdata *u, size_t length, pa_memchunk *result) {
    pa_sink*s = u->sink;
    pa_mix_info info[MAX_MIX_CHANNELS];
    unsigned n;
    size_t block_size_max;

    pa_sink_assert_ref(s);
    pa_sink_assert_io_context(s);
    pa_assert(PA_SINK_IS_LINKED(s->thread_info.state));
    pa_assert(pa_frame_aligned(length, &s->sample_spec));
    pa_assert(result);

    pa_assert(!s->thread_info.rewind_requested);
    pa_assert(s->thread_info.rewind_nbytes == 0);

    if (s->thread_info.state == PA_SINK_SUSPENDED) {
        result->memblock = pa_memblock_ref(s->silence.memblock);
        result->index = s->silence.index;
        result->length = PA_MIN(s->silence.length, length);
        return;
    }

    pa_sink_ref(s);

    if (length <= 0)
        length = pa_frame_align(MIX_BUFFER_LENGTH, &s->sample_spec);

    block_size_max = pa_mempool_block_size_max(s->core->mempool);
    if (length > block_size_max)
        length = pa_frame_align(block_size_max, &s->sample_spec);

    pa_assert(length > 0);

    n = fill_mix_info_module(s, &length, info, MAX_MIX_CHANNELS, u);

    if (n == 0) {
        *result = s->silence;
        pa_memblock_ref(result->memblock);

        if (result->length > length)
            result->length = length;

    } else {
        void *ptr;

        result->memblock = pa_memblock_new(s->core->mempool, length);

        ptr = pa_memblock_acquire(result->memblock);

        if(u->sink_streams_APP)
        {
            result->length = u->sink_streams_APP(info, n,
                            ptr, length,
                            &u->sink->sample_spec,
                            &u->sink->thread_info.soft_volume,
                            u->sink->thread_info.soft_muted,
                            u);
        }

        pa_memblock_release(result->memblock);

        result->index = 0;
    }

    inputs_drop(s, info, n, result);

    pa_sink_unref(s);
}

/* Called from I/O thread context */
static int sink_input_pop_cb(pa_sink_input *i, size_t nbytes, pa_memchunk *chunk) {
    struct userdata *u;
    pa_memchunk nchunk;

    pa_sink_input_assert_ref(i);
    pa_assert(chunk);
    pa_assert_se(u = i->userdata);

    /* Hmm, process any rewind request that might be queued up */
    pa_sink_process_rewind(u->sink, 0);

    /* (1) IF YOU NEED A FIXED BLOCK SIZE USE
     * pa_memblockq_peek_fixed_size() HERE INSTEAD. NOTE THAT FILTERS
     * WHICH CAN DEAL WITH DYNAMIC BLOCK SIZES ARE HIGHLY
     * PREFERRED. */
    while (pa_memblockq_peek(u->memblockq, chunk) < 0) {
        pa_app_sink_render(u, nbytes, &nchunk);
        pa_memblockq_push(u->memblockq, &nchunk);
        pa_memblock_unref(nchunk.memblock);
    }

    pa_memblockq_drop(u->memblockq, chunk->length);

    return 0;
}

/* Called from I/O thread context */
static void sink_input_process_rewind_cb(pa_sink_input *i, size_t nbytes) {
    struct userdata *u;
    size_t amount = 0;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    if (u->sink->thread_info.rewind_nbytes > 0) {
        size_t max_rewrite;

        max_rewrite = nbytes + pa_memblockq_get_length(u->memblockq);
        amount = PA_MIN(u->sink->thread_info.rewind_nbytes, max_rewrite);
        u->sink->thread_info.rewind_nbytes = 0;

        if (amount > 0) {
            pa_memblockq_seek(u->memblockq, - (int64_t) amount, PA_SEEK_RELATIVE, true);

            /* (5) PUT YOUR CODE HERE TO RESET YOUR FILTER  */
        }
    }

    pa_sink_process_rewind(u->sink, amount);
    pa_memblockq_rewind(u->memblockq, nbytes);
}

/* Called from I/O thread context */
static void sink_input_update_max_rewind_cb(pa_sink_input *i, size_t nbytes) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    /* FIXME: Too small max_rewind:
     * https://bugs.freedesktop.org/show_bug.cgi?id=53709 */
    pa_memblockq_set_maxrewind(u->memblockq, nbytes);
    pa_sink_set_max_rewind_within_thread(u->sink, nbytes);
}

/* Called from I/O thread context */
static void sink_input_update_max_request_cb(pa_sink_input *i, size_t nbytes) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    /* (6) IF YOU NEED A FIXED BLOCK SIZE ROUND nbytes UP TO MULTIPLES
     * OF IT HERE. THE PA_ROUND_UP MACRO IS USEFUL FOR THAT. */

    pa_sink_set_max_request_within_thread(u->sink, nbytes);
}

/* Called from I/O thread context */
static void sink_input_update_sink_latency_range_cb(pa_sink_input *i) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    pa_sink_set_latency_range_within_thread(u->sink, i->sink->thread_info.min_latency, i->sink->thread_info.max_latency);
}

/* Called from I/O thread context */
static void sink_input_update_sink_fixed_latency_cb(pa_sink_input *i) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    /* (7) IF YOU NEED A FIXED BLOCK SIZE ADD THE LATENCY FOR ONE
     * BLOCK MINUS ONE SAMPLE HERE. pa_usec_to_bytes_round_up() IS
     * USEFUL FOR THAT. */

    pa_sink_set_fixed_latency_within_thread(u->sink, i->sink->thread_info.fixed_latency);
}

/* Called from I/O thread context */
static void sink_input_detach_cb(pa_sink_input *i) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    pa_sink_detach_within_thread(u->sink);

    pa_sink_set_rtpoll(u->sink, NULL);
}

/* Called from I/O thread context */
static void sink_input_attach_cb(pa_sink_input *i) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    pa_sink_set_rtpoll(u->sink, i->sink->thread_info.rtpoll);
    pa_sink_set_latency_range_within_thread(u->sink, i->sink->thread_info.min_latency, i->sink->thread_info.max_latency);

    /* (8.1) IF YOU NEED A FIXED BLOCK SIZE ADD THE LATENCY FOR ONE
     * BLOCK MINUS ONE SAMPLE HERE. SEE (7) */
    pa_sink_set_fixed_latency_within_thread(u->sink, i->sink->thread_info.fixed_latency);

    /* (8.2) IF YOU NEED A FIXED BLOCK SIZE ROUND
     * pa_sink_input_get_max_request(i) UP TO MULTIPLES OF IT
     * HERE. SEE (6) */
    pa_sink_set_max_request_within_thread(u->sink, pa_sink_input_get_max_request(i));

    /* FIXME: Too small max_rewind:
     * https://bugs.freedesktop.org/show_bug.cgi?id=53709 */
    pa_sink_set_max_rewind_within_thread(u->sink, pa_sink_input_get_max_rewind(i));

    pa_sink_attach_within_thread(u->sink);
}

/* Called from main context */
static void sink_input_kill_cb(pa_sink_input *i) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    /* The order here matters! We first kill the sink input, followed
     * by the sink. That means the sink callbacks must be protected
     * against an unconnected sink input! */
    pa_sink_input_unlink(u->sink_input);
    pa_sink_unlink(u->sink);

    pa_sink_input_unref(u->sink_input);
    u->sink_input = NULL;

    pa_sink_unref(u->sink);
    u->sink = NULL;

    pa_module_unload_request(u->module, true);
}

/* Called from IO thread context */
static void sink_input_state_change_cb(pa_sink_input *i, pa_sink_input_state_t state) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    /* If we are added for the first time, ask for a rewinding so that
     * we are heard right-away. */
    if (PA_SINK_INPUT_IS_LINKED(state) &&
        i->thread_info.state == PA_SINK_INPUT_INIT) {
        pa_log_debug("Requesting rewind due to state change.");
        pa_sink_input_request_rewind(i, 0, false, true, true);
    }
}

/* Called from main context */
static void sink_input_moving_cb(pa_sink_input *i, pa_sink *dest) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    if (dest) {
        pa_sink_set_asyncmsgq(u->sink, dest->asyncmsgq);
        pa_sink_update_flags(u->sink, PA_SINK_LATENCY|PA_SINK_DYNAMIC_LATENCY, dest->flags);
    } else
        pa_sink_set_asyncmsgq(u->sink, NULL);

    if (u->auto_desc && dest) {
        const char *z;
        pa_proplist *pl;

        pl = pa_proplist_new();
        z = pa_proplist_gets(dest->proplist, PA_PROP_DEVICE_DESCRIPTION);
        pa_proplist_setf(pl, PA_PROP_DEVICE_DESCRIPTION, "Virtual Sink %s on %s",
                         pa_proplist_gets(u->sink->proplist, "device.vsink.name"), z ? z : dest->name);

        pa_sink_update_proplist(u->sink, PA_UPDATE_REPLACE, pl);
        pa_proplist_free(pl);
    }
}

/* Called from main context */
static void sink_input_volume_changed_cb(pa_sink_input *i) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    pa_sink_volume_changed(u->sink, &i->volume);
}

/* Called from main context */
static void sink_input_mute_changed_cb(pa_sink_input *i) {
    struct userdata *u;

    pa_sink_input_assert_ref(i);
    pa_assert_se(u = i->userdata);

    pa_sink_mute_changed(u->sink, i->muted);
}

static int PAF_AudioFrame_init(PAF_AudioFrame *paf, int max_length_per_channel)
{
    int i;

    paf->sampleCount = FRAME_SIZE;
    paf->data.nChannels = INPUT_CH_CNT;
    paf->data.sample = (PAF_AudioData **)pa_xmalloc0(paf->data.nChannels * sizeof(PAF_AudioData *));
    for(i=0; i<INPUT_CH_CNT ;i++) {
        paf->data.sample[i] = (PAF_AudioData *)pa_xmalloc0(max_length_per_channel *
            sizeof(PAF_AudioData)/sizeof(int16_t));
    }

    return 0;
}

static int PAF_AudioFrame_free(PAF_AudioFrame *paf)
{
    int i;

    for(i=0; i<INPUT_CH_CNT ;i++) {
        pa_xfree(paf->data.sample[i]);
    }
    return 0;
}


static int audio_processing_lib_load(struct userdata *u)
{
	char path_harmanxbar[256];	
    sprintf(path_harmanxbar, PA_DLSEARCHPATH"/libharmancrossbar.so");
	
    u->pDlopen_handle = dlopen(path_harmanxbar, RTLD_NOW);
    if(u->pDlopen_handle == NULL){
        pa_log("dlopen %s failed(%s)", path_harmanxbar, dlerror());
        return -1;
    }

    u->ap_create = dlsym(u->pDlopen_handle, "PulseAudio_AudioProcessingChain_create");
    if(NULL == u->ap_create){
        pa_log("dlsym %s failed.(%s)", path_harmanxbar, dlerror());
        return -2;
    }

    u->ap_apply = dlsym(u->pDlopen_handle, "PulseAudio_AudioProcessing_apply");
    if(NULL == u->ap_apply){
        pa_log("dlsym %s failed..(%s)", path_harmanxbar, dlerror());
        return -2;
    }

    u->ap_msg_write = dlsym(u->pDlopen_handle, "AC_Msg_Write");
    if(NULL == u->ap_msg_write){
        pa_log("dlsym %s failed..(%s)", path_harmanxbar, dlerror());
        return -2;
    }

    u->ap_msg_read = dlsym(u->pDlopen_handle, "AC_Msg_Read");
    if(NULL == u->ap_msg_read){
        pa_log("dlsym %s failed..(%s)", path_harmanxbar, dlerror());
        return -2;
    }

    return 0;

}

static int audio_processing_lib_close(struct userdata *u)
{
    return dlclose(u->pDlopen_handle);
}

int pa__init(pa_module*m) {
    struct userdata *u;
    pa_sample_spec ss;
    pa_channel_map map;
    pa_modargs *ma;
    pa_sink *master=NULL;
    pa_sink_input_new_data sink_input_data;
    pa_sink_new_data sink_data;
    bool use_volume_sharing = true;
    bool force_flat_volume = false;
    pa_memchunk silence;
    char st[128];
    int32_t fade_in_ms;

    pa_assert(m);

    if (!(ma = pa_modargs_new(m->argument, valid_modargs))) {
        pa_log("Failed to parse module arguments.");
        goto fail;
    }

    if (!(master = pa_namereg_get(m->core, pa_modargs_get_value(ma, "master", NULL), PA_NAMEREG_SINK))) {
        pa_log("Master sink not found");
        goto fail;
    }

    pa_assert(master);
    pa_log_debug("master sink name: %s", master->name);
    pa_log_debug("master sample_spec: %s",pa_sample_spec_snprint(st, sizeof(st), &master->sample_spec));

    ss = master->sample_spec;
    ss.format = PA_SAMPLE_S16LE;
    map = master->channel_map;
    if (pa_modargs_get_sample_spec_and_channel_map(ma, &ss, &map, PA_CHANNEL_MAP_DEFAULT) < 0) {
        pa_log("Invalid sample format specification or channel map");
        goto fail;
    }

    pa_log_debug("sample_spec: %s",pa_sample_spec_snprint(st, sizeof(st), &ss));

    if (pa_modargs_get_value_boolean(ma, "use_volume_sharing", &use_volume_sharing) < 0) {
        pa_log("use_volume_sharing= expects a boolean argument");
        goto fail;
    }

    if (pa_modargs_get_value_boolean(ma, "force_flat_volume", &force_flat_volume) < 0) {
        pa_log("force_flat_volume= expects a boolean argument");
        goto fail;
    }

    if (use_volume_sharing && force_flat_volume) {
        pa_log("Flat volume can't be forced when using volume sharing.");
        goto fail;
    }

    fade_in_ms = FADE_IN_MS;
    if (pa_modargs_get_value_s32(ma, "fade_in_ms", &fade_in_ms) < 0) {
        pa_log("Failed to parse fade_in_ms argument.");
        goto fail;
    }

    u = pa_xnew0(struct userdata, 1);
    u->core = m->core;
    u->module = m;
    m->userdata = u;
    u->channels = ss.channels;
    u->sink_streams_APP = sink_streams_APP;
    u->fade_in_frame = fade_in_ms*ss.rate/1000;

    u->protocol = pa_native_protocol_get(m->core);
    pa_native_protocol_install_ext(u->protocol, m, extension_cb);

    /* Create sink */
    pa_sink_new_data_init(&sink_data);
    sink_data.driver = __FILE__;
    sink_data.module = m;
    if (!(sink_data.name = pa_xstrdup(pa_modargs_get_value(ma, "sink_name", NULL))))
        sink_data.name = pa_sprintf_malloc("%s.vsink", master->name);
    pa_sink_new_data_set_sample_spec(&sink_data, &ss);
    pa_sink_new_data_set_channel_map(&sink_data, &map);
    pa_proplist_sets(sink_data.proplist, PA_PROP_DEVICE_MASTER_DEVICE, master->name);
    pa_proplist_sets(sink_data.proplist, PA_PROP_DEVICE_CLASS, "filter");
    pa_proplist_sets(sink_data.proplist, "device.vsink.name", sink_data.name);

    if (pa_modargs_get_proplist(ma, "sink_properties", sink_data.proplist, PA_UPDATE_REPLACE) < 0) {
        pa_log("Invalid properties");
        pa_sink_new_data_done(&sink_data);
        goto fail;
    }

    if ((u->auto_desc = !pa_proplist_contains(sink_data.proplist, PA_PROP_DEVICE_DESCRIPTION))) {
        const char *z;

        z = pa_proplist_gets(master->proplist, PA_PROP_DEVICE_DESCRIPTION);
        pa_proplist_setf(sink_data.proplist, PA_PROP_DEVICE_DESCRIPTION, "Virtual Sink %s on %s", sink_data.name, z ? z : master->name);
    }

    u->sink = pa_sink_new(m->core, &sink_data, (master->flags & (PA_SINK_LATENCY|PA_SINK_DYNAMIC_LATENCY))
                                               | (use_volume_sharing ? PA_SINK_SHARE_VOLUME_WITH_MASTER : 0));
    pa_sink_new_data_done(&sink_data);

    if (!u->sink) {
        pa_log("Failed to create sink.");
        goto fail;
    }

    u->sink->parent.process_msg = sink_process_msg_cb;
    u->sink->set_state = sink_set_state_cb;
    u->sink->update_requested_latency = sink_update_requested_latency_cb;
    u->sink->request_rewind = sink_request_rewind_cb;
    pa_sink_set_set_mute_callback(u->sink, sink_set_mute_cb);
    if (!use_volume_sharing) {
        pa_sink_set_set_volume_callback(u->sink, sink_set_volume_cb);
        pa_sink_enable_decibel_volume(u->sink, true);
    }
    /* Normally this flag would be enabled automatically be we can force it. */
    if (force_flat_volume)
        u->sink->flags |= PA_SINK_FLAT_VOLUME;
    u->sink->userdata = u;

    pa_sink_set_asyncmsgq(u->sink, master->asyncmsgq);

    /* Create sink input */
    pa_sink_input_new_data_init(&sink_input_data);
    sink_input_data.driver = __FILE__;
    sink_input_data.module = m;
    pa_sink_input_new_data_set_sink(&sink_input_data, master, false);
    sink_input_data.origin_sink = u->sink;
    pa_proplist_setf(sink_input_data.proplist, PA_PROP_MEDIA_NAME, "Virtual Sink Stream from %s", pa_proplist_gets(u->sink->proplist, PA_PROP_DEVICE_DESCRIPTION));
    pa_proplist_sets(sink_input_data.proplist, PA_PROP_MEDIA_ROLE, "filter");
    pa_sink_input_new_data_set_sample_spec(&sink_input_data, &ss);
    pa_sink_input_new_data_set_channel_map(&sink_input_data, &map);

    pa_sink_input_new(&u->sink_input, m->core, &sink_input_data);
    pa_sink_input_new_data_done(&sink_input_data);

    if (!u->sink_input)
        goto fail;

    u->sink_input->pop = sink_input_pop_cb;
    u->sink_input->process_rewind = sink_input_process_rewind_cb;
    u->sink_input->update_max_rewind = sink_input_update_max_rewind_cb;
    u->sink_input->update_max_request = sink_input_update_max_request_cb;
    u->sink_input->update_sink_latency_range = sink_input_update_sink_latency_range_cb;
    u->sink_input->update_sink_fixed_latency = sink_input_update_sink_fixed_latency_cb;
    u->sink_input->kill = sink_input_kill_cb;
    u->sink_input->attach = sink_input_attach_cb;
    u->sink_input->detach = sink_input_detach_cb;
    u->sink_input->state_change = sink_input_state_change_cb;
    u->sink_input->moving = sink_input_moving_cb;
    u->sink_input->volume_changed = use_volume_sharing ? NULL : sink_input_volume_changed_cb;
    u->sink_input->mute_changed = sink_input_mute_changed_cb;
    u->sink_input->userdata = u;

    u->sink->input_to_master = u->sink_input;

    pa_sink_input_get_silence(u->sink_input, &silence);
    u->memblockq = pa_memblockq_new("module-hmdsp-sink memblockq", 0, MEMBLOCKQ_MAXLENGTH, 0, &ss, 1, 1, 0, &silence);
    pa_memblock_unref(silence.memblock);

    /* (9) INITIALIZE ANYTHING ELSE YOU NEED HERE */

    pa_sink_put(u->sink);
    pa_sink_input_put(u->sink_input);

    pa_assert(audio_processing_lib_load(u) >= 0);

    pa_log_debug("PAF_AudioFrame_init...");
    PAF_AudioFrame_init(&(u->paf_prepro), pa_sink_get_max_request(master)/(u->channels));

    pa_log_debug("PulseAudio_AudioProcessingChain_create...");

    u->ap_create();

    if (!(u->rx_thread= pa_thread_new("hmdsp-sink-event-rx", thread_func_event_rx, u))) {
        pa_log("Failed to create thread.");
        goto fail;
    }

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

    /* See comments in sink_input_kill_cb() above regarding
     * destruction order! */

    if (u->sink_input)
        pa_sink_input_unlink(u->sink_input);

    if (u->sink)
        pa_sink_unlink(u->sink);

    if (u->sink_input)
        pa_sink_input_unref(u->sink_input);

    if (u->sink)
        pa_sink_unref(u->sink);

    if (u->memblockq)
        pa_memblockq_free(u->memblockq);

    if (u->protocol) {
        pa_native_protocol_remove_ext(u->protocol, m);
        pa_native_protocol_unref(u->protocol);
    }

    PAF_AudioFrame_free(&(u->paf_prepro));

    audio_processing_lib_close(u);

    pa_xfree(u);
}
