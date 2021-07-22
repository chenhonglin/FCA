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

#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include <pulse/audiocontrol.h>
#include <pulse/pulseaudio.h>
#include <pulse/internal.h>
#include <pulse/xmalloc.h>
#include <pulse/operation.h>
#include <pulse/ext-dsp-framework.h>
#include <pulse/ext-device-restore.h>
#include <pulse/ext-stream-restore.h>
#include <pulse/internal.h>


#include <pulsecore/log.h>
#include <pulsecore/tagstruct.h>
#include <pulsecore/i18n.h>
#include <pulsecore/macro.h>
#include <pulsecore/core-util.h>
#include <pulsecore/log.h>
#include <pulsecore/pstream-util.h>


static void context_state_callback(pa_context *c, void *userdata) {
    struct pa_ac_simple *p = userdata;

    pa_assert(c);
    pa_assert(p);

    switch (pa_context_get_state(c)) {
        case PA_CONTEXT_READY:
        case PA_CONTEXT_TERMINATED:
        case PA_CONTEXT_FAILED:
            pa_threaded_mainloop_signal(p->mainloop, 0);
            break;

        case PA_CONTEXT_UNCONNECTED:
        case PA_CONTEXT_CONNECTING:
        case PA_CONTEXT_AUTHORIZING:
        case PA_CONTEXT_SETTING_NAME:
            break;
    }
}

static void context_event_callback(pa_context *c, const char *event, pa_proplist *p, void *event_userdata)
{
    size_t nbytes = 0;
    event_cb_t cb = (event_cb_t)event_userdata;
    t_Tagged_MSG_Params *msg = NULL;

    pa_assert(c);

    if(strcmp(event, DSP_EVENT)){
        pa_log("ignores event message it doesn't know");
        return;
    }

    pa_proplist_get(p, PA_PROP_DSP_EVENT, (void *)&msg, &nbytes);
    cb(msg);
}


/*create a connection to pulseaudio, waiting for the processing complete within 1000ms*/
struct pa_ac_simple *pa_ac_create_connection(void)
{
    int ret = PA_ERR_MAX;
    int connect_try = 0;
    pa_mainloop_api *mainloop_api = NULL;
    struct pa_ac_simple *p = NULL;

    p = pa_xnew0(struct pa_ac_simple, 1);

    /* Set up a new mainloop */
    if (!(p->mainloop = pa_threaded_mainloop_new())) {
        pa_log(_("pa_mainloop_new() failed."));
        goto quit;
    }

    if (!(mainloop_api = pa_threaded_mainloop_get_api(p->mainloop))) {
        pa_log(_("pa_threaded_mainloop_get_api() failed."));
        goto quit;
    }

    p->context = pa_context_new(mainloop_api, NULL);
    if (p->context == NULL) {
        pa_log(_("pa_context_new() failed."));
        goto quit;
    }

    pa_context_set_state_callback(p->context, context_state_callback, p);

    while(connect_try < CONNECT_TRY_CNT) {
        if (pa_context_connect(p->context, NULL, PA_CONTEXT_NOFAIL, NULL) < 0) {
            connect_try++;
            pa_log(_("pa_context_connect() failed %d times: %s"), connect_try, pa_strerror(pa_context_errno(p->context)));
        } else {
            break;
        }

        if(connect_try == CONNECT_TRY_CNT) {
            pa_log(_("pa_context_connect() failed finally !!!!!!!"));
            goto quit;
        }

        pa_msleep(5);
    }

    pa_threaded_mainloop_lock(p->mainloop);

    if(pa_threaded_mainloop_start(p->mainloop) < 0){
        pa_log(_("pa_threaded_mainloop_start() failed"));
        goto unlock_and_fail;
    }

    while(1) {
        pa_context_state_t state;

        state = pa_context_get_state(p->context);
        if (state == PA_CONTEXT_READY)
            break;

        if (!PA_CONTEXT_IS_GOOD(state)) {
            ret = pa_context_errno(p->context);
            goto unlock_and_fail;
        }

        /* Wait until the context is ready */
        pa_threaded_mainloop_wait(p->mainloop);
    }

    pa_threaded_mainloop_unlock(p->mainloop);

    return p;

unlock_and_fail:
        pa_threaded_mainloop_unlock(p->mainloop);

quit:
    pa_ac_disconnect(p);
    return NULL;
}


void pa_ac_set_event_callback(struct pa_ac_simple *s, event_cb_t cb)
{
    pa_proplist *proplist = NULL;

    pa_assert(s);

    if(!cb){
        pa_log_error("%s: callback is NULL",__func__);
    }

    proplist = pa_proplist_new();
    pa_proplist_sets(proplist, PA_PROP_AUDIO_MANAGER, "true");
    pa_context_proplist_update(s->context, PA_UPDATE_REPLACE, proplist, NULL, NULL);
    pa_context_set_event_callback(s->context, context_event_callback, cb);
}


/*send a AC command to DSP*/
/*this function always return OK, the command status is check in CB*/
void pa_ac_send_command(struct pa_ac_simple *s, t_Tagged_MSG_Params *msg, cmd_status_cb_t cb)
{
    pa_threaded_mainloop_lock(s->mainloop);
    pa_context_ext_audio_framework_msg_send(s->context, MSG_AM_CMD, \
        (void *)msg, cb, s, REMOTE_MODULE_NAME);
    pa_threaded_mainloop_unlock(s->mainloop);
}


void pa_ac_set_dump(struct pa_ac_simple *s, t_dump_params *dump_params, dump_status_cb_t cb)
{
    pa_context_ext_audio_framework_msg_send(s->context, MSG_DUMP_CMD, \
        (void *)dump_params, cb, s, REMOTE_MODULE_NAME);

}

int pa_ac_is_connection_good(struct pa_ac_simple *s)
{
    pa_assert(s);
    int ret = 0;
    pa_context_state_t state;

    if (!s->context || !s->mainloop)
        return -EBADFD;

        pa_threaded_mainloop_lock(s->mainloop);
    state = pa_context_get_state(s->context);
    if (!PA_CONTEXT_IS_GOOD(state))
        ret = -EIO;
        pa_threaded_mainloop_unlock(s->mainloop);

    return ret;
}

void pa_ac_disconnect(struct pa_ac_simple *s)
{
    pa_assert(s);

    if (s->mainloop)
    {
        pa_threaded_mainloop_stop(s->mainloop);

        pa_threaded_mainloop_lock(s->mainloop);
        if (s->context){
            pa_context_disconnect(s->context);
            pa_context_unref(s->context);
        }
        pa_threaded_mainloop_unlock(s->mainloop);

        pa_threaded_mainloop_free(s->mainloop);
    }

    pa_xfree(s);
}

pa_operation* pa_ac_load_module(struct pa_ac_simple *s, const char*name, const char *argument, pa_context_index_cb_t cb, void *userdata) {
    pa_operation *o = NULL;

    if (s->mainloop) {
        pa_threaded_mainloop_lock(s->mainloop);
        o = pa_context_load_module(s->context, name, argument, cb, userdata);
        pa_threaded_mainloop_unlock(s->mainloop);
    }

    return o;
}

pa_operation* pa_ac_unload_module(struct pa_ac_simple *s, uint32_t idx, pa_context_success_cb_t cb, void *userdata) {
    pa_operation *o = NULL;

    if (s->mainloop) {
        //pa_log("pa_ac_unload_module");
        pa_threaded_mainloop_lock(s->mainloop);
        o = pa_context_unload_module(s->context, idx, cb, userdata);
        pa_threaded_mainloop_unlock(s->mainloop);
    }

    return o;
}

pa_operation* pa_ac_suspend_source_by_name(struct pa_ac_simple *s, const char *source_name, int suspend, pa_context_success_cb_t cb, void* userdata) {
    pa_operation *o = NULL;

    if (s->mainloop) {
        //pa_log("pa_ac_suspend_source_by_name");
        pa_threaded_mainloop_lock(s->mainloop);
        o = pa_context_suspend_source_by_name(s->context, source_name, suspend, cb, userdata);
        pa_threaded_mainloop_unlock(s->mainloop);
    }

    return o;
}

pa_operation* pa_ac_mute_source_and_output_by_name(struct pa_ac_simple *s, const char *source_name, const char *output_name, int mute, pa_context_success_cb_t cb, void* userdata) {
    pa_operation *o = NULL;

    if (s->mainloop) {
        pa_log_debug("pa_ac_mute_source_and_output_by_name call");
        pa_threaded_mainloop_lock(s->mainloop);
        o = pa_context_mute_source_and_output_by_name(s->context, source_name, output_name, mute, cb, userdata);
        pa_threaded_mainloop_unlock(s->mainloop);
    }

    return o;
}


