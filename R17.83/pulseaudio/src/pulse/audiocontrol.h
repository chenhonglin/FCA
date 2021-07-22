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


#ifndef fooaudiocontrolhfoo
#define fooaudiocontrolhfoo

#include <pulse/thread-mainloop.h>
#include <pulse/context.h>
#include <pulse/introspect.h>
#include <pulse/ext-dsp-framework.h>

/* Try enough times to make sure the context is in ready status */
#define CONNECT_TRY_CNT     (400)

#define REMOTE_MODULE_NAME  "module-combine-sink"

struct pa_ac_simple {
    pa_threaded_mainloop *mainloop;
    pa_context *context;
    uint8_t operation_success;
};

struct pa_ac_simple *pa_ac_create_connection(void);
void pa_ac_set_event_callback(struct pa_ac_simple *s, event_cb_t cb);
void pa_ac_send_command(struct pa_ac_simple *s, t_Tagged_MSG_Params *msg, cmd_status_cb_t cb);
void pa_ac_set_dump(struct pa_ac_simple *s, t_dump_params *dump_params, dump_status_cb_t cb);
int  pa_ac_is_connection_good(struct pa_ac_simple *s);
void pa_ac_disconnect(struct pa_ac_simple *s);
pa_operation* pa_ac_load_module(struct pa_ac_simple *s, const char*name, const char *argument, pa_context_index_cb_t cb, void *userdata);
pa_operation* pa_ac_unload_module(struct pa_ac_simple *s, uint32_t idx, pa_context_success_cb_t cb, void *userdata);
pa_operation* pa_ac_suspend_source_by_name(struct pa_ac_simple *s, const char *source_name, int suspend, pa_context_success_cb_t cb, void* userdata);
pa_operation* pa_ac_mute_source_and_output_by_name(struct pa_ac_simple *s, const char *source_name, const char *output_name, int mute, pa_context_success_cb_t cb, void* userdata);

#endif
