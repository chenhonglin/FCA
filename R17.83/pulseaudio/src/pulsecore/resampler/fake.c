/***
  This file is part of PulseAudio.

  Copyright 2004-2006 Lennart Poettering

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

#include <pulse/xmalloc.h>
#include <pulsecore/resampler.h>

struct fakesrc_data { /* data specific to the resampler */
    unsigned o_counter;
    unsigned i_counter;
};

static unsigned fakesrc_resample(pa_resampler *r, const pa_memchunk *input, unsigned in_n_frames, pa_memchunk *output, unsigned *out_n_frames) {
    pa_assert(r);
    pa_assert(out_n_frames);

    *out_n_frames = ((uint64_t) in_n_frames * r->o_ss.rate) / r->i_ss.rate;

    //pa_log("in_n_frames = %d, *out_n_frames =%d", in_n_frames, *out_n_frames);

    return 0;
}

static void fakesrc_update_rates_or_reset(pa_resampler *r) {
    struct fakesrc_data *fakesrc_data;
    pa_assert(r);

    fakesrc_data = r->impl.data;
    fakesrc_data->i_counter = 0;
    fakesrc_data->o_counter = 0;
}


int pa_resampler_fake_init(pa_resampler *r) {
    struct fakesrc_data *fakesrc_data;
    pa_assert(r);

    fakesrc_data = pa_xnew0(struct fakesrc_data, 1);

    r->impl.resample = fakesrc_resample;
    r->impl.update_rates = fakesrc_update_rates_or_reset;
    r->impl.reset = fakesrc_update_rates_or_reset;
    r->impl.data = fakesrc_data;

    return 0;
}
