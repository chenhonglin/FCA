
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

#ifndef foohmdspsinkconfighfoo
#define foohmdspsinkconfighfoo

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <pulsecore/mix.h>

#define CH_1        1
#define CH_2        2
#define CH_4        4
#define CH_8        8


#define FRAME_SIZE  (64)                              /* 1 frame = 64 samples */
#define BYTES_PER_SAMPLE (sizeof(int16_t) * CH_2)     /* for dump purpose, all channle treated as stereo channel */


#define INPUT_SRC_CNT       15
#define INPUT_CH_CNT        24
#define ZONE_CNT             1
#define SINK_CNT             1                        /* only one sync is supported for audio dump featuer */

typedef struct input_audio_config {
    const char *name;       /* name of the device */
    unsigned int pos;       /* location*/
    unsigned int chs;       /* channel number*/
}input_audio_config;

int get_device_config(pa_mix_info info, input_audio_config* cfg);

#endif

