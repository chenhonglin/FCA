/*
 *  PCM - SyncDump plugin
 *
 *   This library is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation; either version 2.1 of
 *   the License, or (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <ctype.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/inotify.h>
#include <time.h>

#include <alsa/asoundlib.h>
#include <alsa/pcm_external.h>

#ifndef PIC
/* entry for static linking */
const char *_snd_module_pcm_syncdump = "";
#endif

#ifdef __ANDROID__
#include <cutils/log.h>
#undef LOG_TAG
#define LOG_TAG "SYNCDUMP"
#endif

/* maximum length of a key value */
#define VALUE_MAXLEN    64
/* keys to be replaced by real values in the dump name */
#define LEADING_KEY    '%'/* i.e. %r, %c, %b ... */
#define RATE_KEY    'r'
#define CHANNELS_KEY   'c'
#define FORMAT_KEY    'f'
#define NAME_KEY    'n'
#define PATH_KEY    'p'
#define TIME_KEY    't'
#define PROCESS_KEY 'a'

#define MAX_EVENT_SIZE 1024
#define MAX_DUMPID_SIZE 1024

/*dump control for watching*/
#define DUMP_SWITCH "/mnt/misc/audio/dump/control"
/*dump output folder*/
#define DUMP_PATH   "/mnt/misc/audio/dump/"
/*thread length in /proc/pid/task/comm*/
#define MAX_TASK_COMM_LEN 16

#define CtrlPipe_Close "close"

typedef struct {
	snd_pcm_extplug_t ext;
	char *fname;
	char *final_fname;
	int perm;
	int fd;
	char *ifname;
	int ifd;
	snd_pcm_uframes_t appl_ptr;
	snd_pcm_uframes_t dump_ptr_bytes;
	snd_pcm_uframes_t wbuf_size;
	size_t wbuf_size_bytes;
	size_t wbuf_used_bytes;
	char *wbuf;
	size_t rbuf_size_bytes;
	size_t rbuf_used_bytes;
	char *rbuf;
	snd_pcm_channel_area_t *wbuf_areas;
	size_t buffer_bytes;
	size_t dumplen;
	int inotify_fd;
	int pipe_fd[2];
	pthread_t thread;
	pthread_mutex_t mutex;
	int first_dump;
	unsigned char id;
	unsigned int rate;
	unsigned int channels;
	snd_pcm_format_t pcm_format;
	int frame_bits;
	const char *name;
} snd_pcm_syncdump_t;

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define TO_LE32(x)      (x)
#define TO_LE16(x)      (x)
#else
#define TO_LE32(x)      bswap_32(x)
#define TO_LE16(x)      bswap_16(x)
#endif

static void print_local_time(const char *str)
{
	struct timespec timespec;

	clock_gettime(CLOCK_MONOTONIC, &timespec);
	ALOGD("%s time %ds, %dns", str, timespec.tv_sec, timespec.tv_nsec);
}

static int snd_pcm_syncdump_append_value(char **string_p, char **index_ch_p,
		int *len_p, const char *value)
{
	char *string, *string1,  *index_ch;
	int index, len, value_len;
	/* input pointer values */
	len = *(len_p);
	string = *(string_p);
	index_ch = *(index_ch_p);

	value_len = strlen(value);
	/* reallocation to accommodate the value */
	index = index_ch - string;
	len += value_len;
	string1 = realloc(string, len + 1);
	if (!string1) {
		free(string);
		return -ENOMEM;
	} else {
		string = string1;
	}
	index_ch = string + index;
	/* concatenating the new value */
	strcpy(index_ch, value);
	index_ch += value_len;
	/* return values */
	*(len_p) = len;
	*(string_p) = string;
	*(index_ch_p) = index_ch;
	return 0;
}

static int snd_pcm_syncdump_replace_fname(snd_pcm_syncdump_t *syncdump, char **new_fname_p, int final_len)
{
	char value[VALUE_MAXLEN];
	char *fname = syncdump->fname;
	char *old_last_ch, *old_index_ch, *new_index_ch;
	int old_len,  err = -1;
	struct tm *ptm;
	long ts;
	pid_t pid;
	int fd, ret;
	char application_name[10];


	/* we want to keep fname, const */
	old_len = strlen(fname);
	old_last_ch = fname + old_len - 1;

	old_index_ch = fname;	/* first character of the old name */
	new_index_ch = *new_fname_p;	/* first char of the new name */

	while (old_index_ch <= old_last_ch) {
		if (*(old_index_ch) == LEADING_KEY &&
				old_index_ch != old_last_ch) {
			/* is %, not last char, skipping and checking
			 next char */
			switch (*(++old_index_ch)) {
			case RATE_KEY:
				snprintf(value, sizeof(value), "%d",
						syncdump->rate);
				err = snd_pcm_syncdump_append_value(new_fname_p,
					&new_index_ch, &final_len, value);
				if (err < 0)
					return err;
				break;

			case CHANNELS_KEY:
				snprintf(value, sizeof(value), "%d",
						syncdump->channels);
				err = snd_pcm_syncdump_append_value(new_fname_p,
					&new_index_ch, &final_len, value);
				if (err < 0)
					return err;
				break;

			case FORMAT_KEY:
				err = snd_pcm_syncdump_append_value(new_fname_p,
					&new_index_ch, &final_len,
					snd_pcm_format_name(syncdump->pcm_format));
				if (err < 0)
					return err;
				break;

			case NAME_KEY:
				err = snd_pcm_syncdump_append_value(new_fname_p,
					&new_index_ch, &final_len,
					syncdump->name);
				if (err < 0)
					return err;
				break;

			case PATH_KEY:
				err = snd_pcm_syncdump_append_value(new_fname_p,
					&new_index_ch, &final_len,
					DUMP_PATH);
				if (err < 0)
					return err;
				break;

			case TIME_KEY:
				ts = time(NULL);
				ptm = localtime(&ts);
				if(ptm != NULL) {
					snprintf(value, sizeof(value), "%04d%02d%02d%02d%02d%02d",
							ptm->tm_year+1900, ptm->tm_mon+1,ptm->tm_mday,ptm->tm_hour,
							ptm->tm_min,ptm->tm_sec);
					err = snd_pcm_syncdump_append_value(new_fname_p,
						&new_index_ch, &final_len,
						value);
				}
				if (err < 0)
					return err;
				break;

			case PROCESS_KEY:
				pid = getpid();
				fd = open("/proc/self/comm", O_RDONLY, 0666);
				if(fd < 0){
					ALOGE("open /proc/self/comm failed");
					break;
				}

				ret = read(fd, application_name, 9);
				if(ret <= 0){
					ALOGE("read /proc/self/comm failed");
					close(fd);
					break;
				}
				close(fd);

				application_name[ret-1] = '\0';

				snprintf(value, sizeof(value), "%s_%d",
					application_name, pid);
				err = snd_pcm_syncdump_append_value(new_fname_p,
						&new_index_ch, &final_len, value);
				if (err < 0)
					return err;
				break;


			default:
				/* non-key char, just copying */
				*(new_index_ch++) = *(old_index_ch);
			}
			/* next old char */
			old_index_ch++;
		} else {
			/* plain copying, shifting both strings to next chars */
			*(new_index_ch++) = *(old_index_ch++);
		}
	}
	/* closing the new string */
	*(new_index_ch) = '\0';
	return 0;

}


static int audiodump_output_dump_open(snd_pcm_syncdump_t *syncdump)
{
	int err, final_len, fd;
	char *fname = syncdump->fname;

	final_len = strlen(fname);
	syncdump->final_fname = malloc(final_len + 1);
	if (!syncdump->final_fname )
		return -ENOMEM;

	/* fname can contain keys, generating final_fname */
	err = snd_pcm_syncdump_replace_fname(syncdump, &(syncdump->final_fname), final_len);
	if (err < 0)
		return err;

	ALOGD("syncdump: original fname: %s, final fname: %s\n",
			syncdump->fname, syncdump->final_fname);

	fd = open(syncdump->final_fname, O_WRONLY|O_CREAT|O_EXCL, syncdump->perm);
	if (fd < 0) {
		ALOGE("syncdump : open %s for writing failed", syncdump->final_fname);
		free(syncdump->final_fname);
		syncdump->final_fname = NULL;
		return -1;
	}

	free(syncdump->final_fname);
	syncdump->final_fname = NULL;
	syncdump->fd = fd;
	return 0;
}


static int audiodump_output_dump_close(snd_pcm_syncdump_t *syncdump)
{
	int ret;

	assert(syncdump->fd);
	ret = close(syncdump->fd);
	syncdump->fd = -1;

	return ret;
}

static void *watch_thread(void *data)
{
	int fd;
	int r, i;
	char event[MAX_EVENT_SIZE];
	char content[MAX_DUMPID_SIZE] = {0};
	char thread_name[MAX_TASK_COMM_LEN]= {0};
	int max = -1;
	fd_set read_fds;
	snd_pcm_syncdump_t *syncdump = data;

	/* set thread name */
	sprintf(thread_name, "syncdump-ID[%c]", toascii(syncdump->id));
	pthread_setname_np(pthread_self(), thread_name);

	while(1) {
		FD_ZERO(&read_fds);
		FD_SET(syncdump->pipe_fd[0], &read_fds);
		if(syncdump->pipe_fd[0] > max)
			max = syncdump->pipe_fd[0];

		FD_SET(syncdump->inotify_fd, &read_fds);
		if(syncdump->inotify_fd > max)
			max = syncdump->inotify_fd;

		/* let's sleep waiting for some event happen */
		ALOGD("syncdump: monitor thread %c goes into sleep...", toascii(syncdump->id));
		r = select(max + 1, &read_fds, NULL, NULL, NULL);
		if( r < 0) {
			ALOGE("Select failed %s\n",strerror(errno));
			continue;
		} else {
			ALOGD("syncdump: monitor thread %c wake up", toascii(syncdump->id));
		}

		/* wakeup, it is time to exit */
		if(FD_ISSET(syncdump->pipe_fd[0], &read_fds)) {
			ALOGD("syncdump: received message\n");
			char cmd[10] = {0};
			r = read(syncdump->pipe_fd[0], cmd, strlen(CtrlPipe_Close));
			if(r < 0) {
				ALOGD("syncdump: read pipe failed, continue...\n");
				continue;
			}
			if(!strcmp(cmd, CtrlPipe_Close)) {
				ALOGD("syncdump: received exit message, thread exiting\n");
				pthread_exit(0);
			} else {
				ALOGE("syncdump: unknown message %s from pipe, ignore\n", cmd);
			}
		}

		/* wakeup, something happening on dump switch file */
		if(FD_ISSET(syncdump->inotify_fd, &read_fds)) {
			ALOGD("syncdump: dump switch file be modified\n");
			r = read(syncdump->inotify_fd, event, MAX_EVENT_SIZE);
			if(r < (int)sizeof(struct inotify_event)) {
				ALOGD("syncdump: Counld not get event from inotify!\n");
				continue;
			}
			fd = open(DUMP_SWITCH, O_RDONLY, 0666);
			if(fd < 0) {
				ALOGE("syncdump: open dump control file failed\n");
				continue;
			}

			memset(content, 0, sizeof(content));
			r = read(fd, &content, MAX_DUMPID_SIZE);
			if( r <= 0){
				ALOGE("syncdump : read dump control file failed\n");
				close(fd);
				continue;
			}
			close(fd);

			ALOGD("syncdump : command: %s", content);
			ALOGD("syncdump : my ID: %c", toascii(syncdump->id));

			/*dump stop*/
			if (content[0] == '0') {
				if(syncdump->fd > 0){
					ALOGD("syncdump: stop dump");
					audiodump_output_dump_close(syncdump);
				}
				continue;
			}

			/*dump start*/
			for(i=0; i < r; i++) {
				if((content[i] == syncdump->id) || (content[0] == '1')){
					ALOGD("syncdump: ID [%c] match", syncdump->id);
					syncdump->first_dump = 1;
					if(syncdump->fd < 0) {
						ALOGD("syncdump: start dump");
						audiodump_output_dump_open(syncdump);
					}else{
						ALOGD("syncdump: already in dump state, ignore");
					}
					break;
				}
			}
		}
        } /*end of while*/
	return NULL;
}

static int setup_inotify(snd_pcm_syncdump_t *syncdump) {
	int r;

	if (syncdump->inotify_fd >= 0)
		return 0;

	if ((syncdump->inotify_fd = inotify_init()) < 0) {
		ALOGE("inotify_init() failed: %s", strerror(errno));
		return -1;
	}

	r = inotify_add_watch(syncdump->inotify_fd, DUMP_SWITCH, IN_MODIFY);
	if (r < 0) {
		int saved_errno = errno;
		close(syncdump->inotify_fd);
		syncdump->inotify_fd = -1;

	        if (saved_errno == ENOENT) {
			ALOGE("watch target %s is not existing yet", DUMP_SWITCH);
			return 0;
	        }
	        ALOGE("inotify_add_watch() failed: %s", strerror(saved_errno));
		return -1;
	}

	r = pipe(syncdump->pipe_fd);
	if (r < 0) {
		ALOGE("syncdump: pipe() failed: %s", strerror(errno));
		return -1;
	}

	pthread_mutex_init(&syncdump->mutex, NULL);
	pthread_mutex_lock(&syncdump->mutex);
	pthread_create(&syncdump->thread, NULL, watch_thread, syncdump);
	pthread_mutex_unlock(&syncdump->mutex);

	return 0;
}

static int stop_inotify(snd_pcm_syncdump_t *syncdump) {

	int r;
	char *c = CtrlPipe_Close;

	ALOGD("syncdump: sending exit message to watch thread...");
	r = write(syncdump->pipe_fd[1], c, strlen(CtrlPipe_Close));
	if(r < 0) {
		ALOGE("write to control pipe failed %s", strerror(errno));
		return r;
	}

	ALOGD("syncdump: waiting for thread exit done...");
	r = pthread_join(syncdump->thread, NULL);
	if(r < 0) {
		ALOGE("syncdump: pthread_join() failed %d", r);
		return r;
        }
	ALOGD("syncdump: thread exit done");

	r = close(syncdump->inotify_fd);
	if(r < 0) {
		ALOGE("close inotify_fd failed %d", r);
		return r;
	}

	r = close(syncdump->pipe_fd[0]);
	if(r < 0) {
		ALOGE("close pipe[0] failed %d", r);
		return r;
	}

	r = close(syncdump->pipe_fd[1]);
	if(r < 0) {
		ALOGE("close pipe[1] failed %d", r);
		return r;
	}

	syncdump->inotify_fd = -1;
	syncdump->pipe_fd[0] = -1;
	syncdump->pipe_fd[1] = -1;

	return 0;
}


static inline void *channel_area_addr(const snd_pcm_channel_area_t *area, snd_pcm_uframes_t offset)
{
        unsigned int bitofs = area->first + area->step * offset;
        assert(bitofs % 8 == 0);
        return (char *) area->addr + bitofs / 8;
}

static snd_pcm_sframes_t
syncdump_transfer(snd_pcm_extplug_t *ext,
             const snd_pcm_channel_area_t *dst_areas,
             snd_pcm_uframes_t dst_offset,
             const snd_pcm_channel_area_t *src_areas,
             snd_pcm_uframes_t src_offset,
             snd_pcm_uframes_t size)
{
	int err;
	snd_pcm_syncdump_t *syncdump = (snd_pcm_syncdump_t *)ext;
	snd_pcm_t *pcm = ext->pcm;
	size_t bytes = snd_pcm_frames_to_bytes(pcm, size);
        short *src = channel_area_addr(src_areas, src_offset);
        short *dst = channel_area_addr(dst_areas, dst_offset);

	memcpy(dst, src, bytes);

        if(syncdump->fd > 0){
                if(syncdump->first_dump) {
			syncdump->first_dump = 0;
			print_local_time("dump write first data");
		}
		err = write(syncdump->fd, dst, bytes);
		if (err < 0) {
			ALOGE("syncdump: write dump file failed:%s", strerror(errno));
		}
	}
	return size;
}

static int snd_pcm_syncdump_close(snd_pcm_extplug_t *ext)
{
	snd_pcm_syncdump_t *syncdump = (snd_pcm_syncdump_t *)ext;
	if (syncdump->fname) {
		free((void *)syncdump->fname);
		if (syncdump->fd >= 0) {
			close(syncdump->fd);
		}
	}
	if (syncdump->ifname) {
		free((void *)syncdump->ifname);
		close(syncdump->ifd);
	}

	/* stop watching the dump switch point*/
	if(stop_inotify(syncdump) < 0)
		ALOGE("syncdump: stop_inotify() failed");

	return 0;
}

static int snd_pcm_syncdump_hw_params(snd_pcm_extplug_t *ext, snd_pcm_hw_params_t * params)
{
	snd_pcm_syncdump_t *syncdump = (snd_pcm_syncdump_t *)ext;
	int err;
	snd_pcm_t *pcm = ext->pcm;

	err = snd_pcm_hw_params_get_rate(params, &syncdump->rate, 0);
	if (err < 0) {
		ALOGE("get rate of hw_params failed!");
		return -1;
	}

	err = snd_pcm_hw_params_get_channels(params, &syncdump->channels);
	if (err < 0) {
		ALOGE("get channels of hw_params failed!");
		return -1;
	}

	err = snd_pcm_hw_params_get_format(params, &syncdump->pcm_format);
	if (err < 0) {
		ALOGE("get format of hw_params failed!");
		return -1;
	}

	syncdump->frame_bits = snd_pcm_format_physical_width(syncdump->pcm_format);
	if (syncdump->frame_bits < 0) {
		ALOGE("get frame bits of hw_params failed!");
		return -1;
	}

	return 0;
}

static const snd_pcm_extplug_callback_t syncdump_callback = {
	.transfer = syncdump_transfer,
	.close = snd_pcm_syncdump_close,
	.hw_params = snd_pcm_syncdump_hw_params,
};



/**
 * \brief Creates a new File PCM
 */
int snd_pcm_syncdump_open(snd_pcm_syncdump_t **sd, const char *name,
		      const char *fname, int ifd, int perm, char id)
{
	struct timespec timespec;
	snd_pcm_syncdump_t *syncdump = NULL;

	assert(syncdump);
	syncdump = calloc(1, sizeof(snd_pcm_syncdump_t));
	if (!syncdump) {
		return -ENOMEM;
	}

	ALOGD("device ID %c", toascii(id));

        syncdump->ext.version = SND_PCM_EXTPLUG_VERSION;
        syncdump->ext.name = "Syncdump Plugin";
        syncdump->ext.callback = &syncdump_callback;
        syncdump->ext.private_data = syncdump;

	/* opening output fname is delayed until writing,
	 when PCM params are known */
	if (fname)
		syncdump->fname = strdup(fname);
	syncdump->perm = perm;

	if(id)
		syncdump->id = id;

	syncdump->fd = -1;
	syncdump->ifd = ifd;
	syncdump->inotify_fd = -1;
	syncdump->first_dump = 0;
	syncdump->name = name;

	*sd = syncdump;

	return 0;
}

int check_id(const char *id)
{
        static const char ids[3][8] = { "comment", "type", "hint" };
        unsigned int k;
        for (k = 0; k < sizeof(ids) / sizeof(ids[0]); ++k) {
                if (strcmp(id, ids[k]) == 0)
                        return 1;
        }
        return 0;
}

SND_PCM_PLUGIN_DEFINE_FUNC(syncdump)
{
	snd_config_iterator_t i, next;
	int err;
	snd_pcm_t *spcm;
	snd_config_t *slave = NULL;
	const char *fname = NULL, *ifname = NULL;
	long ifd = -1;
	char *device_id = NULL;
	long perm = 0600;
        int channels = 0;
        snd_pcm_format_t format = SND_PCM_FORMAT_UNKNOWN;
	snd_pcm_syncdump_t *syncdump = NULL;

	snd_config_for_each(i, next, conf) {
		snd_config_t *n = snd_config_iterator_entry(i);
		const char *id;
		if (snd_config_get_id(n, &id) < 0)
			continue;
		if (check_id(id))
			continue;
		if (strcmp(id, "slave") == 0) {
			slave = n;
			continue;
		}
		if (strcmp(id, "file") == 0) {
			err = snd_config_get_string(n, &fname);
			if (err < 0) {
				ALOGE("Invalid type for %s", id);
				return -EINVAL;
			}
			continue;
		}
		if (strcmp(id, "infile") == 0) {
			err = snd_config_get_string(n, &ifname);
			if (err < 0) {
				err = snd_config_get_integer(n, &ifd);
				if (err < 0) {
					ALOGE("Invalid type for %s", id);
					return -EINVAL;
				}
			}
			continue;
		}
		if (strcmp(id, "perm") == 0) {
			err = snd_config_get_integer(n, &perm);
			if (err < 0) {
				ALOGE("Invalid type for %s", id);
				return err;
			}
			if ((perm & ~0777) != 0) {
				ALOGE("The field perm must be a valid permission");
				return -EINVAL;
			}
			continue;
		}
                if (strcmp(id, "format") == 0) {
			const char *str;
                        err = snd_config_get_string(n, &str);
                        if (err < 0) {
                                ALOGE("invalid type for %s", id);
				return -EINVAL;
                        }
                        format = snd_pcm_format_value(str);
                        continue;
                }
                if (strcmp(id, "channels") == 0) {
                        long val;
                        err = snd_config_get_integer(n, &val);
                        if (err < 0) {
                                ALOGE("Invalid type for %s", id);
				return -EINVAL;
                        }
                        channels = val;
                        continue;
                }
		if (strcmp(id, "id") == 0) {
			err = snd_config_get_ascii(n, &device_id);
			if (err < 0)
				return -EINVAL;
			continue;
		}
		ALOGE("Unknown field %s", id);
		return -EINVAL;
	}

	if(!device_id)
		return -EINVAL;
	err = snd_pcm_syncdump_open(&syncdump, name, fname, ifd, perm, *device_id);
        if (err < 0) {
	    ALOGE("alloc syncdump failed!");
	    return err;
	}

        err = snd_pcm_extplug_create(&syncdump->ext, name, root, slave,
                                     stream, mode);
        if (err < 0) {
	    ALOGE("Create extplug syncdump failed!");
	    free(syncdump);
	    return err;
	}


        err = snd_pcm_extplug_set_param(&syncdump->ext, SND_PCM_EXTPLUG_HW_CHANNELS, channels);
        if (err < 0) {
	    ALOGE("set syncdump Channels failed!");
	    free(syncdump);
	    return err;
	}

        err = snd_pcm_extplug_set_slave_param(&syncdump->ext,
                                        SND_PCM_EXTPLUG_HW_CHANNELS, channels);
        if (err < 0) {
	    ALOGE("set syncdump slave  Channels failed!");
	    free(syncdump);
	    return err;
	}

        err = snd_pcm_extplug_set_param(&syncdump->ext, SND_PCM_EXTPLUG_HW_FORMAT, format);
        if (err < 0) {
	    ALOGE("set syncdump format failed!");
	    free(syncdump);
	    return err;
	}

        err = snd_pcm_extplug_set_slave_param(&syncdump->ext, SND_PCM_EXTPLUG_HW_FORMAT, format);
        if (err < 0) {
	    ALOGE("set syncdump slave format failed!");
	    free(syncdump);
	    return err;
	}


	/* start watching the dump switch point*/
	setup_inotify(syncdump);
	*pcmp = syncdump->ext.pcm;
	return err;
}

SND_PCM_PLUGIN_SYMBOL(syncdump);
