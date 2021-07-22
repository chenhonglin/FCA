/***
  This file is part of PulseAudio.

  Copyright 2004-2006 Lennart Poettering
  Copyright 2006 Pierre Ossman <ossman@cendio.se> for Cendio AB

  PulseAudio is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; either version 2.1 of the
  License, or (at your option) any later version.

  PulseAudio is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with PulseAudio; if not, see <http://www.gnu.org/licenses/>.
***/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif

#include <pulse/xmalloc.h>

#include <pulsecore/socket.h>
#include <pulsecore/queue.h>
#include <pulsecore/log.h>
#include <pulsecore/creds.h>
#include <pulsecore/refcnt.h>
#include <pulsecore/flist.h>
#include <pulsecore/macro.h>

#include "pstream.h"

#define PA_STREAM_BSM_MAX (4*8*48*8)
/* We piggyback information if audio data blocks are stored in SHM on the seek mode */
#define PA_FLAG_SHMDATA     0x80000000LU
#define PA_FLAG_SHMRELEASE  0x40000000LU
#define PA_FLAG_SHMREVOKE   0xC0000000LU
#define PA_FLAG_SHMMASK     0xFF000000LU
#define PA_FLAG_SEEKMASK    0x000000FFLU
#define PA_FLAG_SHMWRITABLE 0x00800000LU

/* The sequence descriptor header consists of 5 32bit integers: */
enum {
    PA_PSTREAM_DESCRIPTOR_LENGTH,
    PA_PSTREAM_DESCRIPTOR_CHANNEL,
    PA_PSTREAM_DESCRIPTOR_OFFSET_HI,
    PA_PSTREAM_DESCRIPTOR_OFFSET_LO,
    PA_PSTREAM_DESCRIPTOR_FLAGS,
    PA_PSTREAM_DESCRIPTOR_MAX
};

/* If we have an SHM block, this info follows the descriptor */
enum {
    PA_PSTREAM_SHM_BLOCKID,
    PA_PSTREAM_SHM_SHMID,
    PA_PSTREAM_SHM_INDEX,
    PA_PSTREAM_SHM_LENGTH,
    PA_PSTREAM_SHM_MAX
};

typedef uint32_t pa_pstream_descriptor[PA_PSTREAM_DESCRIPTOR_MAX];

#define PA_PSTREAM_DESCRIPTOR_SIZE (PA_PSTREAM_DESCRIPTOR_MAX*sizeof(uint32_t))

#define MINIBUF_SIZE (256)

/* To allow uploading a single sample in one frame, this value should be the
 * same size (16 MB) as PA_SCACHE_ENTRY_SIZE_MAX from pulsecore/core-scache.h.
 */
#define FRAME_SIZE_MAX_ALLOW (1024*1024*16)

PA_STATIC_FLIST_DECLARE(items, 0, pa_xfree);

struct item_info {
    enum {
        PA_PSTREAM_ITEM_PACKET,
        PA_PSTREAM_ITEM_MEMBLOCK,
        PA_PSTREAM_ITEM_SHMRELEASE,
        PA_PSTREAM_ITEM_SHMREVOKE
    } type;

    /* packet info */
    pa_packet *packet;
#ifdef HAVE_CREDS
    bool with_ancil_data;
    pa_cmsg_ancil_data ancil_data;
#endif

    /* memblock info */
    pa_memchunk chunk;
    uint32_t channel;
    int64_t offset;
    pa_seek_mode_t seek_mode;

    /* release/revoke info */
    uint32_t block_id;
};

struct pstream_read {
    pa_pstream_descriptor descriptor;
    pa_memblock *memblock;
    pa_packet *packet;
    uint32_t shm_info[PA_PSTREAM_SHM_MAX];
    void *data;
    size_t index;
};

struct pa_pstream {
    PA_REFCNT_DECLARE;

    pa_mainloop_api *mainloop;
    pa_defer_event *defer_event;
    pa_iochannel *io;
    pa_srbchannel *srb, *srbpending;
    bool is_srbpending;

    pa_queue *send_queue;

    bool dead;

    struct {
        union {
            uint8_t minibuf[MINIBUF_SIZE];
            pa_pstream_descriptor descriptor;
        };
        struct item_info* current;
        void *data;
        size_t index;
        int minibuf_validsize;
        pa_memchunk memchunk;
    } write;

    struct pstream_read readio, readsrb;

    bool use_shm;
    pa_memimport *import;
    pa_memexport *export;

    pa_pstream_packet_cb_t receive_packet_callback;
    void *receive_packet_callback_userdata;

    pa_pstream_memblock_cb_t receive_memblock_callback;
    void *receive_memblock_callback_userdata;

    pa_pstream_notify_cb_t drain_callback;
    void *drain_callback_userdata;

    pa_pstream_notify_cb_t die_callback;
    void *die_callback_userdata;

    pa_pstream_block_id_cb_t revoke_callback;
    void *revoke_callback_userdata;

    pa_pstream_block_id_cb_t release_callback;
    void *release_callback_userdata;

    pa_mempool *mempool;

#ifdef HAVE_CREDS
    pa_cmsg_ancil_data read_ancil_data, write_ancil_data;
    bool send_ancil_data_now;
#endif
};

static int do_write(pa_pstream *p);
static int do_read(pa_pstream *p, struct pstream_read *re);

static void do_pstream_read_write(pa_pstream *p) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    pa_pstream_ref(p);

    p->mainloop->defer_enable(p->defer_event, 0);

    if (!p->dead && p->srb) {
         do_write(p);
         while (!p->dead && do_read(p, &p->readsrb) == 0);
    }

    if (!p->dead && pa_iochannel_is_readable(p->io)) {
        if (do_read(p, &p->readio) < 0)
            goto fail;
    } else if (!p->dead && pa_iochannel_is_hungup(p->io))
        goto fail;

    while (!p->dead && pa_iochannel_is_writable(p->io)) {
        int r = do_write(p);
        if (r < 0)
            goto fail;
        if (r == 0)
            break;
    }

    pa_pstream_unref(p);
    return;

fail:

    if (p->die_callback)
        p->die_callback(p, p->die_callback_userdata);

    pa_pstream_unlink(p);
    pa_pstream_unref(p);
}

static bool srb_callback(pa_srbchannel *srb, void *userdata) {
    pa_pstream *p = userdata;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);
    pa_assert(p->srb == srb);

    do_pstream_read_write(p);
    if(p->srb != NULL)
        return true;
    else
        return false;
}

static void io_callback(pa_iochannel*io, void *userdata) {
    pa_pstream *p = userdata;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);
    pa_assert(p->io == io);

    do_pstream_read_write(p);
}

static void defer_callback(pa_mainloop_api *m, pa_defer_event *e, void*userdata) {
    pa_pstream *p = userdata;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);
    pa_assert(p->defer_event == e);
    pa_assert(p->mainloop == m);

    do_pstream_read_write(p);
}

static void memimport_release_cb(pa_memimport *i, uint32_t block_id, void *userdata);

pa_pstream *pa_pstream_new(pa_mainloop_api *m, pa_iochannel *io, pa_mempool *pool) {
    pa_pstream *p;

    pa_assert(m);
    pa_assert(io);
    pa_assert(pool);

    p = pa_xnew0(pa_pstream, 1);
    PA_REFCNT_INIT(p);
    p->io = io;
    pa_iochannel_set_callback(io, io_callback, p);

    p->mainloop = m;
    p->defer_event = m->defer_new(m, defer_callback, p);
    m->defer_enable(p->defer_event, 0);

    p->send_queue = pa_queue_new();

    p->mempool = pool;

    /* We do importing unconditionally */
    p->import = pa_memimport_new(p->mempool, memimport_release_cb, p);

    pa_iochannel_socket_set_rcvbuf(io, pa_mempool_block_size_max(p->mempool));
    pa_iochannel_socket_set_sndbuf(io, pa_mempool_block_size_max(p->mempool));

    return p;
}

static void item_free(void *item) {
    struct item_info *i = item;
    pa_assert(i);

    if (i->type == PA_PSTREAM_ITEM_MEMBLOCK) {
        pa_assert(i->chunk.memblock);
        pa_memblock_unref(i->chunk.memblock);
    } else if (i->type == PA_PSTREAM_ITEM_PACKET) {
        pa_assert(i->packet);
        pa_packet_unref(i->packet);
    }

    if (pa_flist_push(PA_STATIC_FLIST_GET(items), i) < 0)
        pa_xfree(i);
}

static void pstream_free(pa_pstream *p) {
    pa_assert(p);

    pa_pstream_unlink(p);

    pa_queue_free(p->send_queue, item_free);

    if (p->write.current)
        item_free(p->write.current);

    if (p->write.memchunk.memblock)
        pa_memblock_unref(p->write.memchunk.memblock);

    if (p->readsrb.memblock)
        pa_memblock_unref(p->readsrb.memblock);

    if (p->readsrb.packet)
        pa_packet_unref(p->readsrb.packet);

    if (p->readio.memblock)
        pa_memblock_unref(p->readio.memblock);

    if (p->readio.packet)
        pa_packet_unref(p->readio.packet);

    pa_xfree(p);
}

void pa_pstream_send_packet(pa_pstream*p, pa_packet *packet, const pa_cmsg_ancil_data *ancil_data) {
    struct item_info *i;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);
    pa_assert(packet);

    if (p->dead)
        return;

    if (!(i = pa_flist_pop(PA_STATIC_FLIST_GET(items))))
        i = pa_xnew(struct item_info, 1);

    i->type = PA_PSTREAM_ITEM_PACKET;
    i->packet = pa_packet_ref(packet);

#ifdef HAVE_CREDS
    if ((i->with_ancil_data = !!ancil_data)) {
        i->ancil_data = *ancil_data;
        if (ancil_data->creds_valid)
            pa_assert(ancil_data->nfd == 0);
        else
            pa_assert(ancil_data->nfd > 0);
    }
#endif

    pa_queue_push(p->send_queue, i);

    p->mainloop->defer_enable(p->defer_event, 1);
}

void pa_pstream_send_memblock(pa_pstream*p, uint32_t channel, int64_t offset, pa_seek_mode_t seek_mode,
                              const pa_memchunk *chunk, pa_sample_spec *ss) {

    size_t length, idx;
    size_t bsm, frame_size;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);
    pa_assert(channel != (uint32_t) -1);
    pa_assert(chunk);

    if (p->dead)
        return;

    idx = 0;
    length = chunk->length;

    bsm = PA_STREAM_BSM_MAX;
    frame_size = ss ? pa_frame_size(ss) : 0;
    if (frame_size)
        bsm -= bsm % frame_size;
    while (length > 0) {
        struct item_info *i;
        size_t n;

        if (!(i = pa_flist_pop(PA_STATIC_FLIST_GET(items))))
            i = pa_xnew(struct item_info, 1);
        i->type = PA_PSTREAM_ITEM_MEMBLOCK;

        n = PA_MIN(length, bsm);
        i->chunk.index = chunk->index + idx;
        i->chunk.length = n;
        i->chunk.memblock = pa_memblock_ref(chunk->memblock);

        i->channel = channel;
        i->offset = offset;
        i->seek_mode = seek_mode;
#ifdef HAVE_CREDS
        i->with_ancil_data = false;
#endif

        pa_queue_push(p->send_queue, i);

        idx += n;
        length -= n;
    }

    p->mainloop->defer_enable(p->defer_event, 1);
}

void pa_pstream_send_release(pa_pstream *p, uint32_t block_id) {
    struct item_info *item;
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    if (p->dead)
        return;

/*     pa_log("Releasing block %u", block_id); */

    if (!(item = pa_flist_pop(PA_STATIC_FLIST_GET(items))))
        item = pa_xnew(struct item_info, 1);
    item->type = PA_PSTREAM_ITEM_SHMRELEASE;
    item->block_id = block_id;
#ifdef HAVE_CREDS
    item->with_ancil_data = false;
#endif

    pa_queue_push(p->send_queue, item);
    p->mainloop->defer_enable(p->defer_event, 1);
}

/* might be called from thread context */
static void memimport_release_cb(pa_memimport *i, uint32_t block_id, void *userdata) {
    pa_pstream *p = userdata;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    if (p->dead)
        return;

    if (p->release_callback)
        p->release_callback(p, block_id, p->release_callback_userdata);
    else
        pa_pstream_send_release(p, block_id);
}

void pa_pstream_send_revoke(pa_pstream *p, uint32_t block_id) {
    struct item_info *item;
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    if (p->dead)
        return;
/*     pa_log("Revoking block %u", block_id); */

    if (!(item = pa_flist_pop(PA_STATIC_FLIST_GET(items))))
        item = pa_xnew(struct item_info, 1);
    item->type = PA_PSTREAM_ITEM_SHMREVOKE;
    item->block_id = block_id;
#ifdef HAVE_CREDS
    item->with_ancil_data = false;
#endif

    pa_queue_push(p->send_queue, item);
    p->mainloop->defer_enable(p->defer_event, 1);
}

/* might be called from thread context */
static void memexport_revoke_cb(pa_memexport *e, uint32_t block_id, void *userdata) {
    pa_pstream *p = userdata;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    if (p->revoke_callback)
        p->revoke_callback(p, block_id, p->revoke_callback_userdata);
    else
        pa_pstream_send_revoke(p, block_id);
}

static void prepare_next_write_item(pa_pstream *p) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    p->write.current = pa_queue_pop(p->send_queue);

    if (!p->write.current)
        return;
    p->write.index = 0;
    p->write.data = NULL;
    p->write.minibuf_validsize = 0;
    pa_memchunk_reset(&p->write.memchunk);

    p->write.descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH] = 0;
    p->write.descriptor[PA_PSTREAM_DESCRIPTOR_CHANNEL] = htonl((uint32_t) -1);
    p->write.descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI] = 0;
    p->write.descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_LO] = 0;
    p->write.descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS] = 0;

    if (p->write.current->type == PA_PSTREAM_ITEM_PACKET) {

        pa_assert(p->write.current->packet);
        p->write.data = p->write.current->packet->data;
        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH] = htonl((uint32_t) p->write.current->packet->length);

        if (p->write.current->packet->length <= MINIBUF_SIZE - PA_PSTREAM_DESCRIPTOR_SIZE) {
            memcpy(&p->write.minibuf[PA_PSTREAM_DESCRIPTOR_SIZE], p->write.data, p->write.current->packet->length);
            p->write.minibuf_validsize = PA_PSTREAM_DESCRIPTOR_SIZE + p->write.current->packet->length;
        }

    } else if (p->write.current->type == PA_PSTREAM_ITEM_SHMRELEASE) {

        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS] = htonl(PA_FLAG_SHMRELEASE);
        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI] = htonl(p->write.current->block_id);

    } else if (p->write.current->type == PA_PSTREAM_ITEM_SHMREVOKE) {

        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS] = htonl(PA_FLAG_SHMREVOKE);
        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI] = htonl(p->write.current->block_id);

    } else {
        uint32_t flags;
        bool send_payload = true;

        pa_assert(p->write.current->type == PA_PSTREAM_ITEM_MEMBLOCK);
        pa_assert(p->write.current->chunk.memblock);

        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_CHANNEL] = htonl(p->write.current->channel);
        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI] = htonl((uint32_t) (((uint64_t) p->write.current->offset) >> 32));
        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_LO] = htonl((uint32_t) ((uint64_t) p->write.current->offset));

        flags = (uint32_t) (p->write.current->seek_mode & PA_FLAG_SEEKMASK);

        if (p->use_shm) {
            uint32_t block_id, shm_id;
            size_t offset, length;
            uint32_t *shm_info = (uint32_t *) &p->write.minibuf[PA_PSTREAM_DESCRIPTOR_SIZE];
            size_t shm_size = sizeof(uint32_t) * PA_PSTREAM_SHM_MAX;
            pa_mempool *current_pool = pa_memblock_get_pool(p->write.current->chunk.memblock);
            pa_memexport *current_export;

            if (p->mempool == current_pool)
                pa_assert_se(current_export = p->export);
            else
                pa_assert_se(current_export = pa_memexport_new(current_pool, memexport_revoke_cb, p));

            if (pa_memexport_put(current_export,
                                 p->write.current->chunk.memblock,
                                 &block_id,
                                 &shm_id,
                                 &offset,
                                 &length) >= 0) {

                flags |= PA_FLAG_SHMDATA;
                if (pa_mempool_is_remote_writable(current_pool))
                    flags |= PA_FLAG_SHMWRITABLE;
                send_payload = false;

                shm_info[PA_PSTREAM_SHM_BLOCKID] = htonl(block_id);
                shm_info[PA_PSTREAM_SHM_SHMID] = htonl(shm_id);
                shm_info[PA_PSTREAM_SHM_INDEX] = htonl((uint32_t) (offset + p->write.current->chunk.index));
                shm_info[PA_PSTREAM_SHM_LENGTH] = htonl((uint32_t) p->write.current->chunk.length);

                p->write.descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH] = htonl(shm_size);
                p->write.minibuf_validsize = PA_PSTREAM_DESCRIPTOR_SIZE + shm_size;
            }
/*             else */
/*                 pa_log_warn("Failed to export memory block."); */

            if (current_export != p->export)
                pa_memexport_free(current_export);
        }

        if (send_payload) {
            p->write.descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH] = htonl((uint32_t) p->write.current->chunk.length);
            p->write.memchunk = p->write.current->chunk;
            pa_memblock_ref(p->write.memchunk.memblock);
        }

        p->write.descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS] = htonl(flags);
    }

#ifdef HAVE_CREDS
    if ((p->send_ancil_data_now = p->write.current->with_ancil_data))
        p->write_ancil_data = p->write.current->ancil_data;
#endif
}

static void check_srbpending(pa_pstream *p) {
    if (!p->is_srbpending)
        return;

    if (p->srb)
        pa_srbchannel_free(p->srb);

    p->srb = p->srbpending;
    p->is_srbpending = false;

    if (p->srb)
        pa_srbchannel_set_callback(p->srb, srb_callback, p);
}

static int do_write(pa_pstream *p) {
    void *d;
    size_t l;
    ssize_t r;
    pa_memblock *release_memblock = NULL;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    if (!p->write.current)
        prepare_next_write_item(p);

    if (!p->write.current) {
        /* The out queue is empty, so switching channels is safe */
        check_srbpending(p);
        return 0;
    }

    if (p->write.minibuf_validsize > 0) {
        d = p->write.minibuf + p->write.index;
        l = p->write.minibuf_validsize - p->write.index;
    } else if (p->write.index < PA_PSTREAM_DESCRIPTOR_SIZE) {
        d = (uint8_t*) p->write.descriptor + p->write.index;
        l = PA_PSTREAM_DESCRIPTOR_SIZE - p->write.index;
    } else {
        pa_assert(p->write.data || p->write.memchunk.memblock);

        if (p->write.data)
            d = p->write.data;
        else {
            d = pa_memblock_acquire_chunk(&p->write.memchunk);
            release_memblock = p->write.memchunk.memblock;
        }

        d = (uint8_t*) d + p->write.index - PA_PSTREAM_DESCRIPTOR_SIZE;
        l = ntohl(p->write.descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH]) - (p->write.index - PA_PSTREAM_DESCRIPTOR_SIZE);
    }

    pa_assert(l > 0);

#ifdef HAVE_CREDS
    if (p->send_ancil_data_now) {
        if (p->write_ancil_data.creds_valid) {
            pa_assert(p->write_ancil_data.nfd == 0);
            if ((r = pa_iochannel_write_with_creds(p->io, d, l, &p->write_ancil_data.creds)) < 0)
                goto fail;
        }
        else
            if ((r = pa_iochannel_write_with_fds(p->io, d, l, p->write_ancil_data.nfd, p->write_ancil_data.fds)) < 0)
                goto fail;
        p->send_ancil_data_now = false;
    } else
#endif
    if (p->srb)
        r = pa_srbchannel_write(p->srb, d, l);
    else if ((r = pa_iochannel_write(p->io, d, l)) < 0)
        goto fail;

    if (release_memblock)
        pa_memblock_release(release_memblock);

    p->write.index += (size_t) r;

    if (p->write.index >= PA_PSTREAM_DESCRIPTOR_SIZE + ntohl(p->write.descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH])) {
        pa_assert(p->write.current);
        item_free(p->write.current);
        p->write.current = NULL;

        if (p->write.memchunk.memblock)
            pa_memblock_unref(p->write.memchunk.memblock);

        pa_memchunk_reset(&p->write.memchunk);

        if (p->drain_callback && !pa_pstream_is_pending(p))
            p->drain_callback(p, p->drain_callback_userdata);
    }

    return (size_t) r == l ? 1 : 0;

fail:

    if (release_memblock)
        pa_memblock_release(release_memblock);

    return -1;
}

static int do_read(pa_pstream *p, struct pstream_read *re) {
    void *d;
    size_t l;
    ssize_t r;
    pa_memblock *release_memblock = NULL;
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    if (re->index < PA_PSTREAM_DESCRIPTOR_SIZE) {
        d = (uint8_t*) re->descriptor + re->index;
        l = PA_PSTREAM_DESCRIPTOR_SIZE - re->index;
    } else {
        pa_assert(re->data || re->memblock);

        if (re->data)
            d = re->data;
        else {
            d = pa_memblock_acquire(re->memblock);
            release_memblock = re->memblock;
        }

        d = (uint8_t*) d + re->index - PA_PSTREAM_DESCRIPTOR_SIZE;
        l = ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH]) - (re->index - PA_PSTREAM_DESCRIPTOR_SIZE);
    }

    if (re == &p->readsrb) {
        r = pa_srbchannel_read(p->srb, d, l);
        if (r == 0) {
            if (release_memblock)
                pa_memblock_release(release_memblock);
            return 1;
        }
    }
    else
#ifdef HAVE_CREDS
    {
        pa_cmsg_ancil_data b;

        if ((r = pa_iochannel_read_with_ancil_data(p->io, d, l, &b)) <= 0)
            goto fail;

        if (b.creds_valid) {
            p->read_ancil_data.creds_valid = true;
            p->read_ancil_data.creds = b.creds;
        }
        if (b.nfd > 0) {
            pa_assert(b.nfd <= MAX_ANCIL_DATA_FDS);
            p->read_ancil_data.nfd = b.nfd;
            memcpy(p->read_ancil_data.fds, b.fds, sizeof(int) * b.nfd);
        }
    }
#else
    if ((r = pa_iochannel_read(p->io, d, l)) <= 0)
        goto fail;
#endif

    if (release_memblock)
        pa_memblock_release(release_memblock);

    re->index += (size_t) r;

    if (re->index == PA_PSTREAM_DESCRIPTOR_SIZE) {
        uint32_t flags, length, channel;
        /* Reading of frame descriptor complete */

        flags = ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS]);

        if (!p->use_shm && (flags & PA_FLAG_SHMMASK) != 0) {
            pa_log_warn("Received SHM frame on a socket where SHM is disabled.");
            return -1;
        }

        if (flags == PA_FLAG_SHMRELEASE) {

            /* This is a SHM memblock release frame with no payload */

/*             pa_log("Got release frame for %u", ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI])); */

            pa_assert(p->export);
            pa_memexport_process_release(p->export, ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI]));

            goto frame_done;

        } else if (flags == PA_FLAG_SHMREVOKE) {

            /* This is a SHM memblock revoke frame with no payload */

/*             pa_log("Got revoke frame for %u", ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI])); */

            pa_assert(p->import);
            pa_memimport_process_revoke(p->import, ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI]));

            goto frame_done;
        }

        length = ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH]);

        if (length > FRAME_SIZE_MAX_ALLOW || length <= 0) {
            pa_log_warn("Received invalid frame size: %lu", (unsigned long) length);
            return -1;
        }

        pa_assert(!re->packet && !re->memblock);

        channel = ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_CHANNEL]);

        if (channel == (uint32_t) -1) {

            if (flags != 0) {
                pa_log_warn("Received packet frame with invalid flags value.");
                return -1;
            }

            /* Frame is a packet frame */
            re->packet = pa_packet_new(length);
            re->data = re->packet->data;

        } else {

            if ((flags & PA_FLAG_SEEKMASK) > PA_SEEK_RELATIVE_END) {
                pa_log_warn("Received memblock frame with invalid seek mode.");
                return -1;
            }

            if ((flags & PA_FLAG_SHMMASK) == PA_FLAG_SHMDATA) {

                if (length != sizeof(re->shm_info)) {
                    pa_log_warn("Received SHM memblock frame with invalid frame length.");
                    return -1;
                }

                /* Frame is a memblock frame referencing an SHM memblock */
                re->data = re->shm_info;

            } else if ((flags & PA_FLAG_SHMMASK) == 0) {

                /* Frame is a memblock frame */

                re->memblock = pa_memblock_new(p->mempool, length);
                re->data = NULL;
            } else {

                pa_log_warn("Received memblock frame with invalid flags value.");
                return -1;
            }
        }

    } else if (re->index > PA_PSTREAM_DESCRIPTOR_SIZE) {
        /* Frame payload available */

        if (re->memblock && p->receive_memblock_callback) {

            /* Is this memblock data? Than pass it to the user */
            l = (re->index - (size_t) r) < PA_PSTREAM_DESCRIPTOR_SIZE ? (size_t) (re->index - PA_PSTREAM_DESCRIPTOR_SIZE) : (size_t) r;

            if (l > 0) {
                pa_memchunk chunk;

                chunk.memblock = re->memblock;
                chunk.index = re->index - PA_PSTREAM_DESCRIPTOR_SIZE - l;
                chunk.length = l;

                if (p->receive_memblock_callback) {
                    int64_t offset;

                    offset = (int64_t) (
                            (((uint64_t) ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI])) << 32) |
                            (((uint64_t) ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_LO]))));

                    p->receive_memblock_callback(
                        p,
                        ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_CHANNEL]),
                        offset,
                        ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS]) & PA_FLAG_SEEKMASK,
                        &chunk,
                        p->receive_memblock_callback_userdata);
                }

                /* Drop seek info for following callbacks */
                re->descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS] =
                    re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI] =
                    re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_LO] = 0;
            }
        }

        /* Frame complete */
        if (re->index >= ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_LENGTH]) + PA_PSTREAM_DESCRIPTOR_SIZE) {

            if (re->memblock) {

                /* This was a memblock frame. We can unref the memblock now */
                pa_memblock_unref(re->memblock);

            } else if (re->packet) {

                if (p->receive_packet_callback)
#ifdef HAVE_CREDS
                    p->receive_packet_callback(p, re->packet, &p->read_ancil_data, p->receive_packet_callback_userdata);
#else
                    p->receive_packet_callback(p, re->packet, NULL, p->receive_packet_callback_userdata);
#endif

                pa_packet_unref(re->packet);
            } else {
                pa_memblock *b;
                uint32_t flags = ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS]);
                pa_assert((flags & PA_FLAG_SHMMASK) == PA_FLAG_SHMDATA);

                pa_assert(p->import);

                if (!(b = pa_memimport_get(p->import,
                                          ntohl(re->shm_info[PA_PSTREAM_SHM_BLOCKID]),
                                          ntohl(re->shm_info[PA_PSTREAM_SHM_SHMID]),
                                          ntohl(re->shm_info[PA_PSTREAM_SHM_INDEX]),
                                          ntohl(re->shm_info[PA_PSTREAM_SHM_LENGTH]),
                                          !!(flags & PA_FLAG_SHMWRITABLE)))) {

                    if (pa_log_ratelimit(PA_LOG_DEBUG))
                        pa_log_debug("Failed to import memory block.");
                }

                if (p->receive_memblock_callback) {
                    int64_t offset;
                    pa_memchunk chunk;

                    chunk.memblock = b;
                    chunk.index = 0;
                    chunk.length = b ? pa_memblock_get_length(b) : ntohl(re->shm_info[PA_PSTREAM_SHM_LENGTH]);

                    offset = (int64_t) (
                            (((uint64_t) ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_HI])) << 32) |
                            (((uint64_t) ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_OFFSET_LO]))));

                    p->receive_memblock_callback(
                            p,
                            ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_CHANNEL]),
                            offset,
                            ntohl(re->descriptor[PA_PSTREAM_DESCRIPTOR_FLAGS]) & PA_FLAG_SEEKMASK,
                            &chunk,
                            p->receive_memblock_callback_userdata);
                }

                if (b)
                    pa_memblock_unref(b);
            }

            goto frame_done;
        }
    }

    return 0;

frame_done:
    re->memblock = NULL;
    re->packet = NULL;
    re->index = 0;
    re->data = NULL;

#ifdef HAVE_CREDS
    p->read_ancil_data.creds_valid = false;
    p->read_ancil_data.nfd = 0;
#endif

    return 0;

fail:
    if (release_memblock)
        pa_memblock_release(release_memblock);

    return -1;
}

void pa_pstream_set_die_callback(pa_pstream *p, pa_pstream_notify_cb_t cb, void *userdata) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    p->die_callback = cb;
    p->die_callback_userdata = userdata;
}

void pa_pstream_set_drain_callback(pa_pstream *p, pa_pstream_notify_cb_t cb, void *userdata) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    p->drain_callback = cb;
    p->drain_callback_userdata = userdata;
}

void pa_pstream_set_receive_packet_callback(pa_pstream *p, pa_pstream_packet_cb_t cb, void *userdata) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    p->receive_packet_callback = cb;
    p->receive_packet_callback_userdata = userdata;
}

void pa_pstream_set_receive_memblock_callback(pa_pstream *p, pa_pstream_memblock_cb_t cb, void *userdata) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    p->receive_memblock_callback = cb;
    p->receive_memblock_callback_userdata = userdata;
}

void pa_pstream_set_release_callback(pa_pstream *p, pa_pstream_block_id_cb_t cb, void *userdata) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    p->release_callback = cb;
    p->release_callback_userdata = userdata;
}

void pa_pstream_set_revoke_callback(pa_pstream *p, pa_pstream_block_id_cb_t cb, void *userdata) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    p->release_callback = cb;
    p->release_callback_userdata = userdata;
}

bool pa_pstream_is_pending(pa_pstream *p) {
    bool b;

    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    if (p->dead)
        b = false;
    else
        b = p->write.current || !pa_queue_isempty(p->send_queue);

    return b;
}

void pa_pstream_unref(pa_pstream*p) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    if (PA_REFCNT_DEC(p) <= 0)
        pstream_free(p);
}

pa_pstream* pa_pstream_ref(pa_pstream*p) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    PA_REFCNT_INC(p);
    return p;
}

void pa_pstream_unlink(pa_pstream *p) {
    pa_assert(p);

    if (p->dead)
        return;

    p->dead = true;

    while (p->srb || p->is_srbpending) /* In theory there could be one active and one pending */
        pa_pstream_set_srbchannel(p, NULL);

    if (p->import) {
        pa_memimport_free(p->import);
        p->import = NULL;
    }

    if (p->export) {
        pa_memexport_free(p->export);
        p->export = NULL;
    }

    if (p->io) {
        pa_iochannel_free(p->io);
        p->io = NULL;
    }

    if (p->defer_event) {
        p->mainloop->defer_free(p->defer_event);
        p->defer_event = NULL;
    }

    p->die_callback = NULL;
    p->drain_callback = NULL;
    p->receive_packet_callback = NULL;
    p->receive_memblock_callback = NULL;
}

void pa_pstream_enable_shm(pa_pstream *p, bool enable) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    p->use_shm = enable;

    if (enable) {

        if (!p->export)
            p->export = pa_memexport_new(p->mempool, memexport_revoke_cb, p);

    } else {

        if (p->export) {
            pa_memexport_free(p->export);
            p->export = NULL;
        }
    }
}

bool pa_pstream_get_shm(pa_pstream *p) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0);

    return p->use_shm;
}

void pa_pstream_set_srbchannel(pa_pstream *p, pa_srbchannel *srb) {
    pa_assert(p);
    pa_assert(PA_REFCNT_VALUE(p) > 0 || srb == NULL);

    if (srb == p->srb)
        return;

    /* We can't handle quick switches between srbchannels. */
    pa_assert(!p->is_srbpending);

    p->srbpending = srb;
    p->is_srbpending = true;

    /* Switch immediately, if possible. */
    if (p->dead)
        check_srbpending(p);
    else
        do_write(p);
}
