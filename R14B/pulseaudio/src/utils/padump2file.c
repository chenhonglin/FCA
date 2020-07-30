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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <getopt.h>
#include <sys/stat.h>
#include <pulse/audiocontrol.h>
#include <pulse/xmalloc.h>
#include <pulsecore/core-util.h>

#define DUMP_TIME_DEFAULT (30)
#define DUMP_PATH_SIZE 256

static void dump_status_callback(void *userdata, unsigned char result)
{
    struct pa_ac_simple *mPaSimple =  (struct pa_ac_simple *)userdata;
    mPaSimple->operation_success = result;

    if(result == PA_DUMP_OK){
        printf("###Dump start:\r\n");
    }else if(result == PA_DUMP_ERR_ACCESS){
        printf("###Dump  file open failed\n");
    }else if(result == PA_DUMP_ERR_BUSY){
        printf("###Dump  is already runing\n");
    }else{
        printf("###Dump unknown error\n");
    }
}

static void help(void) {
    printf("padump2file [options]\n\n"
             "  -h,   Show this help\n"
             "  -t,   How long(second) to dump. Default to 30s if it is not defined\n"
             " dump_file_path, for example: /mnt/pulseaudio/padump\n");
}

int main(int argc, char *argv[])
{
    int opt = 0;
    int c = 0;
    struct pa_ac_simple *s = NULL;
    int dump_time = DUMP_TIME_DEFAULT;
    int cnt;
    char *dump_path = NULL;
    struct tm *ptm;
    long ts;
    t_dump_params dump_params;

    printf("--------------------------------------------------\n");

    while ((c = getopt_long(argc, argv, "t:p:h", NULL, NULL)) != -1){
        switch (c) {
            case 'h':
                help();
                return 0;
            case 't':
                dump_time = strtoul((char*)optarg, 0, 0);
                if(dump_time <= 0)
                    dump_time = DUMP_TIME_DEFAULT;
                break;
            default:
                printf("invalid option");
                return -1;
        }
    }

    //parse dump path
    while (optind <= argc - 1) {
        dump_path = pa_xstrdup(argv[optind++]);
    }

    printf("Connecting to pulseaudio...");
    s = pa_ac_create_connection();
    if(!s){
        printf("pa_create_connection failed, exit\n");
        return -1;
    }
    printf(" OK\n");

    ts = time(NULL);
    ptm = localtime(&ts);

    if(!dump_path)
    {
        dump_path = pa_xmalloc(DUMP_PATH_SIZE);
        sprintf(dump_path, DUMP_HOME "/" "dump-%02d%02d-%02d%02d%02d-%ds/",
            ptm->tm_mon+1,ptm->tm_mday,ptm->tm_hour,ptm->tm_min,ptm->tm_sec, dump_time);
    }
    pa_strlcpy(dump_params.path, dump_path, sizeof(dump_params.path));
    dump_params.time = dump_time;

    printf("Sending dump request %ds\n", dump_params.time);
    s->operation_success =PA_DUMP_ERR_INIT;
    pa_ac_set_dump(s, &dump_params, (dump_status_cb_t)dump_status_callback);

    /*wait for msg send complete*/
    while(s->operation_success == PA_DUMP_ERR_INIT) {
        sleep(1);
    }
    if(s->operation_success == PA_DUMP_OK)
    {
        printf("Path: %s\n", dump_params.path);
        dump_time+=4;
        for(cnt=0;cnt<dump_time;cnt++)
        {
            sleep(1);
            printf("###Dumpping : %02d%%\r",cnt*100/dump_time);
            fflush(stdout);
        }
        printf("###Dump finished.  Path: %s\r\n", dump_params.path);
    }
    printf("--------------------------------------------------\n");

    pa_ac_disconnect(s);
    pa_xfree(dump_path);

    return 0;
}

