/*
**
** Copyright 2008, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#ifndef _SHM_POSIX_
#define _SHM_POSIX_

#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */

#include <bits/posix_limits.h>
#if _POSIX_SHARED_MEMORY_OBJECTS == -1  /* -1 means shm_open()/shm_unlink() not implemented in bionic*/

#ifdef __cplusplus
extern "C" {
#endif

extern int shm_open(const char *path, int flags, mode_t mode);
extern int shm_unlink(const char *path);

#ifdef __cplusplus
}
#endif

#endif //_POSIX_SHARED_MEMORY_OBJECTS
#endif //_SHM_POSIX_
