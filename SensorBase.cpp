/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>

#include <linux/input.h>

#include <cutils/log.h>
#include "local_log_def.h"

#include "SensorBase.h"

/*****************************************************************************/

SensorBase::SensorBase(
        const char* dev_name,
        const char* data_name,
	int defer =0)
    : dev_name(dev_name), data_name(data_name),
      dev_fd(-1), data_fd(-1),data_fd2(-1)
{
	if (!defer) {
		if (data_name) {
			data_fd = openInput(data_name);
		}
	} else {
        	data_fd = openNull();
	}
}

SensorBase::~SensorBase() {
    if (data_fd >= 0) {
        close(data_fd);
    }
    if (dev_fd >= 0) {
        close(dev_fd);
    }
	
	if (data_fd2 >= 0) {
		close(data_fd2);
		data_fd2 = -1;
	}	
}

int SensorBase::open_device() {
    if (dev_fd<0 && dev_name) {
        dev_fd = open(dev_name, O_RDONLY);
        LOGE_IF(dev_fd<0, "Couldn't open %s (%s)", dev_name, strerror(errno));
    }
    return 0;
}

int SensorBase::close_device() {
    if (dev_fd >= 0) {
        close(dev_fd);
        dev_fd = -1;
    }
    return 0;
}

int SensorBase::getFd() const {
    if (!data_name) {
        return dev_fd;
    }
    return data_fd;
}
const char * SensorBase::getDevName() const {
    return input_name;
}

int SensorBase::setDelay(int32_t handle, int64_t ns) {
    return 0;
}

/**
 * as specified in hardware/sensors.h
     * When timeout is not 0:
     *   If successful, 0 is returned.
     *   If the specified sensor doesn't support batch mode, return retu.
     *   If the specified sensor's trigger-mode is one-shot, return -EINVAL.
     *   If WAKE_UPON_FIFO_FULL is specified and the specified sensor's internal
     *   FIFO is too small to store at least 10 seconds worth of data at the
     *   given rate, -EINVAL is returned. Note that as stated above, this has to
     *   be determined at compile time, and not based on the state of the
     *   system.
     *   If some other constraints above cannot be satisfied, return -EINVAL.
     *
     * When timeout is 0:
     *   The caller will never set the wake_upon_fifo_full flag.
     *   The function must succeed, and batch mode must be deactivated.
  */
int SensorBase::batch(int handle, int flags, int64_t period_ns, int64_t timeout) {
    if (timeout !=0) {
        return -EINVAL;
    }
    else {
        return 0;
    }
}

int SensorBase::flush(int32_t handle) {
    return 0;
}



bool SensorBase::hasPendingEvents() const {
    return false;
}

int64_t SensorBase::getTimestamp() {
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return int64_t(t.tv_sec)*1000000000LL + t.tv_nsec;
}

int SensorBase::openInput(const char *inputName, int oldfd)
{
	int fd;
	fd = SensorBase::openInput(inputName);
	if (fd < 0) return oldfd;
	if (data_fd2 >= 0) {
		close(data_fd2);
		data_fd2 = -1;
	}
	close(oldfd);
	dup2(fd, oldfd);
	close(fd);
	return oldfd;
}

int SensorBase::openNull()
{
	int pipefd[2];
	
	if (pipe(pipefd) > 0) {
		return -1;
	}
	if (data_fd2 >= 0) {
		close(data_fd2);
		data_fd2 = -1;
	}
	data_fd2 = pipefd[1];
	if (data_fd < 0) {	
		data_fd = pipefd[0];
	} else {
		int fd;
		int ret;
		fd = pipefd[0];
		close(data_fd);
		ret = dup2(fd, data_fd);
		close(fd);
	}
	return data_fd;
}

int SensorBase::openInput(const char* inputName) {
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;

    input_name[0] = '\0';
    dir = opendir(dirname);
    if(dir == NULL)
        return -1;
    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while((de = readdir(dir))) {
        if(de->d_name[0] == '.' &&
                (de->d_name[1] == '\0' ||
                        (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        strcpy(filename, de->d_name);
        fd = open(devname, O_RDONLY);
        if (fd>=0) {
            char name[80];
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
                name[0] = '\0';
            }
            if (!strcmp(name, inputName)) {
                strcpy(input_name, filename);
                break;
            } else {
                close(fd);
                fd = -1;
            }
        }
    }
    closedir(dir);
    LOGE_IF(fd<0, "couldn't find '%s' input device", inputName);
    return fd;
}
