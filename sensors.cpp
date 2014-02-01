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

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>

#include <linux/input.h>

#include <utils/Atomic.h>
#include <cutils/log.h>
#include "local_log_def.h"

#include "sensors.h"

#include "SensorHubInputSensor.h"

/*****************************************************************************/


#define DELAY_OUT_TIME                  0x7FFFFFFF

#define LIGHT_SENSOR_POLLTIME           2000000000

#define SENSORS_ACCELERATION            (1<<ID_A)
#define SENSORS_ACCELERATION2           (1<<ID_A2)
#define SENSORS_MAGNETIC_FIELD          (1<<ID_M)
#define SENSORS_ORIENTATION             (1<<ID_O)
#define SENSORS_LIGHT                   (1<<ID_L)
#define SENSORS_PROXIMITY               (1<<ID_P)
#define SENSORS_GYROSCOPE               (1<<ID_GY)
#define SENSORS_PRESSURE                (1<<ID_PRESS)
#define SENSORS_TEMPERATURE             (1<<ID_TEMP)
#define SENSORS_ROTATION                (1<<ID_RV)
#define SENSORS_LINEAR_ACCELERATION     (1<<ID_LINACC)
#define SENSORS_GRAVITY                 (1<<ID_GRAV)

#define SENSORS_ACCELERATION_HANDLE     0
#define SENSORS_MAGNETIC_FIELD_HANDLE   1
#define SENSORS_ORIENTATION_HANDLE      2
#define SENSORS_LIGHT_HANDLE            3
#define SENSORS_PROXIMITY_HANDLE        4
#define SENSORS_GYROSCOPE_HANDLE        5
#define SENSORS_PRESSURE_HANDLE         6
#define SENSORS_TEMPERATURE_HANDLE      7
#define SENSORS_ACCELERATION_2_HANDLE   8
#define SENSORS_DEBUG_TESTING_HANDLE    9
#define SENSORS_ROTATION_VECTOR_HANDLE  10
#define SENSORS_LIN_ACCELERATION_HANDLE 11
#define SENSORS_GRAVITY_HANDLE          12

#ifndef UNIX_PATH_MAX
# define UNIX_PATH_MAX 108
#endif

/*****************************************************************************/
struct uinput_user_dev;

/* The SENSORS Module */
static struct sensor_t sSensorList[] = {
    { "OSP Accelerometer",
      "Sensor Platforms",
      1, SENSORS_ACCELERATION_HANDLE,
      SENSOR_TYPE_ACCELEROMETER, RANGE_A, RESOLUTION_A, 0.25f, 20000, 0, 0, { } },
    { "OSP Gyroscope",
      "Sensor Platforms",
      1, SENSORS_GYROSCOPE_HANDLE,
      SENSOR_TYPE_GYROSCOPE, RANGE_GYRO, RESOLUTION_GYRO, 6.5f, 20000, 0, 0, { } },
    { "OSP Magnetometer",
      "Sensor Platforms",
      1, SENSORS_MAGNETIC_FIELD_HANDLE,
      SENSOR_TYPE_MAGNETIC_FIELD, RANGE_M, RESOLUTION_M, .6f, 20000, 0, 0, { } },
};


static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device);


static int sensors__get_sensors_list(struct sensors_module_t* module,
                                     struct sensor_t const** list)
{
    *list = sSensorList;
    return ARRAY_SIZE(sSensorList);
}

static struct hw_module_methods_t sensors_module_methods = {
        open: open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    common: {
        tag: HARDWARE_MODULE_TAG,
        version_major: 1,
        version_minor: 0,
        id: SENSORS_HARDWARE_MODULE_ID,
        name: "Sensor module",
        author: "Sensor Platforms Inc.",
        methods: &sensors_module_methods,
        dso: 0,
        reserved: {0}
    },
    get_sensors_list: sensors__get_sensors_list,
};

struct sensors_poll_context_t {
    struct sensors_poll_device_1 device; // must be first

        sensors_poll_context_t();
        ~sensors_poll_context_t();
    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t delay_ns);
    int pollEvents(sensors_event_t* data, int count);
    int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    int flush(int handle);

private:
    enum {
        accel,
        gyro,
        mag,
        numSensorDrivers,
        numFds,
    };

    enum {
        eCmdEnable,
        eCmdDisable,
        eCmdSetDelay
    };

    static const size_t wake = numFds - 1;
    static const char WAKE_MESSAGE = 'W';
    struct pollfd mPollFds[numFds];
    int mWritePipeFd;
    SensorBase* mSensors[numSensorDrivers];

    int handleToDriver(int handle) const {
        switch (handle) {
        case ID_A:
            return accel;

        case ID_M:
            return mag;

        case ID_GY:
            return gyro;

        }
        return -EINVAL;
    }
};

sensors_poll_context_t::sensors_poll_context_t()
{
    char name[256];
    int v;
    int result;

    mSensors[accel] = new SensorHubInputSensor("sensorplatforms-accel", 
    										   ID_A,  
    										   SENSOR_TYPE_ACCELEROMETER, 
    										   .001f, .001f, .001f, 
    										   0, 1, 2);
    mPollFds[accel].fd = mSensors[accel]->getFd();
    mPollFds[accel].events = POLLIN;
    mPollFds[accel].revents = 0;

   if ((v = ioctl(mPollFds[accel].fd,EVIOCGPHYS(sizeof(name)),name)) >= 0) {
        sSensorList[accel].name = strdup(name);
   }


    mSensors[gyro] = new SensorHubInputSensor("sensorplatforms-gyro", 
    										  ID_GY,
    										  SENSOR_TYPE_GYROSCOPE, 
    										  .001, .001, .001, 
    										  0, 1, 2);
    mPollFds[gyro].fd = mSensors[gyro]->getFd();
    mPollFds[gyro].events = POLLIN;
    mPollFds[gyro].revents = 0;

    if (ioctl(mPollFds[gyro].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[gyro].name = strdup(name);
    }

    mSensors[mag] = new SensorHubInputSensor(MAG_UINPUT_NAME, 
    										 ID_M,  
    										 SENSOR_TYPE_MAGNETIC_FIELD, 
    										 .001f, .001f, .001f, 
    										 0, 1, 2);
    mPollFds[mag].fd = mSensors[mag]->getFd();
    mPollFds[mag].events = POLLIN;
    mPollFds[mag].revents = 0;
    if (ioctl(mPollFds[mag].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[mag].name = strdup(name);
    }

    int wakeFds[2];
    result = pipe(wakeFds);
    LOGE_IF(result<0, "error creating wake pipe (%s)", strerror(errno));
    fcntl(wakeFds[0], F_SETFL, O_NONBLOCK); //@@ read should probably be blocking but since
    fcntl(wakeFds[1], F_SETFL, O_NONBLOCK); //we are using 'poll' it doesn't matter
    mWritePipeFd = wakeFds[1];

    mPollFds[wake].fd = wakeFds[0];
    mPollFds[wake].events = POLLIN;
    mPollFds[wake].revents = 0;
}

sensors_poll_context_t::~sensors_poll_context_t() {
    for (int i=0 ; i<numSensorDrivers ; i++) {
        delete mSensors[i];
    }
    close(mPollFds[wake].fd);
    close(mWritePipeFd);
}

int sensors_poll_context_t::activate(int handle, int enabled) {

    int index = handleToDriver(handle);
    if (index < 0) return index;

    int err =  mSensors[index]->enable(handle, enabled);
    LOGE_IF(err != 0, "Sensor-activate failed (%d)", err);
    if (enabled && !err) {
        const char wakeMessage(WAKE_MESSAGE);
        int result = write(mWritePipeFd, &wakeMessage, 1);
        LOGE_IF(result<0, "error sending wake message (%s)", strerror(errno));
    }
    return err;
}

int sensors_poll_context_t::setDelay(int handle, int64_t delay_ns) {

    int index = handleToDriver(handle);
    if (index < 0) return index;

    int err= mSensors[index]->setDelay(handle, delay_ns);
    LOGE_IF(err < 0, "set delay failed (%d)", err);

    return err;
}

int sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
    int nbEvents = 0;
    int n = 0;

    do {
        // see if we have some leftover from the last poll()
        for (int i=0 ; count && i<numSensorDrivers ; i++) {
            SensorBase* const sensor(mSensors[i]);
            if ((mPollFds[i].revents & POLLIN) || (sensor->hasPendingEvents())) {
                int nb = sensor->readEvents(data, count);
                if (nb < count) {
                    // no more data for this sensor
                    mPollFds[i].revents = 0;
                }
		if (nb >=0) {
                    count -= nb;
                    nbEvents += nb;
                    data += nb;
                }
            }
        }

        if (count) {
            // we still have some room, so try to see if we can get
            // some events immediately or just wait if we don't have
            // anything to return
            n = poll(mPollFds, numFds, nbEvents ? 0 : -1);
            if (n<0) {
                LOGE("poll() failed (%s)", strerror(errno));
                return -errno;
            }
            if (mPollFds[wake].revents & POLLIN) {
                char msg;
                int result = read(mPollFds[wake].fd, &msg, 1);
                LOGE_IF(result<0, "error reading from wake pipe (%s)", strerror(errno));
                LOGE_IF(msg != WAKE_MESSAGE, "unknown message on wake queue (0x%02x)", int(msg));
                mPollFds[wake].revents = 0;
            }
        }
        // if we have events and space, go read them
    } while (n && count);

    return nbEvents;
}

int sensors_poll_context_t::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{

    int index = handleToDriver(handle);
    if (index < 0) return index;

    int err= mSensors[index]->batch(handle, flags, period_ns, timeout);
    LOGE_IF(err < 0, "batch failed (%d)", err);

    return err;
}

int sensors_poll_context_t::flush(int handle) {

    int index = handleToDriver(handle);
    if (index < 0) return index;

    int err= mSensors[index]->flush(handle);
    LOGE_IF(err < 0, "flush failed (%d)", err);

    return err;
}


/*****************************************************************************/

static int poll__close(struct hw_device_t *dev)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    if (ctx) {
        delete ctx;
    }
    return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
        int handle, int enabled) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->activate(handle, enabled);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
        int handle, int64_t ns)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->setDelay(handle, ns);
}

static int poll__poll(struct sensors_poll_device_t *dev,
        sensors_event_t* data, int count)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->pollEvents(data, count);
}

static int poll__batch(struct sensors_poll_device_1 *dev,
                      int handle, int flags, int64_t period_ns, int64_t timeout)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->batch(handle, flags, period_ns, timeout);
}

static int poll__flush(struct sensors_poll_device_1* dev, int handle)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->flush(handle);
}

/*****************************************************************************/

/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device)
{
    int status = -EINVAL;
    sensors_poll_context_t *dev = new sensors_poll_context_t();

    memset(&dev->device, 0, sizeof(sensors_poll_device_t));

    dev->device.common.tag = HARDWARE_DEVICE_TAG;
    dev->device.common.version  = SENSORS_DEVICE_API_VERSION_1_0;
    dev->device.common.module   = const_cast<hw_module_t*>(module);
    dev->device.common.close    = poll__close;
    dev->device.activate        = poll__activate;
    dev->device.setDelay        = poll__setDelay;
    dev->device.poll            = poll__poll;
    dev->device.batch           = poll__batch;
    dev->device.flush           = poll__flush;

    *device = &dev->device.common;
    status = 0;

    return status;
}



