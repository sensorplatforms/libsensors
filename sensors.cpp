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

#include "OSPInputSensor.h"

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
#define SENSORS_UNCALIBRATED_MAG        (1<<ID_UNCALIBRATED_MAG)
#define SENSORS_GAME_ROT_VEC            (1<<ID_GAME_ROT_VEC)
#define SENSORS_UNCALIBRATED_GYRO       (1<<ID_UNCALIBRATED_GYRO)
#define SENSORS_SIG_MOTION              (1<<ID_SIG_MOTION)
#define SENSORS_STEP_DETECTOR           (1<<ID_STEP_DETECTOR)
#define SENSORS_STEP_COUNTER            (1<<ID_STEP_COUNTER)
#define SENSORS_GEOMAGNETIC_ROT_VEC     (1<<ID_GEOMAGNETIC_ROT_VEC)

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

#define SENSORS_UNCALIBRATED_MAG_HANDLE       13
#define SENSORS_GAME_ROT_VEC_HANDLE           14
#define SENSORS_UNCALIBRATED_GYRO_HANDLE      15
#define SENSORS_SIG_MOTION_HANDLE             16
#define SENSORS_STEP_DETECTOR_HANDLE          17
#define SENSORS_STEP_COUNTER_HANDLE           18
#define SENSORS_GEOMAGNETIC_ROT_VEC_HANDLE    19


//For android:
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
    { "OSP magnetometer",
      "Sensor Platforms",
      1, SENSORS_MAGNETIC_FIELD_HANDLE,
      SENSOR_TYPE_MAGNETIC_FIELD, RANGE_M, RESOLUTION_M, .6f, 20000, 0, 0, { } },
    { "OSP Orientation",
      "Sensor Platform",
      1, SENSORS_ORIENTATION_HANDLE,
      SENSOR_TYPE_ORIENTATION, 360.0f, 0.00001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Rotation sensor",
      "Sensor Platforms",
      1, SENSORS_ROTATION_VECTOR_HANDLE,
      SENSOR_TYPE_ROTATION_VECTOR, 1.0f, 0.000000001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Linear Acceleration sensor",
      "Sensor Platforms",
      1, SENSORS_LIN_ACCELERATION_HANDLE,
      SENSOR_TYPE_LINEAR_ACCELERATION, RANGE_A, 0.00001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Gravity sensor",
      "Sensor Platforms",
      1, SENSORS_GRAVITY_HANDLE,
      SENSOR_TYPE_GRAVITY, RANGE_A, 0.00001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Uncalibrated Magnetometer",
      "Sensor Platforms",
      1, SENSORS_UNCALIBRATED_MAG_HANDLE,
      SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED, RANGE_M, RESOLUTION_M, .6f, 20000, 0, 0, { } },
    { "OSP Game Rot Vec",
      "Sensor Platforms",
      1, SENSORS_GAME_ROT_VEC_HANDLE,
      SENSOR_TYPE_GAME_ROTATION_VECTOR,  1.0f, 0.000000001f, 7.35f, 20000, 0, 0, { } },
    { "OSP Uncalibrated Gyroscope",
      "Sensor Platforms",
      1, SENSORS_UNCALIBRATED_GYRO_HANDLE,
      SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, RANGE_GYRO, RESOLUTION_GYRO, 6.5f, 20000, 0, 0, { } },
    { "OSP Significant Motion",
      "Sensor Platforms",
       1, SENSORS_SIG_MOTION_HANDLE,
       SENSOR_TYPE_SIGNIFICANT_MOTION, 1.0f, 1.0f, 1.1f, 0, 0, 0, {}},
    { "OSP Step Detector",
      "Sensor Platforms",
      1, SENSORS_STEP_DETECTOR_HANDLE,
      SENSOR_TYPE_STEP_DETECTOR, 1.0f, 1.0f, 1.1f, 0, 0, 0, {}},
    { "OSP Step Counter",
      "Sensor Platforms",
      1, SENSORS_STEP_COUNTER_HANDLE,
      SENSOR_TYPE_STEP_COUNTER, 1000.0f, 1.0f, 1.1f, 0, 0, 0, {}},
    { "OSP Geomagnetic Rot Vec",
      "Sensor Platforms",
      1, SENSORS_GEOMAGNETIC_ROT_VEC_HANDLE,
      SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR,  1.0f, 0.000000001f, 7.35f, 20000, 0, 0, { } },
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
        orientation,
        rot_vec,
        linear_accel,
        gravity,
        uncal_mag,
        game_rot_vec,
        uncal_gyro,
        sig_motion,
        step_detector,
        step_counter,
        geo_rot_vec,
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
        case ID_O:
            return orientation;

        case ID_GY:
            return gyro;

        case ID_RV:
            return rot_vec;

        case ID_LINACC:
            return linear_accel;

        case ID_GRAV:
            return gravity;

        case ID_UNCALIBRATED_MAG:
            return uncal_mag;

        case ID_GAME_ROT_VEC:
            return game_rot_vec;

        case ID_SIG_MOTION:
            return sig_motion;

        case ID_UNCALIBRATED_GYRO:
            return uncal_gyro;

        case ID_STEP_DETECTOR:
            return step_detector;

        case ID_STEP_COUNTER:
            return step_counter;

        case ID_GEOMAGNETIC_ROT_VEC:
            return geo_rot_vec;

        }
        return -EINVAL;
    }
};

sensors_poll_context_t::sensors_poll_context_t()
{
    char name[256];
    int v;
    int result;

    mSensors[accel] = new OSPInputSensor(ACCEL_UINPUT_NAME, ID_A,  SENSOR_TYPE_ACCELEROMETER, true);
    mPollFds[accel].fd = mSensors[accel]->getFd();
    mPollFds[accel].events = POLLIN;
    mPollFds[accel].revents = 0;

   if ((v = ioctl(mPollFds[accel].fd,EVIOCGPHYS(sizeof(name)),name)) >= 0) {
        sSensorList[accel].name = strdup(name);
   }


    mSensors[gyro] = new OSPInputSensor(GYRO_UINPUT_NAME, ID_GY,  SENSOR_TYPE_GYROSCOPE, true);
    mPollFds[gyro].fd = mSensors[gyro]->getFd();
    mPollFds[gyro].events = POLLIN;
    mPollFds[gyro].revents = 0;

    if (ioctl(mPollFds[gyro].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[gyro].name = strdup(name);
    }

    mSensors[mag] = new OSPInputSensor(MAG_UINPUT_NAME, ID_M,  SENSOR_TYPE_MAGNETIC_FIELD, true);
    mPollFds[mag].fd = mSensors[mag]->getFd();
    mPollFds[mag].events = POLLIN;
    mPollFds[mag].revents = 0;
    if (ioctl(mPollFds[mag].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[mag].name = strdup(name);
    }

    mSensors[rot_vec] = new OSPInputSensor("fm-rotation-vector", ID_RV,  SENSOR_TYPE_ROTATION_VECTOR, true);
    mPollFds[rot_vec].fd = mSensors[rot_vec]->getFd();
    mPollFds[rot_vec].events = POLLIN;
    mPollFds[rot_vec].revents = 0;
    if (ioctl(mPollFds[rot_vec].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[rot_vec].name = strdup(name);
    }

    mSensors[orientation] = new OSPInputSensor("fm-compass-orientation", ID_O,  SENSOR_TYPE_ORIENTATION, true);
    mPollFds[orientation].fd = mSensors[orientation]->getFd();
    mPollFds[orientation].events = POLLIN;
    mPollFds[orientation].revents = 0;
    if (ioctl(mPollFds[orientation].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[orientation].name = strdup(name);
    }

    mSensors[linear_accel] = new OSPInputSensor("fm-linear-acceleration", ID_LINACC,
                                                        SENSOR_TYPE_LINEAR_ACCELERATION, true);
    mPollFds[linear_accel].fd = mSensors[linear_accel]->getFd();
    mPollFds[linear_accel].events = POLLIN;
    mPollFds[linear_accel].revents = 0;
    if (ioctl(mPollFds[linear_accel].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[linear_accel].name = strdup(name);
    }

    mSensors[gravity] = new OSPInputSensor("fm-gravity", ID_GRAV, SENSOR_TYPE_GRAVITY, true);
    mPollFds[gravity].fd = mSensors[gravity]->getFd();
    mPollFds[gravity].events = POLLIN;
    mPollFds[gravity].revents = 0;
    if (ioctl(mPollFds[gravity].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[gravity].name = strdup(name);
    }

    mSensors[uncal_mag] = new OSPInputSensor("fm-uncalibrated-magnetometer",
                            ID_UNCALIBRATED_MAG,
                            SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED,
                            true);
    mPollFds[uncal_mag].fd = mSensors[uncal_mag]->getFd();
    mPollFds[uncal_mag].events = POLLIN;
    mPollFds[uncal_mag].revents = 0;
    if (ioctl(mPollFds[uncal_mag].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[uncal_mag].name = strdup(name);
    }

     mSensors[game_rot_vec] = new OSPInputSensor("fm-game-rotation-vector", 
				 			ID_GAME_ROT_VEC, 
				 			SENSOR_TYPE_GAME_ROTATION_VECTOR, 
				 			true);
     mPollFds[game_rot_vec].fd = mSensors[game_rot_vec]->getFd();
     mPollFds[game_rot_vec].events = POLLIN;
     mPollFds[game_rot_vec].revents = 0;
     if (ioctl(mPollFds[game_rot_vec].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
         sSensorList[game_rot_vec].name = strdup(name);
     }

    mSensors[uncal_gyro] = new OSPInputSensor("fm-uncalibrated-gyroscope", 
							ID_UNCALIBRATED_GYRO, 
							SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, 
							true);
    mPollFds[uncal_gyro].fd = mSensors[uncal_gyro]->getFd();
    mPollFds[uncal_gyro].events = POLLIN;
    mPollFds[uncal_gyro].revents = 0;
    if (ioctl(mPollFds[uncal_gyro].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[uncal_gyro].name = strdup(name);
    }

    mSensors[sig_motion] = new OSPInputSensor("fm-significant-motion",
                                               ID_SIG_MOTION,
                                               SENSOR_TYPE_SIGNIFICANT_MOTION,
					       FM_DECODE_VALUE_AS_INTEGER);
    mPollFds[sig_motion].fd = mSensors[sig_motion]->getFd();
    mPollFds[sig_motion].events = POLLIN;
    mPollFds[sig_motion].revents = 0;
    if (ioctl(mPollFds[sig_motion].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[sig_motion].name = strdup(name);
    }

    mSensors[step_detector] = new OSPInputSensor("fm-step-detector", 
							 ID_STEP_DETECTOR, 
							 SENSOR_TYPE_STEP_DETECTOR, 
							 FM_DECODE_VALUE_AS_INTEGER);

     mPollFds[step_detector].fd = mSensors[step_detector]->getFd();
     mPollFds[step_detector].events = POLLIN;
     mPollFds[step_detector].revents = 0;
     if (ioctl(mPollFds[step_detector].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
         sSensorList[step_detector].name = strdup(name);
     }

    mSensors[step_counter] = new OSPStepCounter("fm-step-counter", 
						       ID_STEP_COUNTER, 
						       SENSOR_TYPE_STEP_COUNTER, 
						       FM_DECODE_VALUE_AS_INTEGER);
    mPollFds[step_counter].fd = mSensors[step_counter]->getFd();
    mPollFds[step_counter].events = POLLIN;
    mPollFds[step_counter].revents = 0;
    if (ioctl(mPollFds[step_counter].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[step_counter].name = strdup(name);
    }

    mSensors[geo_rot_vec] = new OSPInputSensor("fm-geomagnetic-rotation-vector", 
							ID_GEOMAGNETIC_ROT_VEC, 
							SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR, 
							true);
    mPollFds[geo_rot_vec].fd = mSensors[geo_rot_vec]->getFd();
    mPollFds[geo_rot_vec].events = POLLIN;
    mPollFds[geo_rot_vec].revents = 0;
    if (ioctl(mPollFds[geo_rot_vec].fd,EVIOCGPHYS(sizeof(name)),name) >= 0) {
        sSensorList[geo_rot_vec].name = strdup(name);
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



