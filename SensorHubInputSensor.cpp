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
#include <cutils/log.h>
#include <linux/input.h>

#include "SensorHubInputSensor.h"
#include "local_log_def.h"

SensorHubInputSensor::SensorHubInputSensor(const char* Name, int32_t sensorId, int32_t sensorType, float s_x, float s_y, float s_z, int x, int y, int z)
    : SensorBase(NULL, Name, 1),
      mInputReader(NULL),
      mHasPendingEvent(false),
      Name(Name),
      mEnabled(0)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = sensorId;
    mPendingEvent.type = sensorType;
    scale_x = s_x;
    scale_y = s_y;
    scale_z = s_z;
    read_x = x;
    read_y = y;
    read_z = z;

    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

SensorHubInputSensor::~SensorHubInputSensor() {
    if (mEnabled) {
        enable(0, 0);
    }
}

int SensorHubInputSensor::setInitialState() {
    struct input_absinfo absinfo_x;
    struct input_absinfo absinfo_y;
    struct input_absinfo absinfo_z;

    if (!ioctl(data_fd, EVIOCGABS(FREEMOTIOND_EVENT_X), &absinfo_x) &&
            !ioctl(data_fd, EVIOCGABS(FREEMOTIOND_EVENT_Y), &absinfo_y) &&
            !ioctl(data_fd, EVIOCGABS(FREEMOTIOND_EVENT_Z), &absinfo_z)) {
        mPendingEvent.data[read_x] = absinfo_x.value*scale_x;
        mPendingEvent.data[read_y] = absinfo_y.value*scale_y;
        mPendingEvent.data[read_z] = absinfo_z.value*scale_z;
        mHasPendingEvent = true;
    }
    return 0;
}

int SensorHubInputSensor::enable(int32_t a, int en) {
    int flags = en ? 1 : 0;

    if (flags != mEnabled) {
        mEnabled = flags;
        if (mEnabled) {
            mInputReader = new InputEventCircularReader(4),
                    openInput(Name,getFd());
            setInitialState();
        } else {
            openNull();
            delete mInputReader;
        }
    }
    return 0;
}

bool SensorHubInputSensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int SensorHubInputSensor::setDelay(int32_t handle, int64_t delay_ns)
{
    return 0;
}

void SensorHubInputSensor::handleEvent(sensors_event_t* handledEvent, input_event const* incomingEvent)
{
    switch(incomingEvent->code) {
    case ABS_X:
        handledEvent->data[read_x] = (incomingEvent->value*scale_x);
        break;

    case ABS_Y:
        handledEvent->data[read_y] = (incomingEvent->value*scale_y);
        break;

    case ABS_Z:
        handledEvent->data[read_z] = (incomingEvent->value*scale_z);
        break;

    case ABS_MISC:
        //ABS_MISC always comes before EV_SYN and has the lower 32-bit of the 64-bit timestamp
        handledEvent->timestamp = (int64_t)((uint64_t)incomingEvent->value & 0xFFFFFFFF);
        break;

    default:
        LOGE("unknown input event code %d", incomingEvent->code);
        break;
    }


}

void SensorHubOrientationSensor::handleEvent(sensors_event_t* handledEvent, input_event const* incomingEvent)  {

    //use standard scale and swap handling to get units and yaw, pitch, roll converted
    SensorHubInputSensor::handleEvent(handledEvent, incomingEvent);


    //then convert from Win8 couter-clockwise conventions to Android clockwise conventions
#define INCOMING_YAW_INDEX   (2)
#define INCOMING_PITCH_INDEX (0)
#define INCOMING_ROLL_INDEX  (1)
    if (incomingEvent->code == INCOMING_YAW_INDEX) {
        handledEvent->data[ABS_X]= 360 - handledEvent->data[ABS_X];

    } else if (incomingEvent->code == INCOMING_PITCH_INDEX) {
        handledEvent->data[ABS_Y]= -handledEvent->data[ABS_Y];

    } else if (incomingEvent->code == INCOMING_ROLL_INDEX) {
        handledEvent->data[ABS_Z]= -handledEvent->data[ABS_Z];

    }
    else {
        ; //ignore unknown codes
    }


}

int SensorHubInputSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;
    if (mEnabled == 0) {
#if 1
        int r;
        struct input_event event;
        long flags;
        int saved_errno;
        pid_t tid;

        tid = gettid();

        flags = fcntl(data_fd, F_GETFL);
        fcntl(data_fd, F_SETFL, (flags | O_NONBLOCK));
        r = read(data_fd, &event, sizeof(struct input_event));
        saved_errno = errno;
        fcntl(data_fd, F_SETFL, (flags));
        if (r < 0) {
            openNull();
            mHasPendingEvent = false;
        }
#endif
        return 0;
    }

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = 0; //! \TODO initial events need a sensorhub based (or synced) timestamp, but zero is a decent placeholder
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader->fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

again:
    while (count && mInputReader->readEvent(&event)) {

        if ((event->type == EV_REL) || (event->type == EV_ABS)) {
            handleEvent(&mPendingEvent, event);
        } else if (event->type == EV_SYN) {
            /* Upper 32-bits of the timestamp comes in the EV_SYN value */
            mPendingEvent.timestamp |= (int64_t)event->value << 32;
            if (mEnabled) {
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
            }
        } else {
            LOGE("SensorHub: unknown event (type=%d, code=%d)",
                 event->type, event->code);
        }
        mInputReader->next();
    }

    /* if we didn't read a complete event, see if we can fill and
       try again instead of returning with nothing and redoing poll. */
    if (numEventReceived == 0 && mEnabled == 1) {
        n = mInputReader->fill(data_fd);
        if (n)
            goto again;
    }

    return numEventReceived;
}

