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
#include <strings.h>
#include <sys/select.h>
#include <cutils/log.h>
#include "local_log_def.h"

#include "OSPInputSensor.h"

OSPInputSensor::OSPInputSensor(const char* uinputName,
				int32_t sensorId,
				int32_t sensorType,
				bool evtFloat) :
	 SensorBase(NULL, uinputName, 1),    
	mEnabled(false),
	mEventsAreFloat(evtFloat),
	mpInputReader(NULL),
	mHasPendingEvent(false),
	uinputName(uinputName),
	SensorType(sensorType)
{
	if (sensorType == SENSOR_TYPE_MAGNETIC_FIELD || 
		sensorType == SENSOR_TYPE_ORIENTATION ||
		sensorType == SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
		Qscale = (1<<12);
	} else if (sensorType == SENSOR_TYPE_STEP_DETECTOR ||
		sensorType == SENSOR_TYPE_SIGNIFICANT_MOTION) {
		Qscale = 1;
    	} else {
		Qscale = (1<<24);
	}
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = sensorId;
	mPendingEvent.type = sensorType;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
	mpReadAvail = false;
	ctrlPath[0] = '\0';
}

OSPInputSensor::~OSPInputSensor()
{
	if (mEnabled) {
		enable(0, 0);
	}
}

int OSPInputSensor::setInitialState()
{
#if 0
    struct input_absinfo absinfo_x;
    struct input_absinfo absinfo_y;
    struct input_absinfo absinfo_z;
    union {
        int iVal;
        float fVal;
    } evtVal;

#if 0
    if (!ioctl(data_fd, EVIOCGABS(FREEMOTIOND_EVENT_X), &absinfo_x) &&
        !ioctl(data_fd, EVIOCGABS(FREEMOTIOND_EVENT_Y), &absinfo_y) &&
        !ioctl(data_fd, EVIOCGABS(FREEMOTIOND_EVENT_Z), &absinfo_z)) {
        evtVal.iVal = absinfo_x.value;
        mPendingEvent.data[0] = mEventsAreFloat? evtVal.fVal : evtVal.iVal;
        evtVal.iVal = absinfo_y.value;
        mPendingEvent.data[1] = mEventsAreFloat? evtVal.fVal : evtVal.iVal;
        evtVal.iVal = absinfo_z.value;
        mPendingEvent.data[2] = mEventsAreFloat? evtVal.fVal : evtVal.iVal;
        mHasPendingEvent = true;
    }
#endif
#endif
	return 0;
}

static void write_enable(char *p, int val)
{
	int fd;
	int ret;
	char towrite[4];

	fd = open(p, O_WRONLY);
	if (fd < 0) {
		LOGE("HY-DBG: open %s failed errno = %i", p, errno);
		return;
	}
	towrite[1] = '\n';
	towrite[2] = '\n';
	if (val) {
		towrite[0] = '1';	
	} else {
		towrite[0] = '0';
	}

	ret = write(fd, towrite, 3);
	if (ret < 0)
		LOGE("HY-DBG: write %s returned %i errno = %i", p, ret, errno);
	close(fd);
}

int OSPInputSensor::enable(int32_t handle, int enabled)
{
	bool flags = enabled ? true : false;
	char enablePath[512];
	const char *ev;

	LOGE("@@@@ enable: [%d] - %d, %d", handle, enabled, mEnabled);
	if (flags && flags != mEnabled) {
		mpInputReader = new InputEventCircularReader(4),
		openInput(uinputName,getFd());
		mEnabled = flags;
		setInitialState();
		mpReadAvail = true;

		enablePath[511] = '\0';
		ev = getDevName();
		if (ev) {
			snprintf(enablePath, 511, "/sys/class/input/%s/device/enable_new", ev);
		} else {
			snprintf(enablePath, 511, "/dev/null");
		}
		LOGE("@@@@ HY-DBG: enable-path %s uinputName %s", enablePath, getDevName());
		write_enable(enablePath, 1);
		strcpy(ctrlPath, enablePath);
		snprintf(enablePath, 511, "/dev/OSPControl/%s", uinputName);
		{
			int fd;
	
			fd = creat(enablePath, 0777);
			LOGE("@@@@ HY-DBG: enable-path %s fd = %i", enablePath, fd);
			close(fd);
		}
	} else if (!flags) {
		enablePath[511] = '\0';
		ev = getDevName();
		if (ev) {
			snprintf(enablePath, 511, "/sys/class/input/%s/device/enable_new", ev);
		} else {
			snprintf(enablePath, 511, "/dev/null");
		}
		write_enable(enablePath, 0);
		LOGE("@@@@ HY-DBG: enable-path %s uinputName %s", enablePath, getDevName());
		openNull();
		snprintf(enablePath, 511, "/dev/OSPControl/%s", uinputName);
		unlink(enablePath);
#if 1
		if (mpReadAvail) {
			delete mpInputReader;
			mpReadAvail = false;
		}
#endif
		mEnabled = flags;
	}
	LOGE("@@@@ enable: [%d] - %d", handle, enabled);

	return 0;
}

bool OSPInputSensor::handleEvent(input_event const * event,
				sensors_event_t* androidData)
{
	return false;
}

//! use the handling to generate the necessary sideeffect of making SIG_MOTION a oneshot
bool OSPSignificantMotion::handleEvent(input_event const * event,
				sensors_event_t* androidData) 
{
	LOGE("@@@@ custom handling of SIG_MOTION");

	if (event->code == REL_X) {
		LOGE("@@@@ auto-disable SIG_MOTION: [%d]", androidData->sensor);
	}
	androidData->data[0] = 1.0;

	return false;
}

bool OSPInputSensor::hasPendingEvents() const
{
	return mHasPendingEvent;
}

int OSPInputSensor::setDelay(int32_t handle, int64_t delay_ns)
{
	LOGI("@@@@ setDelay: [%d] - %lldms", handle, (delay_ns/1000000));
	return 0;
}
bool OSPStepCounter::handleEvent(input_event const * event, sensors_event_t* androidData) {
	if (event->code == FREEMOTIOND_EVENT_X) {
		androidData->u64.step_counter= event->value;
	}

	return true;
}
#undef USE_EMBEDDED_TS
int OSPInputSensor::readEvents(sensors_event_t* data, int count)
{
	if (count < 1)
		return -EINVAL;

	if (mHasPendingEvent) {
		mHasPendingEvent = false;
		mPendingEvent.timestamp = getTimestamp();
		*data = mPendingEvent;
		return mEnabled ? 1 : 0;
	}

	ssize_t n = mpInputReader->fill(data_fd); //@@ causes read() of input_event
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const* event;

again:
	while (count && mpInputReader->readEvent(&event)) {
		int type = event->type;
#if 0
        union {
            int iVal;
            float fVal;
        } evtVal;
#else
		float fVal;
#endif

		if ((type == EV_REL) || (type == EV_ABS)) {
			//Check for custom handlers first
			if (!handleEvent(event, &mPendingEvent)) {
#if 0
				evtVal.iVal = event->value;
				if (event->code == FREEMOTIOND_EVENT_X) {
					mPendingEvent.data[0] = mEventsAreFloat? evtVal.fVal : evtVal.iVal;
				} else if (event->code == FREEMOTIOND_EVENT_Y) {
					mPendingEvent.data[1] = mEventsAreFloat? evtVal.fVal : evtVal.iVal;
		                } else if (event->code == FREEMOTIOND_EVENT_Z) {
					mPendingEvent.data[2] = mEventsAreFloat? evtVal.fVal : evtVal.iVal;
				} else {
					//Unhandled event
					//LOGE("%s: Unhandled event code - %d", __FUNCTION__, event->code);
				}
#else
				fVal = (float)event->value/(float)Qscale;
				if (event->code == FREEMOTIOND_EVENT_X) {
					mPendingEvent.data[0] = fVal;
				} else if (event->code == FREEMOTIOND_EVENT_Y) {
					mPendingEvent.data[1] = fVal;
				} else if (event->code == FREEMOTIOND_EVENT_Z) {
					mPendingEvent.data[2] = fVal;
				} else if (event->code == ABS_WHEEL) {
					mPendingEvent.data[3] = fVal;
#if defined(USE_EMBEDDED_TS)
				} else if (event->code == ABS_THROTTLE) {
					mPendingEvent.timestamp &= (0xffffffff << 32);
					mPendingEvent.timestamp = event->value;
				} else if (event->code == ABS_RUDDER) {
					mPendingEvent.timestamp &= (0xffffffff);
					mPendingEvent.timestamp |= (event->value << 32);
				}
#endif
				} else {
					//Unhandled event
					//LOGE("%s: Unhandled event code - %d", __FUNCTION__, event->code);
				}
#endif
			}
		} else if (type == EV_SYN) {
			/* HY-DBG: Replace with sent timestamp */
#if !defined(USE_EMBEDDED_TS)
			mPendingEvent.timestamp = timevalToNano(event->time);
#endif
			if (SensorType == SENSOR_TYPE_STEP_DETECTOR || 
				SensorType == SENSOR_TYPE_SIGNIFICANT_MOTION) {
				mPendingEvent.data[0] = 1.0;
				mPendingEvent.data[1] = 0.0;
				mPendingEvent.data[2] = 0.0;
			}
			if (mEnabled) {
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;
			}
		} else {
			LOGE("OSP: unknown event (type=%d, code=%d)",
				type, event->code);
		}
		mpInputReader->next();
	}

	/* if we didn't read a complete event, see if we can fill and
	   try again instead of returning with nothing and redoing poll. */
	if (numEventReceived == 0 && mEnabled == true) {
		n = mpInputReader->fill(data_fd);
		if (n)
			goto again;
	}

	return numEventReceived;
}

