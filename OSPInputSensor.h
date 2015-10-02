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

#ifndef OSPINPUTSENSOR_H_
#define OSPINPUTSENSOR_H_

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"
struct input_event;

#define  FM_DECODE_VALUE_AS_INTEGER false
#define  FM_DECODE_VALUE_AS_FLOAT true

//! use to read any floating point data from OSP provided input sensors
/*!
 *  instantiate one these objects for every OSP provided sensor you're interested in
 */
class OSPInputSensor : public SensorBase {
public:
    OSPInputSensor(const char* uinputName, int32_t sensorId, int32_t sensorType, bool evtFloat);
    virtual ~OSPInputSensor();

    virtual int readEvents(sensors_event_t* data, int count);
    virtual bool hasPendingEvents() const;
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled);

protected:
    OSPInputSensor();
    OSPInputSensor(const OSPInputSensor&);
    void operator=(const OSPInputSensor&);

    bool mEnabled;
    bool mEventsAreFloat;
    InputEventCircularReader *mpInputReader;
    sensors_event_t mPendingEvent;
    bool mHasPendingEvent;
    char input_sysfs_path[PATH_MAX];
    int input_sysfs_path_len;

    virtual int setInitialState();
    virtual bool handleEvent(input_event const * event, sensors_event_t* androidData);
    const char *uinputName;
    int Qscale;
    bool mpReadAvail;
    int32_t SensorType;
    char ctrlPath[PATH_MAX];
};


class OSPStepCounter : public OSPInputSensor {
public:
    OSPStepCounter(const char* uinputName, int32_t sensorId, int32_t sensorType, bool evtFloat) 
      : OSPInputSensor(uinputName, sensorId, sensorType, evtFloat) {
      LOGE("creating custom handler for SIG_MOTION"); };


 protected:
     virtual bool handleEvent(input_event const * event, sensors_event_t* androidData);
};

class OSPSignificantMotion : public OSPInputSensor {
public:
    OSPSignificantMotion(const char* uinputName, int32_t sensorId, int32_t sensorType, bool evtFloat) 
      : OSPInputSensor(uinputName, sensorId, sensorType, evtFloat) {
      LOGE("creating custom handler for SIG_MOTION"); };

 protected:
     virtual bool handleEvent(input_event const * event, sensors_event_t* androidData);
};


/*****************************************************************************/

#endif  // OSPINPUTSENSOR_H_
