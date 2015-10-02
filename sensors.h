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

#ifndef ANDROID_SENSORS_H
#define ANDROID_SENSORS_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <linux/input.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

__BEGIN_DECLS

#define USE_MAG_RAW

/*****************************************************************************/

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#define ID_A        (0)
#define ID_M        (1)
#define ID_O        (2)
#define ID_L        (3)
#define ID_P        (4)
#define ID_GY       (5)
#define ID_PRESS    (6)
#define ID_TEMP     (7)
#define ID_A2       (8)
#define ID_VA       (9)
#define ID_RV       (10)
#define ID_LINACC   (11)
#define ID_GRAV     (12)
#define ID_UNCALIBRATED_MAG    (13)
#define ID_GAME_ROT_VEC        (14)
#define ID_UNCALIBRATED_GYRO   (15)
#define ID_SIG_MOTION          (16)
#define ID_STEP_DETECTOR       (17)
#define ID_STEP_COUNTER        (18)
#define ID_GEOMAGNETIC_ROT_VEC (19)

/**** SENSOR NAMES corresponding to DRIVERS *****/

/* LibFM Sensors */
#define ACCEL_UINPUT_NAME       "fm-accelerometer"
#define GYRO_UINPUT_NAME        "fm-gyroscope"
#define MAG_UINPUT_NAME         "fm-magnetometer"
#define SENSOR_LFM_ROT_VECT     "fm-rotation-vector"

/*****************************************************************************/

/*
 * The SENSORS Module
 */

/* the GP2A is a binary proximity sensor that triggers around 5 cm on
 * this hardware */
#define PROXIMITY_THRESHOLD_GP2A    5.0f

/*****************************************************************************/

#define  FREEMOTIOND_EVENT_X        ABS_X
#define  FREEMOTIOND_EVENT_Y        ABS_Y
#define  FREEMOTIOND_EVENT_Z        ABS_Z

#define EVENT_TYPE_YAW              REL_RX
#define EVENT_TYPE_PITCH            REL_RY
#define EVENT_TYPE_ROLL             REL_RZ
#define EVENT_TYPE_ORIENT_STATUS    REL_WHEEL

#define EVENT_TYPE_PRESSURE         REL_HWHEEL
#define EVENT_TYPE_TEMPERATURE      ABS_MISC


#define EVENT_TYPE_ROT_X            ABS_X
#define EVENT_TYPE_ROT_Y            ABS_Y
#define EVENT_TYPE_ROT_Z            ABS_Z

#define EVENT_TYPE_PROXIMITY        ABS_DISTANCE
#define EVENT_TYPE_LIGHT            REL_MISC



// 1000 LSG = 1G
#define LSG                         (1000.0f)

#define RANGE_PRESS                 (100000.0f)
// data from freeMotionD is already in android units
#define RANGE_A                     (2*GRAVITY_EARTH)
#define RESOLUTION_A                (GRAVITY_EARTH / LSG)

// data from freeMotionD is already in android units
#define RANGE_M                     (500.0f) //+/-2.5G = +/- 250uT = 500uT range
#define RESOLUTION_M                (0.152f) //0.152uT/LSB

/* conversion of orientation data to degree units */
#define CONVERT_O                   (1.0f/64.0f)
#define CONVERT_O_A                 (CONVERT_O)
#define CONVERT_O_P                 (CONVERT_O)
#define CONVERT_O_R                 (-CONVERT_O)

// data from freeMotionD is already in android units
#define RANGE_GYRO                  (2000.0f*(float)M_PI/180.0f)
#define RESOLUTION_GYRO             (0.000000001f)

#define CONVERT_ROT_XYZ             (1.0f)

#define CONVERT_TEMP                (1.0f/10.0f)
#define RANGE_T                     (85.0f)
#define RESOLUTION_T                (0.1f)

// conversion of pressure data to mb units
#define RANGE_P                     (1100.0f)
#define RESOLUTION_P                (1.0f)
/* Press sensor is in Pa, Android wants mb */
#define CONVERT_PRESS               (1.0f/100.0f)

#define SENSOR_STATE_MASK           (0x7FFF)

/*****************************************************************************/

__END_DECLS

#endif  // ANDROID_SENSORS_H
