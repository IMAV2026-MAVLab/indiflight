/*
 * Get local NED position from difference sources (uplink/GPS)
 *
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *
 * This file is part of Indiflight.
 *
 * Indiflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Indiflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef LOCAL_POS_H
#define LOCAL_POS_H

#include "drivers/time.h"
#include "common/maths.h"
#include "io/gps.h"

typedef enum {
    LOCAL_POS_SOURCE_PI,
    LOCAL_POS_SOURCE_UROS,
    LOCAL_POS_SOURCE_GPS,
    LOCAL_POS_SOURCE_MOCKUP,
} local_pos_source_e;

// todo: reformulate using fp_vector_t
// Which quantities of a measurement to use, see msgs/EXTERNAL_POSE.yaml
#define LOCAL_POS_MEAS_USE_POS   (1 << 0)
#define LOCAL_POS_MEAS_USE_QUAT  (1 << 1)
#define LOCAL_POS_MEAS_USE_VEL   (1 << 2)
#define LOCAL_POS_MEAS_TRUST     (1 << 3)

// What the controller does with a setpoint, see msgs/SETPOINT.yaml
#define LOCAL_POS_SP_MODE_MASK   0x07
#define LOCAL_POS_SP_POSITION    0
#define LOCAL_POS_SP_VELOCITY    1
#define LOCAL_POS_SP_TRAJECTORY  2
#define LOCAL_POS_SP_ATTITUDE    3
#define LOCAL_POS_SP_ACRO        4
#define LOCAL_POS_SP_YAW_RATE    (1 << 3)

typedef struct __local_pos_ned_t {
    uint32_t time_us;
    local_pos_source_e source;
    bool new;
    fp_vector_t pos;
    bool vel_valid;
    fp_vector_t vel;
    bool quat_valid;
    fp_quaternion_t quat;
    uint8_t mode;
} local_pos_ned_t;

typedef struct __local_pos_sp_ned_t {
    uint32_t time_us;
    local_pos_source_e source;
    bool valid;
    fp_vector_t pos;
    fp_vector_t vel;
    float psi;
    float psi_rate;
    bool trackPsi;
    uint8_t mode;
} local_pos_sp_ned_t;

extern local_pos_ned_t posMeasNed;
extern local_pos_sp_ned_t posSpNed;

void setLocalPosMeas(local_pos_ned_t* pos);
void setLocalPosSp(local_pos_sp_ned_t* sp);
void setLocalPosSpHere(void);

void llh_to_local(const gpsLocation_t* llh, const gpsLocation_t* home, fp_vector_t* ned);
void local_to_llh(const fp_vector_t* ned, const gpsLocation_t* home, gpsLocation_t* llh);

#endif // LOCAL_POS_H