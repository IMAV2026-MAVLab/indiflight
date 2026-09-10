/*
 * Configure serial port to parse pi-messages and provide facilities to send
 *
 * Copyright 2023 Till Blaha (Delft University of Technology)
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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_TELEMETRY_PI)

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/dshot.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/light_led.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/ahrs.h"
#include "flight/indi.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/ekf.h"
#include "flight/pos_ctl.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/local_pos.h"

#include "rx/rx.h"
#include "rx/pi_override.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/pi.h"
#include "pi-protocol.h"
#include "pi-messages.h"

#ifdef USE_CLI_DEBUG_PRINT
#include "cli/cli_debug_print.h"
#endif

#define TELEMETRY_PI_INITIAL_PORT_MODE MODE_RXTX
#define TELEMETRY_PI_MAXRATE 50
#define TELEMETRY_PI_DELAY ((1000 * 1000) / TELEMETRY_PI_MAXRATE)

static serialPort_t *piPort = NULL;
static const serialPortConfig_t *portConfig;

static bool piTelemetryEnabled =  false;
static portSharing_e piPortSharing;

// wrapper for serialWrite
static void serialWriter(uint8_t byte) { serialWrite(piPort, byte); }

void freePiTelemetryPort(void)
{
    closeSerialPort(piPort);
    piPort = NULL;
    piTelemetryEnabled = false;
}

void initPiTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_PI);
    piPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_PI);
}

#define BLINK_ONCE delay(500); LED1_ON; delay(100); LED1_OFF; delay(100)

void configurePiTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_921600;
    }

    piPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_PI, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_PI_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

    if (!piPort) {
        return;
    }

    piTelemetryEnabled = true;
}

// The exchange already carries the host's wall clock, so the message that
// syncs the two clocks also dates the blackbox: a log writes rtcGetDateTime()
// into its header when it opens, and the FC has no clock of its own to date it
// with. Anything older than the firmware is a host that has not set its own
// clock yet, and would date every log to 1970.
#define PI_RTC_MIN_UNIX_SEC 1735689600  // 2025-01-01

static void piSetRtcFromHost(uint64_t host_ns)
{
#ifdef USE_RTC_TIME
    const int32_t secs = (int32_t)(host_ns / 1000000000ull);
    if (secs < PI_RTC_MIN_UNIX_SEC) {
        return;
    }

    const uint16_t millis = (uint16_t)((host_ns % 1000000000ull) / 1000000ull);
    rtcTime_t t = rtcTimeMake(secs, millis);
    rtcSet(&t);
#else
    UNUSED(host_ns);
#endif
}

void checkPiTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!piTelemetryEnabled && telemetrySharedPort != NULL) {
            piPort = telemetrySharedPort;
            piTelemetryEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(piPortSharing);

        if (newTelemetryEnabledValue == piTelemetryEnabled) {
            return;
        }

        if (newTelemetryEnabledValue)
            configurePiTelemetryPort();
        else
            freePiTelemetryPort();
    }
}

void piSendIMU(void)
{
    piMsgImuTx.time_us = (uint32_t) gyro.rawSensorDev->gyroLastEXTIUs;
    piMsgImuTx.roll = DEGREES_TO_RADIANS(gyro.gyroADCafterRpm[0]);
    piMsgImuTx.pitch = DEGREES_TO_RADIANS(gyro.gyroADCafterRpm[1]);
    piMsgImuTx.yaw = DEGREES_TO_RADIANS(gyro.gyroADCafterRpm[2]);
    piMsgImuTx.x = GRAVITYf * acc.accADCafterRpm[0] * acc.dev.acc_1G_rec;
    piMsgImuTx.y = GRAVITYf * acc.accADCafterRpm[1] * acc.dev.acc_1G_rec;
    piMsgImuTx.z = GRAVITYf * acc.accADCafterRpm[2] * acc.dev.acc_1G_rec;

    if (piPort) {
        piSendMsg(&piMsgImuTx, &serialWriter);
    }
}

void piSendEkfInputs(void)
{
    piMsgEkfInputsTx.time_us = (uint32_t) gyro.rawSensorDev->gyroLastEXTIUs;
    piMsgEkfInputsTx.x = 2048.f * acc.accADCf[0] * acc.dev.acc_1G_rec;
    piMsgEkfInputsTx.y = 2048.f * acc.accADCf[1] * acc.dev.acc_1G_rec;
    piMsgEkfInputsTx.z = 2048.f * acc.accADCf[2] * acc.dev.acc_1G_rec;
    piMsgEkfInputsTx.p = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCafterRpm[0] * 0.0005f );
    piMsgEkfInputsTx.q = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCafterRpm[1] * 0.0005f );
    piMsgEkfInputsTx.r = (int16_t) ( ((float) ((1 << 15) - 1)) * gyro.gyroADCafterRpm[2] * 0.0005f );
    // The allocator never writes omega[] past actNum, so an airframe with fewer
    // rotors than the message carries would ship whatever is in those slots
    int16_t omega[6] = {0};
#ifdef USE_DSHOT_TELEMETRY
    for (int i = 0; i < MIN(indiRun.actNum, 6); i++) {
        omega[i] = (int16_t) indiRun.omega[i];
    }
#endif
    piMsgEkfInputsTx.omega1 = omega[0];
    piMsgEkfInputsTx.omega2 = omega[1];
    piMsgEkfInputsTx.omega3 = omega[2];
    piMsgEkfInputsTx.omega4 = omega[3];
    piMsgEkfInputsTx.omega5 = omega[4];
    piMsgEkfInputsTx.omega6 = omega[5];

    if (piPort) {
        piSendMsg(&piMsgEkfInputsTx, &serialWriter);
    }
}

// Straight from the RX driver: before PI OVERRIDE substitutes a channel and
// before the calibration, so this is the pilot in the receiver's native range.
static int16_t piReadRcChannel(uint8_t wireChannel)
{
    if (wireChannel >= rxRuntimeState.channelCount) {
        return 0;
    }

    return (int16_t) rxRuntimeState.rcReadRawFn(&rxRuntimeState, wireChannel);
}

void piSendRc(void)
{
    piMsgRcTx.time_us = micros();
    piMsgRcTx.channel_1 = piReadRcChannel(0);
    piMsgRcTx.channel_2 = piReadRcChannel(1);
    piMsgRcTx.channel_3 = piReadRcChannel(2);
    piMsgRcTx.channel_4 = piReadRcChannel(3);
    piMsgRcTx.channel_5 = piReadRcChannel(4);
    piMsgRcTx.channel_6 = piReadRcChannel(5);
    piMsgRcTx.channel_7 = piReadRcChannel(6);
    piMsgRcTx.channel_8 = piReadRcChannel(7);
    piMsgRcTx.channel_9 = piReadRcChannel(8);
    piMsgRcTx.channel_10 = piReadRcChannel(9);
    piMsgRcTx.channel_11 = piReadRcChannel(10);
    piMsgRcTx.channel_12 = piReadRcChannel(11);
    piMsgRcTx.channel_13 = piReadRcChannel(12);
    piMsgRcTx.channel_14 = piReadRcChannel(13);
    piMsgRcTx.channel_15 = piReadRcChannel(14);
    piMsgRcTx.channel_16 = piReadRcChannel(15);

    if (piPort) {
        piSendMsg(&piMsgRcTx, &serialWriter);
    }
}

void piSendStatus(void)
{
    piMsgPiStatusTx.time_us = micros();

    uint8_t flags = 0;
    if (ARMING_FLAG(ARMED)) {
        flags |= PI_STATUS_FLAG_ARMED;
    }
    // Which control family the pilot's switches have selected, so that the host
    // knows which of the two offboard languages the FC is currently obeying.
    if (FLIGHT_MODE(POSITION_MODE)
#ifdef USE_LOCAL_POSITION
            && !isManualTakeover()
#endif
       ) {
        flags |= PI_STATUS_FLAG_POS_CTL_ACTIVE;
    }
#if defined(USE_RX_PI_OVERRIDE)
    if (IS_RC_MODE_ACTIVE(BOXPIOVERRIDE)) {
        flags |= PI_STATUS_FLAG_PI_OVERRIDE_ACTIVE;
    }
#endif
    if (rxAreFlightChannelsValid()) {
        flags |= PI_STATUS_FLAG_RX_LINK_VALID;
    }
#ifdef USE_EKF
    if (isConvergedEkf()) {
        flags |= PI_STATUS_FLAG_EKF_CONVERGED;
    }
#endif
    piMsgPiStatusTx.flags = flags;

    if (piPort) {
        piSendMsg(&piMsgPiStatusTx, &serialWriter);
    }
}

void piSendBattery(void)
{
    piMsgBatteryTx.time_us = micros();
    piMsgBatteryTx.voltage = getBatteryVoltage() * 0.01f;  // 10mV units -> V
    piMsgBatteryTx.current = getAmperage() * 0.01f;        // 10mA units -> A
    piMsgBatteryTx.cell_count = getBatteryCellCount();

    if (piPort) {
        piSendMsg(&piMsgBatteryTx, &serialWriter);
    }
}

void processPiTelemetry(void)
{
    // handled event based now, whenever there is stuff to be send, those functions
    // call piSendEkfInputs, or similar. More boilerplate, but lower latency
    piSendIMU();

    // Downlink rate, decoupled from the task period the uplink needs
    static timeUs_t lastDownlinkUs = 0;
    const timeUs_t now = micros();

    if (cmpTimeUs(now, lastDownlinkUs) >= TELEMETRY_PI_DELAY) {
        lastDownlinkUs = now;
        piSendRc();
        piSendStatus();
        piSendBattery();
    }
}

static void processNewMessage(uint8_t msgId) {
    switch (msgId) {
#if defined(USE_RX_PI_OVERRIDE)
        case PI_MSG_RC_OVERRIDE_ID: {
            rxPiOverrideFrameReceive(piMsgRcOverrideRx->roll, piMsgRcOverrideRx->pitch,
                piMsgRcOverrideRx->yaw, piMsgRcOverrideRx->throttle);
            break;
        }
#endif
        case PI_MSG_TIMESYNC_ID: {
            // Answered here rather than from the send path: any delay added
            // between the two stamps is offset error for the host
            piMsgTimesyncTx.seq = piMsgTimesyncRx->seq;
            piMsgTimesyncTx.host_ns = piMsgTimesyncRx->host_ns;
            piMsgTimesyncTx.fc_time_us = micros();
            if (piPort) {
                piSendMsg(&piMsgTimesyncTx, &serialWriter);
            }
            piSetRtcFromHost(piMsgTimesyncRx->host_ns);
            break;
        }
#ifdef USE_LOCAL_POSITION
        case PI_MSG_EXTERNAL_POSE_ID: {
            local_pos_ned_t pos;
            pos.mode = piMsgExternalPoseRx->mode;
            pos.time_us = piMsgExternalPoseRx->time_us;
            pos.source = LOCAL_POS_SOURCE_PI;
            // process new message (should be NED)
            pos.pos.V.X = piMsgExternalPoseRx->ned_x;
            pos.pos.V.Y = piMsgExternalPoseRx->ned_y;
            pos.pos.V.Z = piMsgExternalPoseRx->ned_z;
            pos.vel.V.X = piMsgExternalPoseRx->ned_xd;
            pos.vel.V.Y = piMsgExternalPoseRx->ned_yd;
            pos.vel.V.Z = piMsgExternalPoseRx->ned_zd;
            // the quaternion x,y,z should be NED
            pos.quat.w = piMsgExternalPoseRx->body_qi;
            pos.quat.x = piMsgExternalPoseRx->body_qx;
            pos.quat.y = piMsgExternalPoseRx->body_qy;
            pos.quat.z = piMsgExternalPoseRx->body_qz;

            pos.vel_valid = true;
            pos.quat_valid = true;

            setLocalPosMeas(&pos);
            break;
        }
        case PI_MSG_SETPOINT_ID: {
            local_pos_sp_ned_t sp = {0};
            sp.mode = piMsgSetpointRx->mode;

            if ((sp.mode & LOCAL_POS_SP_MODE_MASK) > LOCAL_POS_SP_ACRO) {
                break;
            }

            sp.time_us = piMsgSetpointRx->time_us;
            sp.source = LOCAL_POS_SOURCE_PI;
            sp.pos.V.X = piMsgSetpointRx->vec_a_x;
            sp.pos.V.Y = piMsgSetpointRx->vec_a_y;
            sp.pos.V.Z = piMsgSetpointRx->vec_a_z;
            sp.vel.V.X = piMsgSetpointRx->vec_b_x;
            sp.vel.V.Y = piMsgSetpointRx->vec_b_y;
            sp.vel.V.Z = piMsgSetpointRx->vec_b_z;

            // The yaw rate bit only applies to modes 0-2: ATTITUDE carries yaw
            // in the quaternion w it puts in scalar_c, and ACRO in vec_a
            if ((sp.mode & LOCAL_POS_SP_YAW_RATE)
                    && ((sp.mode & LOCAL_POS_SP_MODE_MASK) <= LOCAL_POS_SP_TRAJECTORY)) {
                sp.psi_rate = piMsgSetpointRx->scalar_c;
                sp.trackPsi = false;
            } else {
                sp.psi = piMsgSetpointRx->scalar_c;
                sp.trackPsi = true;
            }

            setLocalPosSp(&sp);
            break;
        }
#endif
    }
}

pi_parse_states_t p_telem;

void processPiUplink(void)
{
    if (!piPort) {
        return;
    }

    while (serialRxBytesWaiting(piPort)) {
        uint8_t msgId = piParse(&p_telem, serialRead(piPort));
        if (msgId != PI_MSG_NONE_ID) {
            processNewMessage(msgId);
        }
    }
}

void handlePiTelemetry(void)
{
    if (!piTelemetryEnabled || !piPort) {
        return;
    }

    processPiTelemetry();
    processPiUplink();
}

#endif
