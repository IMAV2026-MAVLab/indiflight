/*
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

#include "platform.h"

#if defined(USE_RX_PI_OVERRIDE)

#include "rx/pi_override.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "drivers/time.h"

// Wire order, as in mspFrame[] of rx/msp.c, not the canonical order of
// fc/rc_controls.h: this array is indexed by rcmap[] positions, and the default
// AETR map puts throttle at 2 and yaw at 3, the opposite way round.
enum {
    PI_OVERRIDE_WIRE_ROLL = 0,
    PI_OVERRIDE_WIRE_PITCH = 1,
    PI_OVERRIDE_WIRE_THROTTLE = 2,
    PI_OVERRIDE_WIRE_YAW = 3,
};

static uint16_t piOverrideFrame[4];

// Latches true on the first RC_OVERRIDE ever received; staleness after that is
// piOverrideFrameTimeUs below
static bool piOverrideFrameValid = false;
static timeUs_t piOverrideFrameTimeUs = 0;

void rxPiOverrideFrameReceive(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle)
{
    piOverrideFrame[PI_OVERRIDE_WIRE_ROLL] = roll;
    piOverrideFrame[PI_OVERRIDE_WIRE_PITCH] = pitch;
    piOverrideFrame[PI_OVERRIDE_WIRE_THROTTLE] = throttle;
    piOverrideFrame[PI_OVERRIDE_WIRE_YAW] = yaw;
    piOverrideFrameTimeUs = micros();
    piOverrideFrameValid = true;
}

uint16_t rxPiOverrideReadRawRc(const rxRuntimeState_t *rxRuntimeState, const rxConfig_t *rxConfig, uint8_t chan)
{
    uint16_t rxSample = (rxRuntimeState->rcReadRawFn)(rxRuntimeState, chan);

    bool override = chan < 4 && ((1 << chan) & rxConfig->pi_override_channels_mask);

    const bool fresh = piOverrideFrameValid
        && (cmpTimeUs(micros(), piOverrideFrameTimeUs) < PI_OVERRIDE_TIMEOUT_US);

    // The two offboard languages are exclusive, and position control outranks
    if (IS_RC_MODE_ACTIVE(BOXPIOVERRIDE) && override && fresh
            && !FLIGHT_MODE(POSITION_MODE)) {
        return piOverrideFrame[chan];
    } else {
        return rxSample;
    }
}
#endif
