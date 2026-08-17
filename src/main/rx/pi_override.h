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

#pragma once

#include "rx/rx.h"
#include "pg/rx.h"

uint16_t rxPiOverrideReadRawRc(const rxRuntimeState_t *rxRuntimeState, const rxConfig_t *rxConfig, uint8_t chan);

/*
 * Called from telemetry/pi.c's uplink dispatch when an RC_OVERRIDE message is
 * parsed. Stores the roll/pitch/yaw/throttle values for
 * rxPiOverrideReadRawRc() to read - mirrors rxMspFrameReceive() in rx/msp.c.
 */
void rxPiOverrideFrameReceive(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle);

// Age past which the last frame stops being applied
#define PI_OVERRIDE_TIMEOUT_US 200000
