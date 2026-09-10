#!/usr/bin/env python3

# Self-check for the position_use_lateral_thrust setting, using the MOCKUP target
#
# Copyright 2026 Rafael Perez-Segui
#
# This file is part of Indiflight.
#
# Indiflight is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# Indiflight is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.
#
# If not, see <https://www.gnu.org/licenses/>.
#
# Build the library first, then run from the repository root:
#     make TARGET=MOCKUP PROFILE=CineRat
#     python3 src/utils/test_position_lateral_thrust.py

import ctypes as ct
import json
import math
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
from indiflight_mockup_interface import IndiflightSITLMockup, flightModeFlags

LIBRARY = "./obj/main/indiflight_MOCKUP.so"
BASE_PROFILE = "configs/profiles/CineRat.txt"
SETPOINT_NED = [5., 0., -1.]     # m, 5 m north of the drone
TICKS_TO_CONVERGE_EKF = 80000    # 10 s at 8 kHz
TICKS_IN_POSITION_MODE = 16000   # 2 s at 8 kHz


class fp_vector_t(ct.Structure):
    _fields_ = [("X", ct.c_float), ("Y", ct.c_float), ("Z", ct.c_float)]


class fp_quaternion_t(ct.Structure):
    _fields_ = [("w", ct.c_float), ("x", ct.c_float), ("y", ct.c_float), ("z", ct.c_float)]


class local_pos_sp_ned_t(ct.Structure):
    _fields_ = [("time_us", ct.c_uint32), ("source", ct.c_int), ("valid", ct.c_bool),
                ("pos", fp_vector_t), ("vel", fp_vector_t), ("psi", ct.c_float),
                ("psi_rate", ct.c_float), ("trackPsi", ct.c_bool), ("mode", ct.c_uint8)]


def flyNorthSetpoint(useLateralThrust, headingDeg=0., trackPsi=True):
    """Hover a quad at the given heading, ask for a position 5 m north, return the resulting setpoints."""
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as profile:
        with open(BASE_PROFILE, "r") as base:
            profile.write(base.read())
        profile.write("\nset ekf_meas_source = 3\n")  # LOCAL_POS_SOURCE_MOCKUP
        profile.write("positionprofile 0\n")
        profile.write(f"set position_use_lateral_thrust = {useLateralThrust}\n")

    mockup = IndiflightSITLMockup(LIBRARY, Nr=4)
    mockup.load_profile(profile.name)
    os.remove(profile.name)
    mockup.enableFlightMode(flightModeFlags.ANGLE_MODE)

    heading = math.radians(headingDeg)
    attitude = [math.cos(0.5 * heading), 0., 0., math.sin(0.5 * heading)]

    def spin(ticks):
        for _ in range(ticks):
            mockup.sendImu([0., 0., 0.], [0., 0., -9.81])
            mockup.sendMocap([0., 0., -1.], [0., 0., 0.], attitude)
            mockup.sendMotorSpeeds([2000., 2000., 2000., 2000.])
            mockup.tick()

    spin(TICKS_TO_CONVERGE_EKF)
    if not mockup.getVariableReference(ct.c_bool, "ekf_converged").value:
        raise RuntimeError("EKF did not converge, cannot exercise position control")

    mockup.enableFlightMode(flightModeFlags.POSITION_MODE)
    mockup.sendPositionSetpoint(SETPOINT_NED, 0.)
    setpoint = mockup.getVariableReference(local_pos_sp_ned_t, "posSpNed")
    readBack = [setpoint.pos.X, setpoint.pos.Y, setpoint.pos.Z]
    if readBack != SETPOINT_NED:
        raise RuntimeError(f"local_pos_sp_ned_t layout mismatch, read back {readBack}")
    setpoint.trackPsi = trackPsi
    mockup.arm()
    spin(TICKS_IN_POSITION_MODE)

    spf = mockup.getVariableReference(fp_vector_t, "spfSpBodyFromPos")
    att = mockup.getVariableReference(fp_quaternion_t, "attSpNedFromPos")
    result = {"spf": [spf.X, spf.Y, spf.Z], "att": [att.w, att.x, att.y, att.z]}
    mockup.disarm()
    return result


def measureInSubprocess(*args):
    """One library load per case, the mockup keeps global state across a run."""
    output = subprocess.check_output([sys.executable, __file__] + [str(a) for a in args])
    return json.loads(output.decode().strip().splitlines()[-1])


if __name__ == "__main__":
    if len(sys.argv) > 1:
        useLateralThrust, headingDeg, trackPsi = int(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3])
        print(json.dumps(flyNorthSetpoint(useLateralThrust, headingDeg, bool(trackPsi))))
        sys.exit(0)

    tilting = measureInSubprocess(0, 0., 1)
    lateral = measureInSubprocess(1, 0., 1)
    yawed = measureInSubprocess(1, 90., 0)
    print(f"tilting: {tilting}")
    print(f"lateral: {lateral}")
    print(f"yawed:   {yawed}")

    # underactuated: thrust stays on the body z axis and the attitude setpoint pitches north
    assert abs(tilting["spf"][0]) < 1e-3, tilting
    assert abs(tilting["spf"][1]) < 1e-3, tilting
    assert tilting["spf"][2] < -1., tilting
    assert tilting["att"][2] < -0.1, tilting

    # fully actuated: the attitude setpoint stays level and the force setpoint points north
    assert lateral["spf"][0] > 1., lateral
    assert abs(lateral["spf"][1]) < 1e-3, lateral
    assert lateral["spf"][2] < -1., lateral
    assert abs(lateral["att"][1]) < 1e-3 and abs(lateral["att"][2]) < 1e-3, lateral

    # nose east and no heading to track: the force setpoint rotates into the body frame,
    # and the attitude setpoint holds the current heading instead of snapping to north
    assert abs(yawed["spf"][0]) < 0.1, yawed
    assert yawed["spf"][1] < -1., yawed
    assert abs(yawed["spf"][1] + lateral["spf"][0]) < 0.1, yawed
    assert abs(yawed["att"][3] - math.sin(math.radians(45.))) < 1e-2, yawed

    print("PASS")
