#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright © 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the CNRS-LAAS nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL CNRS-LAAS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


from __future__ import absolute_import, print_function, division

import errno
import os
import Queue
import random
import socket
import struct
import sys
import signal
import time

import numpy as np

import rospy
from rospy.timer import Timer
from rospy.rostime import Duration
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from skyscanner.msg import Attitude, Guidance, UAVSpeed, UAVState
from skyscanner.msg import Wind, WindSample

from utilities.FlightGear import FlightGear
from utilities.geometry import coord_to_meters, wrap_angle_deg, wrap_angle_rad


ABS_TOL = 1e-3
DECIMAL_DIGITS = 3


class FlightGearUAV(object):

    FEET_TO_M = 0.305
    KNOT_TO_MPERS = 0.514

    def __init__(self, ac_id, fg_in_port=51001, fg_out_port=50001,
                 telnet_port=5401, wind_port_out=50002, ground_alt=0,
                 orig_lat=0, orig_lon=0):
        """Init PaparazziUAV

        Arguments:
            ac_id: aircraft id.
            ground_alt: ground altitude above sea level.
        """
        self.ac_id = ac_id
        self.ground_alt = ground_alt

        self.tick_time = 0

        self.east = .0
        self.north = .0
        self.up = .0  # in m above grond level

        self.phi = .0  # in rad
        self.psi = .0  # in rad
        self.theta = .0  # in rad

        self.airspeed = 0  # in m/s
        self.groundspeed = 0  # in m/s
        self.vertical_speed = 0  # in m/s

        self.voltage = 0.  # in V
        self.current = 0.  # in A
        self.power = 0.  # in W
        self.energy = 0.  # in mAh

        self.battery_level = np.NaN

        self.orig_lat = orig_lat  # in degrees
        self.orig_lon = orig_lon  # in degrees

        self.desired_psi = 0  # in degrees
        self.desired_up = 1000  # in m

        self.measured_wind_x = 0.
        self.measured_wind_y = 0.
        self.measured_wind_z = 0.

        # Use UDP to send and receive state
        self.fg_out_port = fg_out_port
        self.fg_in_port = fg_in_port
        self.wind_port_out = wind_port_out

        self.udp_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_recv_sock.setblocking(False)
        self.udp_recv_sock.bind(('127.0.0.1', self.fg_out_port))
        self.fg_out_msg_struct = struct.Struct('>dddddddddi')  # Big endian!

        self.udp_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_send_sock.setblocking(False)

        self.udp_wind_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_wind_sock.setblocking(False)
        self.udp_wind_sock.bind(('127.0.0.1', self.wind_port_out))

        rospy.init_node('flightgearuav')

        self.ros_pub_state = rospy.Publisher('uav_state', UAVState,
                                             queue_size=1)
        self.ros_pub_speed = rospy.Publisher('uav_speed', UAVSpeed,
                                             queue_size=1)

        self.ros_pub_measured_wind = rospy.Publisher('measured_wind',
                                                     WindSample, queue_size=1)

        rospy.Subscriber('/tick', Clock, self.on_tick_recv, queue_size=10)
        rospy.Subscriber('/fast_tick', Clock, self.on_fast_tick_recv,
                         queue_size=10)
        rospy.Subscriber('guidance', Guidance, self.on_guidance_recv)

        # Use telnet to send low rate commands like autopilot activation
        self.fg = None
        try:
            self.fg = FlightGear('localhost', telnet_port)
            self.set_autopilot_lock(True, True)
        except:
            rospy.logwarn("Flightgear Telnet server not availabe.")

    def on_tick_recv(self, tick):
        """On new time slot beginning.

        Arguments:
            tick: time-slot time.
        """
        self.recv_wind_from_fg()
        self.publish_wind_sample()

    def on_fast_tick_recv(self, tick):
        """On new time slot beginning.

        Arguments:
            tick: time-slot time.
        """
        self.tick_time = tick.clock.to_sec()
        self.recv_from_fg()
        self.publish_state()
        self.publish_speed()
        self.send_to_fg()

    def publish_state(self):
        """Publish state into ROS topic."""
        # Increment time in 1 to be consistent with simuav behavior which sends
        # the state at the *end* of the cycle, so tick_time + 1
        h = Header(stamp=rospy.Time(self.tick_time + 1))
        position = Point(self.east, self.north, self.up)
        attitude = Attitude(self.phi, wrap_angle_rad(-self.psi - np.pi/2),
                            self.theta)
        battery_level = self.battery_level
        uav_state = UAVState(h, self.ac_id, position, attitude, battery_level)

        rospy.loginfo("[fguav] [ac_id={}]\n{}".format(self.ac_id, uav_state))
        self.ros_pub_state.publish(uav_state)

    def publish_speed(self):
        """Publish speed into ROS topic."""
        h = Header(stamp=rospy.Time(self.tick_time + 1))
        uav_speed = UAVSpeed(h, self.ac_id, self.airspeed, self.groundspeed,
                             self.vertical_speed)

        rospy.loginfo("[fguav] [ac_id={}]\n{}".format(self.ac_id, uav_speed))
        self.ros_pub_speed.publish(uav_speed)

    def publish_energy_consumption(self):
        """Publish power information into ROS topic."""
        # Increment time in 1 to be consistent with simuav behavior which sends
        # the state at the *end* of the cycle, so tick_time + 1
        h = Header(stamp=rospy.Time(self.tick_time + 1))
        energy_con = EnergyConsumption(
            h, self.ac_id, self.voltage, self.current, self.power, self.energy)
        rospy.logdebug("[fgfsuav] [ac_id={}] state:\n{}".format(self.ac_id,
                                                                energy_con))
        self.ros_pub_energy_consumption.publish(energy_con)

    def publish_wind_sample(self):
        """Publish wind sample into ROS topic."""
        h = Header(stamp=rospy.Time(self.tick_time))
        position = Point(float(self.east), float(self.north), float(self.up))
        wind = Wind(float(self.measured_wind_x),  float(self.measured_wind_y),
                    float(self.measured_wind_z))
        wind_sample = WindSample(h, self.ac_id, position, wind)

        rospy.loginfo("[fgfsuav] [ac_id={}] wind:\n{}".format(self.ac_id,
                                                              wind_sample))
        self.ros_pub_measured_wind.publish(wind_sample)

    def set_autopilot_lock(self, true_heading, target_agl):
        """Set flightgear autopilot.

        Arguments:
            true_heading: set or clear heading autopilot to track the
                true heading.
            target_agl: set or clear pitch autopilot to track the
                above ground altitude.
        """
        if true_heading:
            self.set_fg_telnet('/autopilot/locks/heading',
                                 'true-heading-hold')
        else:
            self.set_fg_telnet('/autopilot/locks/heading', '')

        if target_agl:
            self.set_fg_telnet('/autopilot/locks/altitude', 'agl-hold')
        else:
            self.set_fg_telnet('/autopilot/locks/altitude', '')

    def set_fg_telnet(self, prop, val):
        """Set property in flight gear using telnet.

        Arguments:
            prop: property key
            val: value
        """
        try:
            self.fg[prop] = val
        except Exception as e:
            rospy.logerr(e)

    def get_fg_telnet(self, prop):
        """Get property in flight gear using telnet.

        Arguments:
            prop: property key

        Returns:
            property or None if cannot be obtained
        """
        val = None
        try:
            val = self.fg[prop]
        except Exception as e:
            rospy.logerr(e)
        return val

    def send_to_fg(self):
        """Send commands to flightgear using UDP socket. """
        psi_deg = self.desired_psi * 180 / np.pi
        try:
            msg = '|'.join([str(psi_deg), str(self.desired_up)])
            self.udp_send_sock.sendto(''.join([msg, '\n']),
                                      ('127.0.0.1', self.fg_in_port))
        except socket.error as a:
            raise

    def recv_wind_from_fg(self):
        """Receive wind from flightgear using UDP socket."""
        def parse_ascii(data, l_sep, v_sep):
            data = data.split(l_sep)[0]
            params = data.split(v_sep)
            result = []
            for v in params:
                result.append(float(v))
            return result

        try:
            data = self.udp_wind_sock.recv(4096)
            if data:
                state = parse_ascii(data, l_sep='\n', v_sep='|')
                self.measured_wind_x = float(state[1])  # east
                self.measured_wind_y = float(state[0])  # north
                self.measured_wind_z = float(state[2])  # up
        except socket.error as e:
            if e.errno == errno.EAGAIN:
                rospy.loginfo("No data available")
            else:
                raise  # reraise exception

    def recv_from_fg(self):
        """Receive flightgear uav state using UDP socket."""

        def parse_binary(data):
            state = self.fg_out_msg_struct.unpack(data)
            if state[9] != 0x12345678:
                raise ValueError("Incorrect magic number")
            return state

        def parse_ascii(data, l_sep, v_sep):
            data = data.split(l_sep)[0]
            params = data.split(v_sep)
            result = []
            for v in params:
                result.append(float(v))
            return result
        try:
            data = self.udp_recv_sock.recv(4096)
            if data:
                state = parse_ascii(data, l_sep='\n', v_sep='|')

                lat_m = coord_to_meters(self.orig_lat, 0.0, state[0], 0.0)
                if state[0] > self.orig_lat:
                    self.north = lat_m
                else:
                    self.north = -lat_m
                lon_m = coord_to_meters(0.0, self.orig_lon, 0.0, state[1])
                if state[1] > self.orig_lon:
                    self.east = lon_m
                else:
                    self.east = -lon_m
                self.up = state[2]
                self.psi = state[3] * np.pi / 180.0
                self.theta = state[4] * np.pi / 180.0
                self.phi = state[5] * np.pi / 180.0
                self.airspeed = state[6]
                self.groundspeed = state[7]
                self.vertical_speed = state[8]
        except socket.error as e:
            if e.errno == errno.EAGAIN:
                rospy.loginfo("No data available")
            else:
                raise  # reraise exception

    def on_guidance_recv(self, guidance):
        """Handle aircraft desired state reception."""
        rospy.loginfo("[fguav][ac_id={}] Guidance:\n{}".format(
            guidance.ac_id, guidance))
        if not np.isclose(self.desired_psi, guidance.heading, atol=0.01):
            if not np.isnan(guidance.heading):
                self.desired_psi = wrap_angle_rad(-guidance.heading + np.pi/2)
            else:
                rospy.logwarn("guidance.heading is NaN!")
        if not np.isclose(self.desired_up, guidance.altitude, atol=0.01):
            if not np.isnan(guidance.altitude):
                self.desired_up = guidance.altitude
            else:
                rospy.logwarn("guidance.altitude is NaN!")


def main():
    def shutdown_hook():
        pass

    # Setup math precision
    global ABS_TOL, DECIMAL_DIGITS
    ABS_TOL = rospy.get_param(rospy.search_param('abs_tol'), 1e-3)
    DECIMAL_DIGITS = int(np.around(np.log10(1./ABS_TOL)))

    ac_id = rospy.get_param(rospy.search_param('ac_id'))

    lat = rospy.get_param(rospy.search_param('lat'))
    lon = rospy.get_param(rospy.search_param('lon'))

    fg_hostname = rospy.get_param(rospy.search_param('fg_hostname'),
                                  default="localhost")
    control_port_in = rospy.get_param(rospy.search_param('control_port_in'))
    control_port_out = rospy.get_param(rospy.search_param('control_port_out'))
    wind_port_out = rospy.get_param(rospy.search_param('wind_port_out'),
                                    default=50002)
    telnet_port = rospy.get_param(rospy.search_param('telnet_port'),
                                  default=5401)

    uav = FlightGearUAV(ac_id, orig_lat=lat, orig_lon=lon,
                        fg_in_port=control_port_in,
                        fg_out_port=control_port_out,
                        wind_port_out=wind_port_out,
                        telnet_port=telnet_port)

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    main()
