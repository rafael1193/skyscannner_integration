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
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import absolute_import, print_function, division

import random
import os
import sys
import signal
import time
import Queue

import numpy as np

import rospy
from rospy.timer import Timer
from rospy.rostime import Duration
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from skyscanner.msg import Attitude, EnergyConsumption, InputSequence, UAVState
from skyscanner.msg import UAVStateSequence, Wind, WindSample

PPRZ_SRC = os.getenv("PAPARAZZI_SRC")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python/")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


ABS_TOL = 1e-3
DECIMAL_DIGITS = 3


class PaparazziUAV(object):

    BIND_REGEX = "((^[0-9]* NPS_RATE_ATTITUDE .*)|" \
                 "(^[0-9]* NPS_SPEED_POS .*)|" \
                 "(^[0-9]* NPS_WIND .*)|" \
                 "(^[0-9]* BAT .*))"

    def __init__(self, ac_id, ground_alt=0):
        """Init PaparazziUAV

        Arguments:
            ac_id: aircraft id.
            ground_alt: ground altitude above sea level.
        """
        self.ac_id = ac_id
        self.ground_alt = ground_alt

        self.east = .0
        self.north = .0
        self.up = .0
        self.phi = .0  # in rad
        self.psi = .0  # in rad
        self.theta = .0  # in rad
        self.voltage = 0.  # in V
        self.current = 0.  # in A
        self.power = 0.  # in W
        self.energy = 0.  # in mAh

        self.measured_wind_x = 0.
        self.measured_wind_y = 0.
        self.measured_wind_z = 0.

        self.path_queue = Queue.Queue()  # format: (time, (x, y, z))

        self._last_log = 0
        self._log_interval = 1

        # initiate ivy message interface to catch all messages
        self.ivy_mes = IvyMessagesInterface()
        self.ivy_mes.subscribe(self.ivy_callback, PaparazziUAV.BIND_REGEX)

        rospy.init_node('paparazziuav')

        self.ros_pub_state = rospy.Publisher('uav_state', UAVState,
                                             queue_size=1)
        self.ros_pub_measured_wind = rospy.Publisher('measured_wind',
                                                     WindSample, queue_size=1)
        self.ros_pub_energy_consumption = rospy.Publisher(
            'energy_consumption', EnergyConsumption, queue_size=1)

        rospy.Subscriber("/tick", Clock, self.on_tick_recv, queue_size=10)

        rospy.Subscriber('expected_uav_state_sequence', UAVStateSequence,
                         self.on_exp_uav_state_seq_recv)

        # TODO: Handle connection and disconnection of simulator.
        # With this architecture publish_state will send values even when the
        # simulator was disconnected.

    def on_tick_recv(self, tick):
        """On new time slot beginning.

        Arguments:
            tick: time-slot time.
        """
        self.tick_time = tick.clock.to_sec()

        self.execute_path(method="SEGMENT")
        self.publish_state()
        self.publish_energy_consumption()
        self.publish_wind_sample()

    def ivy_callback(self, ac_id, msg):
        """Handle state callback from ivy bus.

        Arguments:
            ac_id: aircraft id.
            msg: PprzMessage.
        """
        if ac_id != self.ac_id:
            return

        if msg.msg_class == "telemetry":
            if msg.name == "NPS_RATE_ATTITUDE":
                self.phi = float(msg.phi)
                self.psi = float(msg.psi)
                self.theta = float(msg.theta)
            elif msg.name == "NPS_SPEED_POS":
                # NPS_SPEED_POS is sent in NED coordinates and we use ENU
                self.east = float(msg.ltpp_y)
                self.north = float(msg.ltpp_x)
                self.up = -float(msg.ltpp_z)
            elif msg.name == "NPS_WIND":
                # TODO Check if wind needs also to be converted to ENU
                self.measured_wind_x = float(msg.vy)
                self.measured_wind_y = float(msg.vx)
                self.measured_wind_z = -float(msg.vz)
            elif msg.name == "BAT":
                self.voltage = int(msg.voltage)/10.
                self.current = int(msg.amps)/100.
                self.power = self.voltage * self.current
                self.energy = float(msg.energy)

    def publish_state(self):
        """Publish state into ROS topic."""
        # Increment time in 1 to be consistent with simuav behavior which sends
        # the state at the *end* of the cycle, so tick_time + 1
        h = Header(stamp=rospy.Time(self.tick_time + 1))
        position = Point(self.east, self.north, self.up)
        attitude = Attitude(self.phi, self.psi, self.theta)
        battery_level = 100000  # np.NaN
        uav_state = UAVState(h, self.ac_id, position, attitude, battery_level)

        rospy.loginfo("[pprzuav] [ac_id={}]\n{}".format(self.ac_id, uav_state))
        self.ros_pub_state.publish(uav_state)

    def publish_energy_consumption(self):
        """Publish power information into ROS topic."""
        # Increment time in 1 to be consistent with simuav behavior which sends
        # the state at the *end* of the cycle, so tick_time + 1
        h = Header(stamp=rospy.Time(self.tick_time + 1))
        energy_con = EnergyConsumption(
            h, self.ac_id, self.voltage, self.current, self.power, self.energy)
        rospy.logdebug("[pprzuav] [ac_id={}] state:\n{}".format(self.ac_id,
                                                                energy_con))
        self.ros_pub_energy_consumption.publish(energy_con)

    def publish_wind_sample(self):
        """Publish wind sample into ROS topic."""
        h = Header(stamp=rospy.Time(self.tick_time))
        position = Point(float(self.east), float(self.north), float(self.up))
        wind = Wind(float(self.measured_wind_x),  float(self.measured_wind_y),
                    float(self.measured_wind_z))
        wind_sample = WindSample(h, self.ac_id, position, wind)

        rospy.loginfo("[pprzuav] [ac_id={}] wind:\n{}".format(self.ac_id,
                                                              wind_sample))
        self.ros_pub_measured_wind.publish(wind_sample)

    def execute_path(self, method="GOTO_WP"):
        """Execute a path in paparazzi using different methods.

        Arguments:
            method (str): type of message to use. Default "GOTO_WP".
                "GOTO_WP": MISSION_GOTO_WP message
                "SEGMENT": MISSION_SEGMENT message
        """

        if method == "GOTO_WP":
            self._execute_path_goto_wp()
        elif method == "SEGMENT":
            self._execute_path_segment()

    def _execute_path_segment(self):
        """Execute path as segments."""
        def get_until(queue, time):
            """Generator returning all elements in queue until time.

            Arguments:
                time: time.
            """
            try:
                e = queue.get(False)
                while np.around(e[0] - time, decimals=DECIMAL_DIGITS) <= 0:
                    yield e
                    e = queue.get(False)
                else:
                    with queue.mutex:
                        queue.queue.appendleft(e)
            except Queue.Empty:
                return

        # straight line by default
        def_s = (self.tick_time, (self.east, self.north, self.up))
        def_d = (self.east + 15 * np.cos(self.psi),
                 self.north + 15 * np.sin(self.psi))

        mode = 0  # first command is appended
        # mode = 3  # first command replaces all others
        s = None
        for p in get_until(self.path_queue, np.inf):
            if not s:
                s = p
            else:
                self.mission_segment((s[1][0], s[1][1]), (p[1][0], p[1][1]),
                                     self.ground_alt + p[1][2],
                                     insert_mode=mode)
                msg = "".join(["[ac_id={}] Sending segment {} -> {} ",
                               " with alt. {} expected for t={}"])
                msg = msg.format(self.ac_id, s[1][:2], p[1][:2], p[1][2],
                                 (s[0], p[0]))
                rospy.loginfo(msg)
                s = p
                mode = 0  # the following are appended
                time.sleep(0.1)

    def _execute_path_goto_wp(self):
        """Execute path as waypoints."""
        # default destination
        dest = (0, (0, 0, self.ground_alt + 200))  # (time, (east, north, up))

        try:
            p = self.path_queue.get(False)
            while np.around(p[0], decimals=DECIMAL_DIGITS) < self.tick_time:
                p = self.path_queue.get(False)
            if np.isclose(p[0], self.tick_time, atol=ABS_TOL):
                rospy.loginfo('[ac_id={}] Got wp {} for time t={}'.format(
                    self.ac_id, p[1], p[0]))
                dest = p
                try:
                    way_p = dest[1]
                    self.mission_goto_wp(way_p[0], way_p[1],
                                         self.ground_alt + way_p[2],
                                         insert_mode=0)
                    while True:
                        dest = self.path_queue.get(False)
                        rospy.loginfo("dest {} ".format(str(dest)) +
                                      "t={}".format(self.tick_time))
                        way_p = dest[1]
                        self.mission_goto_wp(way_p[0], way_p[1],
                                             self.ground_alt + way_p[2],
                                             insert_mode=0)
                except Queue.Empty:
                    rospy.loginfo("All sent")
            else:
                # Put again the element on the queue and warn
                with self.path_queue.mutex:
                    self.path_queue.queue.appendleft(p)
                rospy.logwarn("[ac_id={}] No position ".format(self.ac_id) +
                              "defined for t={}. ".format(self.tick_time) +
                              "Fallback to default")
                # keep default destination
        except Queue.Empty:
            rospy.logwarn("[ac_id={}] ".format(self.ac_id) +
                          "Empty path queue at t={}. ".format(self.tick_time) +
                          "Fallback to default")
            # keep default destination
        #
        # way_p = dest[1]
        # self.ac_goto_wp(way_p[0], way_p[1], way_p[2], 5)

    def mission_goto_wp(self, east, north, up, duration=0, insert_mode=0):
        """Send a MISSION_GOTO_WP command to paparazzi.

        Arguments:
            east: position east
            north: position north
            up: position up (in meters above sea level)
            duration: duration of mission before being discarded. Default is 0,
                meaning no specified duration.
            insert_mode: default APPEND
                0: APPEND
                1: PREPEND
                2: REPLACE_CURRENT
                3: REPLACE_ALL
        """
        msg = PprzMessage("datalink", "MISSION_GOTO_WP")
        msg['ac_id'] = self.ac_id
        # APPEND, PREPEND, REPLACE_CURRENT, REPLACE_ALL
        msg['insert'] = insert_mode
        # ENU to NED coordinates
        msg['wp_east'] = east
        msg['wp_north'] = north
        msg['wp_alt'] = up
        msg['duration'] = duration
        self.ivy_mes.send_raw_datalink(msg)

    def mission_segment(self, s_pos, d_pos, altitude,
                        duration=0, insert_mode=0):
        """Send a MISSION_SEGMENT command to paparazzi.

        Arguments:
            s_pos: source position as tuple (east, north)
            d_pos: destination position as tuple (east, north)
            altitude: altitude of the segment (in meters above sea level)
            duration: duration of mission (in seconds) before being discarded.
                Default is 0, meaning no specified duration.
            insert_mode: default APPEND
                0: APPEND
                1: PREPEND
                2: REPLACE_CURRENT
                3: REPLACE_ALL
        """
        msg = PprzMessage("datalink", "MISSION_SEGMENT")
        msg['ac_id'] = self.ac_id
        # APPEND, PREPEND, REPLACE_CURRENT, REPLACE_ALL
        msg['insert'] = insert_mode
        msg['segment_east_1'] = s_pos[0]
        msg['segment_north_1'] = s_pos[1]
        msg['segment_east_2'] = d_pos[0]
        msg['segment_north_2'] = d_pos[1]
        msg['segment_alt'] = altitude
        msg['duration'] = duration
        self.ivy_mes.send_raw_datalink(msg)

    def on_exp_uav_state_seq_recv(self, exp_state_seq):
        """Handle aircraft input reception."""
        rospy.loginfo("[ac_id={}] Received UAV state sequence:\n{}".format(
            exp_state_seq.ac_id, exp_state_seq))

        # Fill queue
        for i in exp_state_seq.states:
            self.path_queue.put((i.header.stamp.to_sec(), (i.position.x,
                                 i.position.y, i.position.z)))


def main():
    def shutdown_hook():
        if uav and uav.ivy_mes:
            uav.ivy_mes.shutdown()

    # Setup math precision
    global ABS_TOL, DECIMAL_DIGITS
    ABS_TOL = rospy.get_param(rospy.search_param('abs_tol'), 1e-3)
    DECIMAL_DIGITS = int(np.around(np.log10(1./ABS_TOL)))

    ac_id = rospy.get_param('ac_id')
    ground_alt = rospy.get_param(rospy.search_param('ground_alt'))

    uav = PaparazziUAV(ac_id, ground_alt)

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    main()
