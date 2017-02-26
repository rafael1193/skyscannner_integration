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

import cmath
import os
import socket
import struct
import sys
import time

import rospy
from geometry_msgs.msg import Point
from skyscanner.srv import GetWind

PPRZ_SRC = os.getenv("PAPARAZZI_SRC")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python/")
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


class PaparazziEnvironment(object):
    """
    PaparazziEnvironment holds environment data (eg. wind)
    and sends them to paparazzi.
    """

    BIND_REGEX = "(^[0-9]* NPS_SPEED_POS .*)"

    def __init__(self, nps_addr=None, time_scale=1):
        """
        Initialize PaparazziEnvironment.

        Arguments:
            nps_addr: (optional) key-value pairs of ac_id and corresponding
                (host, port). If None, use Ivy bus instead of UDP datagrams.
            time_scale: simulation speed multiplier (default 1)
        """
        # init ROS node
        rospy.init_node('paparazzienvironment')
        rospy.wait_for_service('/get_wind')

        self.ivy_msg = None
        self.nps_udp_socket = None

        self.time_scale = time_scale
        if nps_addr is not None:
            self.use_udp = True
            self.nps_addr = nps_addr
            self.start_time = rospy.get_time()
            self.wind_udp_packer = struct.Struct('d f f f f f I')
            self.nps_udp_socket = socket.socket(socket.AF_INET,
                                                socket.SOCK_DGRAM)
        else:
            self.use_udp = False

        # Set up get_wind client
        self.get_wind = rospy.ServiceProxy('/get_wind', GetWind)
        self.get_wind.wait_for_service(timeout=10)

        # initiate ivy message interface catching position messages
        self.ivy_msg = IvyMessagesInterface(start_ivy=True, verbose=False)
        self.ivy_msg.subscribe(self.on_ivy_msg_recv,
                               PaparazziEnvironment.BIND_REGEX)
        self.init_time = rospy.Time.now()

        self._last_log = 0
        self._log_interval = 1  # in seconds

    def on_ivy_msg_recv(self, ac_id, msg):
        """Handle state callback from ivy bus.

        Arguments:
            ac_id: aircraft id.
            msg: PprzMessage.
        """
        if msg.msg_class == "telemetry" and msg.name == "NPS_SPEED_POS":
            x = float(msg.ltpp_x)
            y = float(msg.ltpp_y)
            z = float(msg.ltpp_z)
            t = rospy.Time.now() - self.init_time
            w = self.get_wind(ac_id, t, Point(x, y, z)).wind

            if self.use_udp:
                self.send_wind_udp(ac_id, w.east, w.north, w.up)
            else:
                self.send_wind_ivy(w.east, w.north, w.up, self.time_scale)

    def send_wind_ivy(self, wind_east, wind_north, wind_up, time_scale):
        """Send wind to paparazzi in a ground.WORLD_ENV message using ivy."""
        msg = PprzMessage('ground', 'WORLD_ENV')
        msg['wind_east'] = wind_east
        msg['wind_north'] = wind_north
        # TODO: Check whether sign inversion below is correct or not
        msg['wind_up'] = - wind_up
        msg['ir_contrast'] = 400  # preset value
        msg['time_scale'] = time_scale
        msg['gps_availability'] = 1  # preset value
        self.ivy_msg.send(msg)

        # log wind message
        if self._last_log + self._log_interval <= time.time():
            self._last_log = time.time()
            log_str = str(msg)
            rospy.loginfo(log_str)

    def send_wind_udp(self, ac_id, wind_east, wind_north, wind_up):
        """Send wind to paparazzi simulator using udp."""
        # TODO: Allow time_scale control when using UDP
        if ac_id not in self.nps_addr:
            rospy.logerr("No (host, port) defined for ac_id {}".format(ac_id))
        elapsed = rospy.get_time() - self.start_time
        (speed, heading) = cmath.polar(wind_north + wind_east*1j)
        values = (elapsed, wind_north, wind_east, wind_up, heading, speed,
                  0x12345678)
        message = self.wind_udp_packer.pack(*values)
        rospy.loginfo(values)
        rospy.loginfo(self.nps_addr[ac_id])
        self.nps_udp_socket.sendto(message, self.nps_addr[ac_id])


def main():

    pprz_evn = None

    def shutdown_hook():
        if not pprz_evn:
            return
        elif pprz_evn.ivy_msg:
            pprz_evn.ivy_msg.shutdown()
        elif pprz_evn.nps_udp_socket:
            pprz_evn.nps_udp_socket.close()

    rospy.on_shutdown(shutdown_hook)

    # Change time scale if available
    clock_time_scale = 1
    if rospy.has_param('clock_time_scale'):
        clock_time_scale = rospy.get_param('clock_time_scale')

    aircraft_ns = rospy.get_param('aircrafts_ns')
    nps_addr = {}

    for ac in aircraft_ns:
        ac_id = rospy.get_param('/'.join([ac, 'ac_id']))
        host = rospy.get_param('/'.join([ac, 'nps_host']), '127.0.0.1')
        port = rospy.get_param('/'.join([ac, 'nps_port']))
        nps_addr[ac_id] = (host, port)

    pprz_evn = PaparazziEnvironment(nps_addr=nps_addr,
                                    time_scale=clock_time_scale)
    rospy.spin()


if __name__ == '__main__':
    main()
