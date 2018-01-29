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

import cmath
import socket

import numpy as np

import rospy
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from skyscanner.msg import UAVState
from skyscanner.srv import GetWind

from utilities.attr_dict import AttrDict


class FlightgearEnvironment(object):
    """
    FlightgearEnvironment gets environment data and sends them to Flightgear.
    """

    def __init__(self):
        """Initialize FlightgearEnvironment."""
        # init ROS node
        rospy.init_node('paparazzienvironment')
        rospy.wait_for_service('get_wind')

        self.tick_time = 0

        self.aircrafts = AttrDict()

        # Get aircraft namespaces
        aircraft_ns = rospy.get_param('aircrafts_ns')

        for ac in aircraft_ns:
            ac_id = rospy.get_param(rospy.search_param(
                '/'.join([ac, 'ac_id'])))
            self.aircrafts[ac_id] = AttrDict()
            self.aircrafts[ac_id].name = ac
            self.aircrafts[ac_id].position = np.array([0, 0, 0])
            fg_host = str(rospy.get_param(rospy.search_param(
                '/'.join([ac, 'fg_hostname'])), 'localhost'))
            wind_port_in = rospy.get_param(rospy.search_param(
                '/'.join([ac, 'wind_port_in'])))
            self.aircrafts[ac_id].addr = (fg_host, wind_port_in)
            self.aircrafts[ac_id].udp_send_sock = socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM)
            rospy.Subscriber('/'.join([ac, 'uav_state']), UAVState,
                             self.on_uav_state_recv, queue_size=10)

        # Set up get_wind client
        self.get_wind = rospy.ServiceProxy('get_wind', GetWind)
        self.get_wind.wait_for_service(timeout=10)

        rospy.Subscriber('tick', Clock, self.on_tick_recv, queue_size=10)

    def on_tick_recv(self, tick):
        self.tick_time = tick.clock.to_sec()
        for (ac_id, ac) in self.aircrafts.iteritems():
            pos = Point(ac.position[0], ac.position[1], ac.position[2])
            wind = self.get_wind(ac_id, rospy.Time(secs=self.tick_time),
                                 pos).wind

            # self.send_wind_udp(ac_id, wind.east, wind.north, wind.up)
            if not (np.isnan(wind.east) or np.isnan(wind.north) or (
               np.isnan(wind.up))):
                self.send_wind_udp(ac_id, wind.east, wind.north, wind.up)
            else:
                rospy.logwarn("[fgenvironment] There is a NaN in wind! "
                              "e:{} n:{} u:{}".format(wind.east, wind.north,
                                                      wind.up))
                self.send_wind_udp(ac_id, 0, 0, 0)

    def on_uav_state_recv(self, uav_state):
        """Handle expected UAV state message reception.

        Arguments:
            expected_uav_state: UAVState message.
        """
        ac_id = uav_state.ac_id
        self.aircrafts[ac_id].position = np.array([uav_state.position.x,
                                                   uav_state.position.y,
                                                   uav_state.position.z])

    def send_wind_udp(self, ac_id, wind_east, wind_north, wind_up):
        """Send wind to flightgear using udp.

        Arguments:
            ac_id: aircraft id
            wind_east:
            wind_north:
            wind_up:
        """
        try:
            msg = '|'.join([str(wind_north), str(wind_east), str(-wind_up),
                            "1", "0", "0"])
            # See Protocol/skyscanner_wind.xml for the message format
            msg = ''.join([msg, '\n'])
            ac = self.aircrafts[ac_id]
            ac.udp_send_sock.sendto(msg, ac.addr)
        except socket.error as a:
            raise


def main():

    def shutdown_hook():
        pass

    rospy.on_shutdown(shutdown_hook)

    fg_env = FlightgearEnvironment()

    rospy.spin()


if __name__ == '__main__':
    main()
