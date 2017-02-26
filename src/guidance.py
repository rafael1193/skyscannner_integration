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
from abc import ABCMeta, abstractmethod, abstractproperty

import Queue

from collections import deque
from threading import Lock

import numpy as np

import rospy
from rospy.timer import Timer
from rospy.rostime import Duration
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock
from skyscanner.msg import Attitude, Guidance, TrajectorySequence, UAVState

from utilities.geometry import heading_of_vector, wrap_angle_rad
from utilities.guidance import Curve, Circumference, Segment, Waypoint
from utilities.guidance import StepGuidance, RampGuidance
from utilities.guidance import PLOSGuidance, VFMathGuidance


ABS_TOL = 1e-3
DECIMAL_DIGITS = 3


class GuidanceNode(object):

    def __init__(self):
        """Init Guidance.

        Arguments:
            ac_ids: list of ac_id
        """
        rospy.init_node('guidance')

        self.ac_id = 1
        self.tick_time = 0

        self.path = deque([])
        self.next_path = deque([])
        self.path_update_lock = Lock()

        # Circumferences
        # self.path.append(Circumference(np.array([0, 0, 1000]), 200))
        # self.path.append(Circumference(np.array([400, 0, 1000]), -200))
        # self.path.append(Circumference(np.array([800, 0, 1000]), 200))

        # Smooth square
        # self.path.append(Segment(np.array([0, 100, 1000]),
        #                          np.array([0, 900, 1000])))
        # self.path.append(Circumference(np.array([100, 900, 1000]), 100))
        # self.path.append(Segment(np.array([100, 1000, 1000]),
        #                          np.array([900, 1000, 1000])))
        # self.path.append(Circumference(np.array([900, 900, 1000]), 100))
        # self.path.append(Segment(np.array([1000, 900, 1000]),
        #                          np.array([1000, 100, 1000])))
        # self.path.append(Circumference(np.array([900, 100, 1000]), 100))
        # self.path.append(Segment(np.array([900, 0, 1000]),
        #                          np.array([100, 0, 1000])))
        # self.path.append(Circumference(np.array([400, 400, 1000]), 150,
        #                                timeout=2000))
        # seg = Segment(np.array([1156.84, -1015.70, 1000]),
        #               np.array([1153.67, -850.78, 1000]),
        #               timeout=2000)
        # seg = Segment(np.array([600, 600, 1000]),
        #               np.array([800, 800, 1000]),
        #               timeout=20000)
        # self.path.append(seg)

        guider_class = VFMathGuidance
        guider_class_name = str(guider_class.__name__)

        # Get all the parameters defined with the same base name as the
        # guidance class
        guider_params = rospy.get_param(rospy.search_param(guider_class_name))

        rospy.logwarn(guider_class_name)
        rospy.logwarn(guider_params)

        self.guidance = guider_class(self.path, fallback_curve=None,
                                     **guider_params)

        # self.guidance = RampGuidance(self.path)
        # self.guidance = StepGuidance(self.path, np.pi/4, 0.0)

        self.position = np.array([0, 0, 0])

        self.heading_setpoint = 0
        self.desired_altitude = 1000

        rospy.Subscriber('/tick', Clock, self.on_tick_recv, queue_size=10)

        rospy.Subscriber('/fast_tick', Clock, self.on_fast_tick_recv,
                         queue_size=10)

        rospy.Subscriber('uav_state', UAVState, self.on_uav_state_recv,
                         queue_size=10)
        rospy.Subscriber('uav_state', UAVState,
                         self.on_expected_uav_state_recv, queue_size=10)
        rospy.Subscriber('trajectory_sequence', TrajectorySequence,
                         self.on_trajectory_recv, queue_size=10)

        self.ros_pub_guidance = rospy.Publisher('guidance', Guidance,
                                                queue_size=1)
        self.ros_pub_exp_guidance = rospy.Publisher('expected_uav_guidance',
                                                    UAVState, queue_size=1)

    def publish_curve(self, curve):
        """Publish the trace of a curve as expected position points."""
        for tp in curve.trace:
            self.publish_expected_position(tp)

    def on_tick_recv(self, tick):
        """On new time slot beginning.

        Arguments:
            tick: time-slot time.
        """
        pass

    def on_fast_tick_recv(self, tick):
        """On new time slot beginning.

        Arguments:
            tick: time-slot time.
        """
        self.tick_time = tick.clock.to_sec()
        self.path_update_lock.acquire()
        guid_angle = self.guidance.compute_angle(self.position, self.tick_time)
        self.heading_setpoint = guid_angle
        rospy.loginfo("[guidance] heading_setpoint: {}".format(
            self.heading_setpoint*180/np.pi))
        self.path_update_lock.release()

        rospy.loginfo("ang: {}".format(self.heading_setpoint))

        self.publish_guidance(1)

    def on_trajectory_recv(self, trajectory_seq):
        """Handle TrajectorySequence message reception."""
        self.next_path.clear()
        discard_type = [Curve.DISCARD_TIMELIMIT, Curve.DISCARD_CLOSETO]
        for trajectory in trajectory_seq.trajectories:
            if trajectory.circle is True:
                cir = Circumference(np.array([trajectory.origin.x,
                                              trajectory.origin.y,
                                              trajectory.origin.z]),
                                    -trajectory.radius,
                                    timeout=trajectory.duration,
                                    time_limit=trajectory.time_limit,
                                    discard_type=discard_type)
                self.next_path.append(cir)
            else:
                lin = Segment(np.array([trajectory.origin.x,
                                        trajectory.origin.y,
                                        trajectory.origin.z]),
                              np.array([trajectory.destination.x,
                                        trajectory.destination.y,
                                        trajectory.destination.z]),
                              timeout=trajectory.duration,
                              time_limit=trajectory.time_limit,
                              discard_type=discard_type)
                self.next_path.append(lin)

        rospy.loginfo("[guidance] path: {}".format(self.path))
        rospy.loginfo("[guidance] next_path: {}".format(self.next_path))

        for c in self.next_path.__copy__():
            self.publish_curve(c)
        self.path.extend(self.next_path)

    def on_uav_state_recv(self, uav_state):
        """Handle UAV state message reception.

        Arguments:
            uav_state: UAVState message.
            ac_id: aircraft id.
        """
        self.position = np.array([uav_state.position.x,
                                  uav_state.position.y,
                                  uav_state.position.z])

    def on_expected_uav_state_recv(self, exp_uav_state):
        """Handle UAV expected state message reception.

        Arguments:
            uav_state: UAVState message.
            ac_id: aircraft id.
        """
        self.desired_altitude = exp_uav_state.position.z

    def publish_expected_position(self, position):
        """Publish expected position into ROS topic."""
        # Increment time in 1 to be consistent with simuav behavior which sends
        # the state at the *end* of the cycle, so tick_time + 1
        h = Header(stamp=rospy.Time(self.tick_time + 1))
        position = Point(position[0], position[1], position[2])
        # psi converted to NED?
        attitude = Attitude(np.NaN,
                            wrap_angle_rad(self.heading_setpoint - np.pi),
                            np.NaN)
        battery_level = np.NaN
        uav_state = UAVState(h, self.ac_id, position, attitude, battery_level)

        rospy.loginfo("[fguav] [ac_id={}]\n{}".format(self.ac_id, uav_state))
        self.ros_pub_exp_guidance.publish(uav_state)

    def publish_guidance(self, ac_id):
        """Publish guidance.

        Arguments:
            ac_id: ac_id
        """
        heading = wrap_angle_rad(self.heading_setpoint)

        h = Header(stamp=rospy.Time(self.tick_time + 1))
        guid = Guidance(h, ac_id, heading, self.desired_altitude)

        rospy.loginfo("[guidance] [ac_id={}]\n{}".format(self.ac_id, guid))
        self.ros_pub_guidance.publish(guid)


def main():
    def shutdown_hook():
        pass

    # Setup math precision
    # global ABS_TOL, DECIMAL_DIGITS
    # ABS_TOL = rospy.get_param(rospy.search_param('abs_tol'), 1e-3)
    # DECIMAL_DIGITS = int(np.around(np.log10(1./ABS_TOL)))

    guidance = GuidanceNode()

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()


if __name__ == '__main__':
    main()
