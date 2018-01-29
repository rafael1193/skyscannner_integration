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

from Queue import Queue, Empty

import numpy as np
import rospy

from utilities.guidance import Segment, Circumference

from std_msgs.msg import Header
from skyscanner.msg import UAVState, UAVStats, TrajectorySequence

from utilities.attr_dict import AttrDict


ABS_TOL = 1e-3
DECIMAL_DIGITS = 3

USE_MATH_TRAJ = True
actual_traj = None


def main():

    def state_attrdict(uav_state_msg):
        e = AttrDict()
        e.ac_id = uav_state_msg.ac_id
        e.time = uav_state_msg.header.stamp.to_sec()
        e.recv_time = rospy.get_rostime().to_sec()
        e.position = (uav_state_msg.position.x,
                      uav_state_msg.position.y,
                      uav_state_msg.position.z)
        e.attitude = (uav_state_msg.attitude.phi,
                      uav_state_msg.attitude.psi,
                      uav_state_msg.attitude.theta)
        return e

    def compute_stats(state, expected_state):
        """Compute errors between corresponding UAV states.

        Arguments:
            state:
            expected_state:

        Rerturns:
            skyscanner.msg.UAVStats object.
        """
        def angle_diff(a, b):
            """Calculate the distance between two angles in degrees.

            Arguments:
                a: measured angle in degrees
                b: reference angle in degrees

            Returns:
                Distance between a and b including sign
            """
            phi = np.abs(a - b) % 360
            sign = 1

            if not ((a - b >= 0 and a - b <= 180) or (
                    a - b <= -180 and a - b >= -360)):
                sign = -1
            if phi > 180:
                result = 360 - phi
            else:
                result = phi

            return result * sign

        stats = UAVStats()
        stats.header = Header(stamp=rospy.Time(state.time))
        stats.ac_id = state.ac_id
        stats.heading_error = \
            angle_diff(state.attitude[1]*np.pi/180,
                       expected_state.attitude[1]*np.pi/180) * 180/np.pi
        stats.altitude_error = state.position[2] - expected_state.position[2]
        p1 = np.array(state.position[:2])
        p2 = np.array(expected_state.position[:2])
        stats.crosstrack_error = np.linalg.norm(p1 - p2)

        return stats

    def get_until(queue, condition):
        """Generator returning all elements in queue until condition is met.

        Arguments:
            queue: Queue.queue
            condition: fn(e) -> True or False where `e` element of queue.
                Must return True while condition is being met, False otherwise.
        """
        try:
            e = queue.get(False)
            while condition(e):
                yield e
                e = queue.get(False)
            else:
                with queue.mutex:
                    queue.queue.appendleft(e)
        except Empty:
            return

    def on_state_recv(state):
        """Handle state reception."""
        ac_id = state.ac_id
        if ac_id not in ac:
            rospy.logwarn("[stats] ac_id '{}' not recognized".format(ac_id))
            return

        if USE_MATH_TRAJ:
            publish_stats_math_traj(state)
        else:
            publish_stats_exp_pos(state)
        # else:

    def publish_stats_math_traj(state):
        global actual_traj
        msg_time = state.header.stamp.to_sec()
        cond = lambda e: msg_time < e.time_limit

        for s in get_until(ac[ac_id].traj_q, cond):
            actual_traj = s

        if actual_traj is None:
            return

        np_pos = np.array([state.position.x,
                           state.position.y,
                           state.position.z])
        stats = UAVStats()
        stats.header = Header(stamp=rospy.Time(msg_time))
        stats.ac_id = state.ac_id
        stats.heading_error = 0
        stats.altitude_error = 0
        stats.crosstrack_error = actual_traj.crosstrack_error(np_pos)[1]

        rospy.loginfo(stats)
        ac[ac_id].stats_pub.publish(stats)

    def publish_stats_exp_pos(state):
        msg_time = state.header.stamp.to_sec()
        cond = lambda e: np.around(msg_time - e.time,
                                   decimals=DECIMAL_DIGITS) >= 0

        # Get first element meeting condition discarding the previous ones
        s = None
        for s in get_until(ac[ac_id].expected_states_q, cond):
            pass

        if s is None:
            return

        if s.time != msg_time:
            rospy.logwarn("[stats] s.time != msg_time '{}!={}'".format(
                s.time, msg_time))
        stats = compute_stats(state_attrdict(state), s)
        rospy.loginfo(stats)
        ac[ac_id].stats_pub.publish(stats)

    def on_expected_state_recv(exp_state):
        """Handle expected state reception."""
        ac_id = exp_state.ac_id
        if ac_id in ac:
            ac[ac_id].expected_states_q.put(state_attrdict(exp_state))
        else:
            rospy.logwarn("[stats] ac_id '{}' not recognized".format(ac_id))

    def on_trajectory_recv(trajectory_seq):
        """Handle TrajectorySequence message reception."""
        print("trajectory recv")
        ac_id = trajectory_seq.ac_id
        if ac_id in ac:
            for trajectory in trajectory_seq.trajectories:
                if trajectory.circle is True:
                    cir = Circumference(np.array([trajectory.origin.x,
                                                  trajectory.origin.y,
                                                  trajectory.origin.z]),
                                        -trajectory.radius,
                                        timeout=trajectory.duration,
                                        time_limit=trajectory.time_limit)
                    ac[ac_id].traj_q.put(cir)
                else:
                    lin = Segment(np.array([trajectory.origin.x,
                                            trajectory.origin.y,
                                            trajectory.origin.z]),
                                  np.array([trajectory.destination.x,
                                            trajectory.destination.y,
                                            trajectory.destination.z]),
                                  timeout=trajectory.duration,
                                  time_limit=trajectory.time_limit)
                    ac[ac_id].traj_q.put(lin)
        else:
            rospy.logwarn("[stats] ac_id '{}' not recognized".format(ac_id))

    def on_shutdown():
        pass

    rospy.init_node('stats')
    rospy.on_shutdown(on_shutdown)

    # Setup math precision
    global ABS_TOL, DECIMAL_DIGITS, actual_traj
    ABS_TOL = rospy.get_param(rospy.search_param('abs_tol'), 1e-3)
    DECIMAL_DIGITS = int(np.around(np.log10(1./ABS_TOL)))

    # rospy.Subscriber('/tick', Clock, on_tick_recv, queue_size=10)

    # Get aircraft namespaces
    aircraft_ns = rospy.get_param('aircrafts_ns')
    ac = {}
    for ns in aircraft_ns:
        ac_id = rospy.get_param(rospy.search_param('/'.join([ns, 'ac_id'])))
        ac[ac_id] = AttrDict()
        ac[ac_id].name = ns
        ac[ac_id].expected_states_q = Queue()
        ac[ac_id].traj_q = Queue()
        ac[ac_id].state_sub = rospy.Subscriber(
            '/'.join([ns, 'uav_state']), UAVState, on_state_recv, queue_size=1)
        ac[ac_id].exp_state_sub = rospy.Subscriber(
            '/'.join([ns, 'expected_uav_state']), UAVState,
            on_expected_state_recv, queue_size=1)
        ac[ac_id].exp_state_sub = rospy.Subscriber(
            '/'.join([ns, 'trajectory_sequence']), TrajectorySequence,
            on_trajectory_recv, queue_size=1)
        ac[ac_id].stats_pub = rospy.Publisher('/'.join([ns, 'stats']),
                                              UAVStats, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()
