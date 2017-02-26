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

import threading
import time

import rospy
from rospy.timer import Timer
from rosgraph_msgs.msg import Clock
from skyscanner.msg import ClockControl


class PeriodicClock(threading.Thread):
    def __init__(self, action, step=0.001):
        """Init PeriodicClock.

        Arguments:
            action: callback fn(time_float)
            step: clock resolution
        """
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()
        self.step = step
        self.action = action
        self._initial_time = 0.
        self._held = threading.Event()
        self._hold_time = 0.

    def run(self):
        self._initial_time = time.time()
        while not self.stop_event.is_set():
            if not self._held.is_set():
                self.action(time.time() - self._initial_time)
            self.stop_event.wait(self.step)

    def stop(self):
        """Stop the clock and reset it."""
        self.stop_event.set()
        self._initial_time = 0.

    def hold(self):
        """Hold clock avance."""
        self._hold_time = time.time()
        self._held.set()

    def unhold(self):
        """Let the clock continue."""
        self._initial_time = self._initial_time + (
            time.time() - self._hold_time)
        self._held.clear()

    def is_held(self):
        """Check if clock is being hold.

        Returns:
            bool: whether the clock is running or not
        """
        return self._held.isSet()


class TickGenerator(object):

    def __init__(self, duration=1, topic='tick'):
        self.topic_name = topic
        self.publisher = rospy.Publisher(topic, Clock, queue_size=10)
        self.duration = rospy.Duration(duration)
        self.timer = Timer(self.duration, self.send_tick)

    def send_tick(self, timer_event):
        """Tell all concerned nodes that a new cycle has begun."""
        rospy.loginfo("{} at {}s and {}ns".format(
            self.topic_name,
            timer_event.current_expected.secs,
            timer_event.current_expected.nsecs))
        self.publisher.publish(timer_event.current_expected)

    def start(self):
#        self.timer.start()
        pass


def main():
    def send_time(elapsed_time):
        """Publish time.

        Arguments:
            elapsed_time: actual time
        """
        ros_clock_pub.publish(rospy.Time(elapsed_time * time_scale))

    def on_clock_control_recv(clock_control):
        """Control clock time evolution.

        Arguments:
            clock_control: ClockControl message
        """
        if clock_control.hold:
            if clock.is_held():
                rospy.logwarn("Tried to hold clock, but it was already held")
            else:
                clock.hold()
        else:  # unhold request
            if clock.is_held():
                clock.unhold()
            else:
                rospy.logwarn("Tried to unhold clock, but it wasn't held")

    def on_shutdown():
        rospy.loginfo("clock shutdown")
        clock.stop()
        clock.join()
        tick_timer.timer.shutdown()
        fast_tick_timer.timer.shutdown()

    time_scale = rospy.get_param('clock_time_scale')

    rospy.init_node('clock_generator')
    rospy.on_shutdown(on_shutdown)

    ros_clock_pub = rospy.Publisher('clock', Clock, queue_size=10)
    rospy.Subscriber("clock_control", ClockControl,
                     on_clock_control_recv, queue_size=1)

    clock = PeriodicClock(send_time)
    fast_tick_timer = TickGenerator(duration=0.02, topic='fast_tick')
    tick_timer = TickGenerator(duration=1, topic='tick')
    clock.start()
    fast_tick_timer.start()
    tick_timer.start()

    rospy.spin()


if __name__ == '__main__':
    main()
