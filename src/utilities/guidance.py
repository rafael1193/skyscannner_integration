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

import numpy as np
import numpy.linalg as npla

import rospy

from utilities.geometry import heading_of_vector


class Curve(object):
    """Abstract class defining a curve."""

    __metaclass__ = ABCMeta

    DISCARD_TIMEOUT = "timeout"
    DISCARD_TIMELIMIT = "timelimit"
    DISCARD_CLOSETO = "closetonext"

    def __init__(self, timeout=60, time_limit=np.inf, approaching_distance=25,
                 discard_type=[DISCARD_TIMEOUT, DISCARD_CLOSETO]):
        """Init Curve.

        Arguments:
            timeout: in seconds (Default 60)
            time_limit: time after it, the curve shouldn't be used anymore.
        """
        self.timeout = timeout
        self.time_limit = time_limit
        self.discard_type = discard_type
        self.approaching_distance = approaching_distance

    def orientation(self):
        """Direction to follow the curve for circular ones.

        Positive or negative definition must be decided for each type of curve

        Returns:
            1  if positive orientation
            -1 if negative orientation
        """
        return 1

    @abstractmethod
    def crosstrack_error(self, pos):
        """Calculate the euclidean crosstrack error with respect to pos.

        Arguments:
            pos: position of the UAV.

        Returns:
            d_u: unitary vector indicating crosstrack direction.
            d: absolute value of the crosstrack error.
        """
        pass

    @abstractmethod
    def los_vector(self, pos):
        """Get the Line Of Sight vector for pos.

        Arguments:
            pos: position of the UAV.

        Returns:
            los_u: unitary line of sight vector.
            los_d: distance to destination.
        """
        pass

    @abstractproperty
    def trace(self):
        """Get curve trace as sequence of positions.

        Returns:
            Sequence of points forming the trace of the curve.
        """
        return tuple()

    def __repr__(self):
        return "%s(%r)" % (self.__class__, self.__dict__)


class Waypoint(Curve):

    def __init__(self, dest, **kwargs):
        super(Waypoint, self).__init__(**kwargs)
        self._destination = dest

    @property
    def destination(self):
        return self._destination

    def crosstrack_error(self, pos):
        d_v = self.destination - pos
        d = npla.norm(self.destination - pos)
        d_u = d_v / d

        return d_u, d

    def los_vector(self, pos):
        los_v = self.destination - pos
        los_d = npla.norm(los_v)
        los_u = los_v / los_d
        return los_u, los_d

    @property
    def trace(self):
        return (self._destination,)


class Segment(Curve):

    def __init__(self, orig, dest, **kwargs):
        super(Segment, self).__init__(**kwargs)
        self._origin = orig
        self._destination = dest
        u = self._destination - self._origin
        self._dir = u / npla.norm(u)

    @property
    def destination(self):
        return self._destination

    @property
    def origin(self):
        return self._origin

    @property
    def dir(self):
        return self._dir

    def crosstrack_error(self, pos):
        # As it should be a segment, we need to take in account orig and dest
        # points. If not, the UAV will continue straight past the end.

        # IT WORKS ONLY WITH SEGMENTS WITH POSITIVE PENTS

        cond_o = pos[0] < self.origin[0] and pos[1] < self.origin[1]
        cond_d = pos[0] > self.destination[0] or pos[1] > self.destination[1]
        # If p_x < orig_x AND p_y < orig_y, go to orig.
        if cond_o:
            d_v = self.origin - pos
            d = npla.norm(self.origin - pos)
            d_u = d_v / d
            return d_u, d
        # If p_x < dest_x OR p_y < dest_y, go to dest.
        if cond_d:
            d_v = self.destination - pos
            d = npla.norm(self.destination - pos)
            d_u = d_v / d
            return d_u, d
        # Else (which means the area comprised by Domain and the Image of the
        # segment) follow the line.
        else:
            # There are two perpendicular vectors. The one pointing to the
            # segment should make pos to move closer to it
            d_v1 = np.array([-self.dir[1], self.dir[0], self.dir[2]])
            d1 = self._crosstrack_error_distance(pos + d_v1)
            d_v2 = np.array([self.dir[1], -self.dir[0], self.dir[2]])
            d2 = self._crosstrack_error_distance(pos + d_v2)

            if d1 < d2:
                return d_v1, d1
            else:
                return d_v2, d2

            # Schema:
            #  >>
            # ---------d  <<
            #         /|
            #        / |  ^^
            #  line /  |
            #      /   |  ^^
            #     /    |
            #    / line|
            # --o      |
            # ^^|      |

    def _crosstrack_error_distance(self, pos):
        num = npla.norm(np.cross(pos - self.destination, self.dir))
        den = npla.norm(self.dir)
        return num / den

    def los_vector(self, pos):
        los_v = self.destination - pos
        los_d = npla.norm(los_v)
        los_u = los_v / los_d
        return los_u, los_d

    @property
    def orientation(self):
        """Direction to follow the curve for circular ones.

        Returns:
            1  if radius is positive
            -1 if radius is negative
        """
        return 1

    def phi(self, pos):
        """Get the tracking error measured as the result of applying f(pos)
        where f is the curve."""
        # If p_x < orig_x AND p_y < orig_y, go to orig.
        if pos[0] < self.origin[0] and pos[1] < self.origin[1]:
            x1, y1 = self.origin[0:2]
            x, y = pos[0:2]
            rospy.logwarn("orig")
            return (x-x1)*(x-x1)+(y-y1)*(y-y1)  # Like a zero radius circ
        # If p_x < dest_x OR p_y < dest_y, go to dest.
        if pos[0] > self.destination[0] or pos[1] > self.destination[1]:
            x1, y1 = self.destination[0:2]
            x, y = pos[0:2]
            rospy.logwarn("dest")
            return (x-x1)*(x-x1)+(y-y1)*(y-y1)  # Like a zero radius circ
        # Else (which means the area comprised by Domain and the Image of the
        # segment) follow the line.
        else:
            x1, y1 = self.origin[0:2]
            x2, y2 = self.destination[0:2]
            x, y = pos[0:2]
            rospy.logwarn("line")
            return (x1-x2)*y+(y2-y1)*x+y1*x2-y2*x1

    def grad_phi(self, pos):
        """Get the gradient of the curve."""
        if pos[0] < self.origin[0] and pos[1] < self.origin[1]:
            x = pos[0] - self.origin[0]
            y = pos[1] - self.origin[1]
            return np.array([2 * x, 2 * y, 0])
        if pos[0] > self.destination[0] or pos[1] > self.destination[1]:
            x = pos[0] - self.destination[0]
            y = pos[1] - self.destination[1]
            return np.array([2 * x, 2 * y, 0])
        else:
            x1, y1 = self.origin[0:2]
            x2, y2 = self.destination[0:2]
            return np.array([y2 - y1, x1 - x2, 0])

    @property
    def trace(self):
        return (self._origin, self._destination)


class Circumference(Curve):

    def __init__(self, center, radius, **kwargs):
        super(Circumference, self).__init__(**kwargs)
        self._center = center
        self._radius = radius  # +: ccw, -: cw
        if self._radius >= 0:
            self._rot = np.array([0., 0., 1.])  # rotate ccw
        else:
            self._rot = np.array([0., 0., -1.])  # rotate cw

        # TODO: self._rot defines the orientation plane which the
        # circumference belongs to.

    @property
    def orientation(self):
        """Direction to follow the curve for circular ones.

        Returns:
            1  if radius is positive
            -1 if radius is negative
        """
        if self.radius >= 0:
            return -1
        else:
            return 1

    @property
    def center(self):
        return self._center

    @property
    def radius(self):
        return self._radius

    def crosstrack_error(self, pos):
        # We take the projection of the UAV position on the circle plane
        pos[2] = self.center[2]
        pc_vec = self.center - pos
        pc_norm = npla.norm(pc_vec)
        r_vec = pc_vec / pc_norm * np.abs(self.radius)
        d_vec = pc_vec - r_vec
        d = npla.norm(d_vec)
        return d_vec / d, d

    def los_vector(self, pos):
        # We take the projection of the UAV position on the circle plane
        pos[2] = self.center[2]
        pc_vec = self.center - pos
        pc_u = pc_vec / npla.norm(pc_vec)

        los_u = np.cross(self._rot, pc_u)
        rospy.loginfo("pc_u: {}, los_u: {}, rot: {}".format(pc_u,
                                                            los_u,
                                                            self._rot))
        return los_u, npla.norm(los_u)

    def phi(self, pos):
        """Get the tracking error measured as the result of applying f(pos)
        where f is the curve."""
        x = pos[0] - self.center[0]
        y = pos[1] - self.center[1]
        r = self.radius
        return x * x + y * y - r * r

    def grad_phi(self, pos):
        """Get the gradient of the curve."""
        x = pos[0] - self.center[0]
        y = pos[1] - self.center[1]
        return np.array([2 * x, 2 * y, 0])

    @property
    def trace(self):
        trace_list = []
        for alpha_deg in range(0, 361):
            alpha_rad = alpha_deg * np.pi / 180
            r_vec = np.array([np.cos(alpha_rad),
                              np.sin(alpha_rad), 0]) * self.radius
            trace_list.append(self.center + r_vec)

        return trace_list


class Ellipse(Curve):

    def __init__(self, center, hor_radius, ver_radius, orientation, **kwargs):
        """Initialize an Ellipse curve

        x^2   y^2
        --- + --- = 1
        a^2   b^2

        Arguments:
            center: center of the ellipse
            hor_radius: horizontal radius (a)
            ver_radius: vertical radius (b)
            orientation: 1 ccw, -1 cw ¿?
        """
        super(Ellipse, self).__init__(**kwargs)
        self._center = center
        self._a = hor_radius
        self._b = ver_radius
        self._orient = orientation
        if self._radius >= 0:
            self._rot = np.array([0., 0., 1.])  # rotate ccw
        else:
            self._rot = np.array([0., 0., -1.])  # rotate cw

        # TODO: self._rot defines the orientation plane which the
        # circumference belongs to.

    @property
    def orientation(self):
        """Direction to follow the curve for circular ones.

        Returns:
            1  if radius is positive
            -1 if radius is negative
        """
        if self.orientation >= 0:
            return -1
        else:
            return 1

    @property
    def center(self):
        return self._center

    @property
    def a(self):
        return self._a

    @property
    def b(self):
        return self._b

    def crosstrack_error(self, pos):
        # let's suppose the ellipse it's not too much excentric and use the
        # same algorithm as for the circle using the mean of a and b as radius
        pos[2] = self.center[2]
        pc_vec = self.center - pos
        pc_norm = npla.norm(pc_vec)
        r_vec = pc_vec / pc_norm * np.abs(self._a + self._b)
        d_vec = pc_vec - r_vec
        d = npla.norm(d_vec)
        return d_vec / d, d

    def los_vector(self, pos):
        raise NotImplemented()

    def phi(self, pos):
        """Get the tracking error measured as the result of applying f(pos)
        where f is the curve."""
        x = pos[0] - self.center[0]
        y = pos[1] - self.center[1]
        return x * x / self.a / self.a + y * y / self.b / self.b - 1

    def grad_phi(self, pos):
        """Get the gradient of the curve."""
        x = pos[0] - self.center[0]
        y = pos[1] - self.center[1]
        return np.array([2 * x / self.a / self.a, 2 * y / self.b / self.b, 0])

    @property
    def trace(self):
        trace_list = []
        for alpha_deg in range(0, 361):
            alpha_rad = alpha_deg * np.pi / 180
            r_vec = np.array([a * np.cos(alpha_rad),
                              b * np.sin(alpha_rad), 0])
            trace_list.append(self.center + r_vec)

        return trace_list


class GuidanceBase(object):

    __metaclass__ = ABCMeta

    _FALLBACK_CURVE = Circumference(np.array([0, 0, 1000]), -100,
                                    timeout=0.0, time_limit=0.0)

    def __init__(self, path, time=0, fallback_curve=_FALLBACK_CURVE, **kwargs):
        """Initialize guidance.

        Arguments:
            path: queue of curves.
            fallback_curve: Curve to follow when path is void. If None,
                continue following the last curve
        """
        self._path = path
        self.curve_start_time = time
        self.fallback_curve = fallback_curve
        if self.fallback_curve is not None:
            self.current_curve = self.fallback_curve
        else:
            self.current_curve = GuidanceBase._FALLBACK_CURVE

        self.next_curve(time)

    @property
    def fallback_curve(self):
        return self._fallback_curve

    @fallback_curve.setter
    def fallback_curve(self, curve):
        self._fallback_curve = curve

    @property
    def path(self):
        return self._path

    @path.setter
    def path(self, path):
        self._path = path

    def duration_exceded(self, time):
        if time - self.curve_start_time >= self.current_curve.timeout:
            return True
        else:
            return False

    def time_exceded(self, time):
        if time >= self.current_curve.time_limit:
            return True
        else:
            return False

    def next_curve(self, time):
        self.curve_start_time = time
        try:
            self.current_curve = self.path.popleft()
            rospy.loginfo("Using {}".format(str(self.current_curve)))
        except IndexError:
            if self.fallback_curve is not None:
                if self.current_curve is None:
                    self.current_curve = GuidanceBase._FALLBACK_CURVE
                else:
                    self.current_curve = self.fallback_curve
                rospy.logwarn("Using fallback curve in guidance system.")

    def close_to_curve(self, curve, pos, distance):
        if isinstance(curve, Circumference):
            if npla.norm(pos[:2] - curve.center[:2]) < \
               distance + np.abs(curve.radius):
                return True
            else:
                return False
        elif curve.crosstrack_error(pos)[1] < distance:
            return True
        else:
            return False

    def update_curve(self, pos, time):
        """Update current curve."""
        if self.current_curve is self.fallback_curve:
            self.next_curve(time)  # Try to get out of fallback state
        else:
            if len(self.path) > 0:
                n_p = self.path[0]
                c_p = self.current_curve
                condition = False
                approaching_dist = self.current_curve.approaching_distance
                discard_type = self.current_curve.discard_type

                if Curve.DISCARD_CLOSETO in discard_type and \
                        self.close_to_curve(n_p, pos, approaching_dist) and \
                        self.close_to_curve(c_p, pos, approaching_dist):
                    condition = True
                    rospy.loginfo("DISCARD_CLOSETO")
                elif Curve.DISCARD_TIMEOUT in discard_type and \
                        self.duration_exceded(time):
                    condition = True
                    rospy.loginfo("DISCARD_TIMEOUT")
                elif Curve.DISCARD_TIMELIMIT in discard_type and \
                        self.time_exceded(time):
                    condition = True
                    rospy.loginfo("DISCARD_TIMELIMIT")

                if condition:
                    self.next_curve(time)
            else:
                if self.time_exceded(time) or self.duration_exceded(time):
                    if self.fallback_curve is None:
                        self.current_curve = self.current_curve
                    else:
                        self.current_curve = self.fallback_curve

    @abstractmethod
    def compute_angle(self, pos, time):
        pass


class StepGuidance(GuidanceBase):

    def __init__(self, path, high, low, period=40.0,
                 duty_cycle=0.5, time=0, **kwargs):
        super(StepGuidance, self).__init__(path,
                                           time, GuidanceBase._FALLBACK_CURVE)
        self.heading = 0
        self.prev_time = 0
        self.high = high
        self.low = low
        self.period = period
        self.duty_cycle = duty_cycle
        self.down_trig = False

    def compute_angle(self, pos, time):
        if (time % self.period) / self.period < self.duty_cycle:
            return self.high
        else:
            return self.low


class RampGuidance(GuidanceBase):

    def __init__(self, path, slope=np.pi/30, time=0, **kwargs):
        super(RampGuidance, self).__init__(path,
                                           time, GuidanceBase._FALLBACK_CURVE)
        self.heading = 0
        self.prev_time = None
        self.slope = slope
        self.down_trig = False

    def compute_angle(self, pos, time):
        if self.prev_time is None:
            self.prev_time = time - 0.1  # Assume 0.1s period
        self.heading += self.slope * (time - self.prev_time)
        if self.heading >= 2 * np.pi and not self.down_trig:
            self.slope *= -1
            self.down_trig = True
        elif self.heading <= 0.0 and self.down_trig:
            self.slope *= -1
            self.down_trig = False
        self.prev_time = time
        return self.heading


class VFMathGuidance(GuidanceBase):
    """Vector Field guidance algorithm implemented as described by
    Yuri A. Kapitanyuk, Anton V. Proskurnikov and Ming Cao in "A guiding
    vector field algorithm for path following control of nonholonomic mobile
    robots"
    """

    def __init__(self, path, ks=1, kn=0.000065, time=0,
                 fallback_curve=GuidanceBase._FALLBACK_CURVE, **kwargs):
        """Initialize VFMathGuidance guidance.

        ks=1, kn=0.000065 gives critically damped response for malolo1

        Arguments:
            path: queue of curves.
            k_p: pursuit gain.
            k_l: LOS gain.
            time: initial time
            fallback_curve: Curve to follow when path is void. If None,
                continue following the last curve
        """
        super(VFMathGuidance, self).__init__(path, time, fallback_curve)
        self.kn = kn
        self.ks = ks
        self.E = np.array([[0, 1], [-1, 0]])

    def compute_angle(self, pos, time):
        """Compute angle using guidance algorithm.

        Arguments:
            pos: position of UAV
            time: current time

        Returns:
            angle to follow in radians
        """
        self.update_curve(pos, time)
        p = self.current_curve
        rospy.loginfo("[guidance] doing: {}".format(repr(p)))

        pos = pos[0:2]
        E = self.E * p.orientation  # CW or CCW
        # rospy.loginfo("[guidance] E: {}".format(E))
        e = self.ks * p.phi(pos)
        rospy.loginfo("[guidance] e: {}".format(e))
        ene = p.grad_phi(pos)[0:2]
        # rospy.loginfo("[guidance] ene: {}".format(ene))
        tau = np.dot(ene, E)
        # rospy.loginfo("[guidance] tau: {}".format(tau))
        uve = tau - self.kn * e * ene
        # rospy.loginfo("[guidance] uve: {}".format(uve))
        eme_d = uve / npla.norm(uve)
        # rospy.loginfo("[guidance] eme_d: {}".format(eme_d))

        return heading_of_vector(eme_d)


class PLOSGuidance(GuidanceBase):

    def __init__(self, path, k_p=100, k_l=2, time=0,
                 fallback_curve=GuidanceBase._FALLBACK_CURVE, **kwargs):
        """Initialize PLOS guidance.

        Arguments:
            path: queue of curves.
            k_p: pursuit gain.
            k_l: LOS gain.
            time: initial time
            fallback_curve: Curve to follow when path is void. If None,
                continue following the last curve
        """
        super(PLOSGuidance, self).__init__(path, time, fallback_curve)
        self.k_p = k_p
        rospy.logwarn(k_p)
        self.k_l = k_l

    def compute_angle(self, pos, time):
        """Compute angle using PLOS guidance algorithm.

        Arguments:
            pos: position of UAV
            time: current time

        Returns:
            angle to follow in radians
        """

        self.update_curve(pos, time)

        p = self.current_curve

        dest_u, __ = p.los_vector(pos)
        pursuit = self.k_p * dest_u

        ct_u, d = p.crosstrack_error(pos)
        los = self.k_l * ct_u * d

        rospy.loginfo("pursuit: {}  los: {}".format(pursuit, los))
        rospy.loginfo("doing: {}".format(repr(p)))

        return heading_of_vector(pursuit + los)


class VFGuidance(GuidanceBase):

    def __init__(self, path, v=15, pc=600, time=0,
                 fallback_curve=GuidanceBase._FALLBACK_CURVE, **kwargs):
        """Initialize PLOS guidance.

        Arguments:
            path: queue of curves.
            v: uav_speed.
            pc: VF parameter.
            time: initial time
            fallback_curve: Curve to follow when path is void. If None,
                continue following the last curve
        """
        super(VFGuidance, self).__init__(path, time, fallback_curve)
        self.pc = pc
        self.v = v

    def compute_angle(self, pos, time):
        """Compute angle using VF guidance algorithm.

        Arguments:
            pos: position of UAV
            time: current time

        Returns:
            angle to follow in radians
        """
        self.update_curve(pos, time)

        p = self.current_curve
        rospy.loginfo("doing: {}".format(repr(p)))
        if not isinstance(p, Circumference):
            raise ValueError("VF guidance only valid for circumferences")

        dest_u, __ = p.los_vector(pos)
        ct_u, r = p.crosstrack_error(pos)
        pc = self.pc if p.radius >= 0 else - self.pc
        rd = np.abs(p.radius)
        r = np.abs(r)
        common = self.v / np.sqrt((r-rd)*(r-rd)+pc*pc+r*r)

        r_dot = (common * ((r - rd))) * ct_u
        r_thetha_dot = (common * np.abs(pc) * r) * dest_u

        # rospy.logwarn("dest: {}  pos: {}".format(p.destination,
        #                                          pos))
        rospy.loginfo("[guidance] r_dot: {}  r_theta_dot: {}".format(
            r_dot, r_thetha_dot))

        return heading_of_vector(r_dot + r_thetha_dot)
