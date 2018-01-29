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

from threading import Lock

import numpy as np

from mesonh_atmosphere import MesoNHAtmosphere

import rospy
from skyscanner.msg import Wind
from skyscanner.srv import GetWind, GetWindResponse

M_IN_KM = 1000.
default_origin = np.array([0., 0., 0., 0.])
default_scale = np.array([1., 1./M_IN_KM, 1./M_IN_KM, 1./M_IN_KM])


class DummyEnv(object):
    """Fake Atmosphere class that always returns zero wind"""

    def __init__(*args, **kwargs):
        pass

    def get_wind(*args, **kwargs):
        return np.array([0., 0., 0.])


class AtmosphereService(object):
    """Class providing the `get_wind` service"""

    def __init__(self, atm, origin=None, scale=None):
        """Initialize Atmosphere service.

        Arguments:
            atm: wind provider object defining a get_wind method
                 (MesoNHAtmosphere or DummyEnv).
            origin: position offset (default: None).
            scale: scale of data (default: None).
            """

        self.atm = atm
        self.origin = [origin, default_origin][origin is None]
        self.scale = [scale, default_scale][scale is None]

        self.atm_lock = Lock()

        self.wind_srv = rospy.Service('get_wind', GetWind,
                                      self.handle_get_wind)

    def handle_get_wind(self, req):
        """Handle get wind request for position.

        Arguments:
            req: GetWindRequest.

        Returns:
            GetWindResponse containing real wind in position.

        Raises:
            rospy.ServiceException with the error
        """
        rospy.logdebug("[atmosphere] [ac_id={}] get_wind request: "
                       "time={} position={}".format(
                           req.ac_id, req.time.to_time(), req.position))
        try:
            w = self.get_wind(req.position.x, req.position.y, req.position.z,
                              req.time.to_time())
            wind = Wind(w[0], w[1], w[2])
            rospy.logdebug(
                "[atmosphere] [ac_id={}] get_wind response:\n{}".format(
                    req.ac_id, wind))
            return GetWindResponse(wind)
        except Exception as e:
            rospy.logerr(e)
            raise rospy.ServiceException(e)

    def get_wind(self, east, north, up, seconds):
        loc = np.array([seconds, up, east, north]) * self.scale + self.origin
        self.atm_lock.acquire()
        weast, wnorth, wup = self.atm.get_wind(loc, method='linear')
        self.atm_lock.release()
        return weast, wnorth, wup


def main():
    rospy.init_node('atmosphere')

    atm = None
    origin = default_origin

    if rospy.get_param("mesonh/use_dummy_env", False):
        rospy.logwarn("Using dummy atmosphere.")
        atm = DummyEnv()
    else:
        mesonh_file_time_step = rospy.get_param("mesonh/mesonh_file_time_step")
        mesonh_files_path = rospy.get_param("mesonh/mesonh_files_path")
        max_time_minutes = rospy.get_param("mesonh/max_time_minutes")
        mesonh_files = [mesonh_files_path +
                        "U0K10.1.min{:02d}.{:03d}_diaKCL.nc".format(minute,
                                                                    second)
                        for minute in range(1, max_time_minutes)
                        for second in range(1, 61)]

        # build atmosphere simulation source
        atm = MesoNHAtmosphere(mesonh_files, mesonh_file_time_step, tinit=0)

        # origin for translation from paparazzi to mesoNH frame
        origin = rospy.get_param('mesonh/origin',
                                 {'x': 0, 'y': 0, 'z': 0, 't': 0})
        origin = np.array([origin['t'], origin['z'], origin['x'], origin['y']])

    AtmosphereService(atm, origin)

    rospy.spin()


if __name__ == '__main__':
    main()
