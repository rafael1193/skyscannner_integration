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

import numpy as np


def coord_to_meters(lat1, lon1, lat2, lon2):
    """Calculate distance between two earth coordinates using haversine
    formula. Adapted from: http://stackoverflow.com/a/11172685
    """
    R = 6378.137  # Radius of earth in KM
    dLat = (lat2 - lat1) * np.pi / 180
    dLon = (lon2 - lon1) * np.pi / 180
    a = np.sin(dLat/2) * np.sin(dLat/2) + \
        np.cos(lat1 * np.pi / 180) * np.cos(lat2 * np.pi / 180) * \
        np.sin(dLon/2) * np.sin(dLon/2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    d = R * c
    return d * 1000  # meters


def wrap_angle_rad(angle):
    while angle < 0.0 and angle >= 2 * np.pi:
        if angle < 0.0:
            angle += 2 * np.pi
        elif angle >= 2 * np.pi:
            angle -= 2 * np.pi
    return angle


def wrap_angle_deg(angle):
    while angle < 0.0 and angle >= 360.0:
        if angle < 0.0:
            angle += 360.0
        elif angle >= 360.0:
            angle -= 360.0
    return angle


def heading_of_vector(vector):
    x1 = vector[1]
    x2 = vector[0]
    return np.arctan2(x1, x2)
