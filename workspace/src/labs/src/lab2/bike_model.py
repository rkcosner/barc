#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

from numpy import sin, cos, tan, arctan, array, dot
from numpy import sign, argmin, sqrt, abs, pi
import numpy as np
import math
import rospy

def bikeFE(x, y, psi, v, a, d_f, a0,m, Ff, theta, ts):
    """
    process model
    """
    # external parameters
    l_f                    = 1.5
    l_r                    = 1.5
    g                      = 9.8

    # compute slip angle
    beta         = arctan(l_r / (l_f + l_r) * tan(d_f))

    F_ext = -Ff -a0*v**2 + m*g*sin(theta)

    deriv = np.array([v*cos(psi+beta),
                      v*sin(psi+beta),
                      v/l_r * sin(beta),
                      a + F_ext/m])

    return np.array([x, y, psi, v]) + ts * deriv