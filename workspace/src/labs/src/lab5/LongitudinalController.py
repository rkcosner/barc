#!/usr/bin/env python

import rospy
import time
from barc.msg import ECU, Encoder
from numpy import pi

# from encoder
v_meas      = 0.0
t0          = time.time()
r_tire      = 0.05 # radius of the tire
servo_pwm   = 1512.0
motor_pwm   = 1500.0
motor_pwm_offset = 1500.0

# reference speed 
v_ref = 0.5 # reference speed is 0.5 m/s

# encoder measurement update
def enc_callback(data):
    global t0, v_meas
    global n_FL, n_FR, n_BL, n_BR
    global ang_km1, ang_km2

    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR

    # compute the average encoder measurement
    n_mean = (n_FL + n_FR)/2

    # transfer the encoder measurement to angular displacement
    ang_mean = n_mean*2*pi/8

    # compute time elapsed
    tf = time.time()
    dt = tf - t0
    
    # compute speed with second-order, backwards-finite-difference estimate
    v_meas    = r_tire*(ang_mean - 4*ang_km1 + 3*ang_km2)/(dt)
    rospy.logwarn("velocity = {}".format(v_meas))
    # update old data
    ang_km1 = ang_mean
    ang_km2 = ang_km1
    t0      = time.time()

# reference speed
v_ref       = 0.5
thetaL1     = 0.0
thetaL2     = 0.0
thetaR1     = 0.0
thetaR2     = 0.0

# ===================================PID longitudinal controller================================#
class PID():
    def __init__(self, kp=1, ki=1, kd=1, integrator=0, derivator=0):
        self.kp = kp
        self.ki = ki
        self.integrator = integrator
        self.derivator = derivator
        self.integrator_max = 30
        self.integrator_min = -30

    def acc_calculate(self, speed_reference, speed_current):
        self.error = speed_reference - speed_current

        # Propotional control
        self.P_effect = self.kp*self.error

        # Integral control
        self.integrator = self.integrator + self.error
        
		# Anti windup
        if self.integrator >= self.integrator_max:
           self.integrator = self.integrator_max

        self.I_effect = self.ki*self.integrator

        acc = self.P_effect + self.I_effect

        # # Derivative control
        # self.derivator = self.error - self.derivator
        # self.D_effect = self.kd*self.derivator
        # self.derivator = self.error

        # acc = self.P_effect + self.I_effect + self.D_effect
        # # if acc <= 0:
        # #     acc = 20
        return acc

# =====================================end of the controller====================================#

def callback(data):
    global v_meas, thetaL1, thetaL2, thetaR1, thetaR2
    bandw = 0.200 # sec, system frequency = 5 Hz
    vL=(3*data.FL - 4*thetaL1 + thetaL2)/(2*bandw)  # Provides us with the approximated angular velocity L
    vR=(3*data.FR - 4*thetaR1 + thetaR2)/(2*bandw)  # Provides us with the approximated angular velocity R
    v_meas=((vL+vR)/2)*(2*pi/8)*r_tire  # Calculate the linear velocity: (EncoderCounts/Sec)*(Radians/Encoder)*Radius
    # Move forward one time step
    thetaL2=thetaL1
    thetaL1=data.FL
    thetaR2=thetaR1
    thetaR1=data.FR

# state estimation node
def controller():
    global motor_pwm, servo_pwm, motor_pwm_offset
    global v_ref, u

    # Initialize node:
    rospy.init_node('simulationGain', anonymous=True)


    # topic subscriptions / publications
    rospy.Subscriber('encoder', Encoder, enc_callback)
    ecu_pub   = rospy.Publisher('ecu_pwm', ECU, queue_size = 10)
    encoder   = rospy.Subscriber('encoder', Encoder, callback )

    # Set node rate
    loop_rate   = 5
    rate        = rospy.Rate(loop_rate)

    # Initialize the PID controller
    PID_control = PID(kp = 75, ki =5.5, kd = 0) # Calculated #s:P748, I503

    while not rospy.is_shutdown():
        # calculate acceleration from PID controller.
        motor_pwm =  PID_control.acc_calculate(v_ref, v_meas) + motor_pwm_offset
        # publish control command
        ecu_pub.publish( ECU(motor_pwm, servo_pwm) )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       controller()
    except rospy.ROSInterruptException:
        pass
