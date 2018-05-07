#!/usr/bin/env python

# Author: Tony Zheng
# MEC231A BARC Project

import rospy
import time
from geometry_msgs.msg import Twist
from barc.msg import ECU, Input, Moving, Encoder
from numpy import pi

v_meas      = 0.0
t0          = time.time()
r_tire      = 0.05 # radius of the tire
servo_pwm   = 1511.0
motor_pwm   = 1500.0
motor_pwm_offset = 1500.0

# reference speed
v_ref = 0.0
thetaL1     = 0
thetaL2     = 0
thetaR1     = 0
thetaR2     = 0

def start_callback(data):
    global move, still_moving
    #print("2")
    #print(move)
    if data.linear.x >0:
        move = True
    elif data.linear.x <0:
        move = False
    #print("3")
    #print(move)
    pubname.publish(newECU)

def moving_callback_function(data):
    global still_moving, move
    if data.moving == True:
        still_moving = True
    else:
        move = False
        still_moving = False

def encoder_callback_function(data):
    global v_meas, thetaL1, thetaL2, thetaR1, thetaR2
    # determine the measured velocity from the encoder data
    bandw = 0.1 # sec, system frequency = 5 Hz
    vL=(3*data.FL - 4*thetaL1 + thetaL2)/(2*bandw)  # Provides us with the approximated angular velocity L
    vR=(3*data.FR - 4*thetaR1 + thetaR2)/(2*bandw)  # Provides us with the approximated angular velocity R
    v_meas=((vL+vR)/2)*(2*pi/8)*r_tire  # Calculate the linear velocity: (EncoderConts/Sec)*(Radians/Encoder)*Radius
    # Move forward one time step
    thetaL2=thetaL1
    thetaL1=data.FL
    thetaR2=thetaR1
    thetaR1=data.FR

# update
def callback_function(data):
    global move, still_moving, v_meas, newECU, pubname, v_ref
    #######################################################################
    # Convert the velocity into motorPWM and steering angle into servoPWM
    # a_servo, b_servo = -0.0006100255, 0.96345
    # a_servo *= 0.98
    # b_servo *= 0.94
    # newECU.servo = (data.delta - b_servo) / a_servo
    aa = -1672.7
    bb = 1511
    aa *= 0.80 # vary this parameter
    # bb *= 0.93
    newECU.servo = aa*data.delta + bb
    # rospy.logwarn('delta = {}'.format(data.delta))
    # rospy.logwarn('servo_pwm = {}\n'.format(newECU.servo))
    # newECU.servo = (newECU.servo - 1512) / 2 + 1512

    v_ref = data.vel
    ########################################################################

    servomax = 1800
    servomin = 1200
    if (newECU.servo<servomin):
        newECU.servo = servomin
    elif (newECU.servo>servomax):
        newECU.servo = servomax     # input steering angle

    pubname.publish(newECU)

# ==========================PID longitudinal controller========================#
class PID():
    def __init__(self, kp=1, ki=1, kd=1, integrator=0, derivator=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
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
        ## Anti windup
        if self.integrator >= self.integrator_max:
            self.integrator = self.integrator_max
        if self.integrator <= self.integrator_min:
            self.integrator = self.integrator_min
        self.I_effect = self.ki*self.integrator

        # Derivative control
        self.derivator = self.error - self.derivator
        self.D_effect = self.kd*self.derivator
        self.derivator = self.error

        acc = self.P_effect + self.I_effect + self.D_effect
        return acc



def inputToPWM():
    global pubname , newECU , subname, move , still_moving, v_meas, v_ref
    global motor_pwm, servo_pwm, motor_pwm_offset, servo_pwm_offset

    # initialize node
    rospy.init_node('inputToPWM', anonymous=True)

    newECU = ECU()
    newECU.motor = 1500
    newECU.servo = 1550
    move = False
    still_moving = False

    #print("1")
    #print(move)

    # topic subscriptions / publications
    pubname = rospy.Publisher('ecu_pwm',ECU, queue_size = 2)
    rospy.Subscriber('turtle1/cmd_vel', Twist, start_callback)
    subname = rospy.Subscriber('uOpt', Input, callback_function)
    rospy.Subscriber('moving', Moving, moving_callback_function)
    rospy.Subscriber('encoder', Encoder, encoder_callback_function)

    # set node rate
    loop_rate   = 20  # TODO(nish): raise sample rate
    ts          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    PID_control = PID(kp=70, ki=5, kd=0)
    maxspeed = 1650
    minspeed = 1400

    while not rospy.is_shutdown():
        # calculate acceleration from PID controller
        # rospy.logwarn('v_ref = {}, v_meas = {}'.format(v_ref, v_meas))
        # motor_pwm = PIDcontrol.acc_calculate(v_ref, v_meas) + motor_pwm_offset
        # newECU.motor = motor_pwm
        newECU.motor = 1576

        # safety check
        if (newECU.motor<minspeed):
            newECU.motor = minspeed
        elif (newECU.motor>maxspeed):
            newECU.motor = maxspeed
        if ((move == False) or (still_moving == False)):
            newECU.motor = 1500
            newECU.servo = 1550

        pubname.publish(newECU)
        # rospy.logwarn('{}'.format(newECU.motor))
        # rospy.logwarn('{}'.format(move))# wait
        rate.sleep()


if __name__ == '__main__':
    try:
       inputToPWM()
    except rospy.ROSInterruptException:
        pass
