#!/usr/bin/env python

# Author: Tony Zheng
# MEC231A BARC Project

import rospy
import time
from geometry_msgs.msg import Twist
from barc.msg import ECU, Input, Moving, encoder



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
    global v_meas
    # determine the measured velocity from the encoder data
    bandw = 0.200 # sec, system frequency = 200 Hz
    vL=(3*data.FL - 4*thetaL1 + thetaL2)/(2*bandw)  # Provides us with the approximated angular velocity L
    vR=(3*data.FR - 4*thetaR1 + thetaR2)/(2*bandw)  # Provides us with the approximated angular velocity R
    v_meas=((vL+vR)/2)*(2*pi/8)*r_tire  # Calculate the linear velocity: (EncoderConts/Sec)*(Radians/Encoder)*Radius

# update
def callback_function(data):
    global move, still_moving, v_meas
    ################################################################################################################################################
    # Convert the velocity into motorPWM and steering angle into servoPWM
    a_motor, b_motor = -0.6055, 0.0129
    a_servo, b_servo = -0.0006100255, 0.96345
    # newECU.motor =  (data.vOpt - a_motor*current_vel) / b_motor
<<<<<<< HEAD
    newECU.motor =  (data.vOpt - a_motor * v_meas) / b_motor
    newECU.servo = (data.deltaOpt - b_servo) / a_servo
=======
    newECU.motor =  (data.vel - a_motor * 0.5) / b_motor
    newECU.servo = (data.delta - b_servo) / a_servo
>>>>>>> 70dd11391946dd63ba804b9742dda8319e83a981
    #################################################################################################################################################

    maxspeed = 1575
    minspeed = 1400
    servomax = 1800
    servomin = 1200
    if (newECU.motor<minspeed):
        newECU.motor = minspeed
    elif (newECU.motor>maxspeed):
        newECU.motor = maxspeed
    if (newECU.servo<servomin):
        newECU.servo = servomin
    elif (newECU.servo>servomax):
        newECU.servo = servomax     # input steering angle

    if ((move == False) or (still_moving == False)):
        newECU.motor = 1500
        newECU.servo = 1550
    #print("5")
    #print(move)

    pubname.publish(newECU)
# state estimation node
def inputToPWM():

    # initialize node
    rospy.init_node('inputToPWM', anonymous=True)

    global pubname , newECU , subname, move , still_moving, v_meas
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
    loop_rate   = 40
    ts          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    rospy.spin()



if __name__ == '__main__':
    try:
       inputToPWM()
    except rospy.ROSInterruptException:
        pass
