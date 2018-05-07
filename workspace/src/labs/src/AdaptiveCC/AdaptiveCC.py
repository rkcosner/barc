#!/usr/bin/env python

# Author: Tony Zheng
# ME 131 Lab 7

import rospy
import time
import roslib
import sys
import cv2
import controlpy
import scipy.linalg
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Image
from math import sqrt, atan, pi, pow, cos, sin, asin, tan, atan2
from barc.msg import barc_state,ECU, Input, Moving
from controlpy import analysis

# state estimation node
class image_processing_node():
    def __init__(self):

        self.vid = cv2.VideoCapture("/dev/video6")
        self.vid.set(12,5) #contrast
        self.vid.set(13,255) #saturation

        # Calibration Matrices
        self.mtx = np.array([[592.156892, 0.000000, 326.689246], [0.000000, 584.923917, 282.822026], [0.000000, 0.000000, 1.000000]])
        self.dist = np.array([-0.585868, 0.248490, -0.023236, -0.002907, 0.000000])
        self.rel,self.dst = self.vid.read()
        # Camera resolution
        self.w = 640
        self.h = 480

        # Reference velocity
        self.v_ref = 1

        # Number of moving average points
        self.sx = 3 # TODO(nish): try changing to 10
        self.movmean = np.zeros([2,self.sx])

        # Set node rate
        self.loop_rate   = 50
        self.ts          = 1.0 / self.loop_rate
        self.rate        = rospy.Rate(self.loop_rate)
        self.t0          = time.time()

        self.dt = self.ts
        self.count = 0
        self.incount = 0
        self.total = 0
        self.avg = 0
        self.total2 = 0
        self.avg2 = 0
        self.publish_image = False;
        self.timeprev = time.time()-self.dt

        time.sleep(.5)

        # Compute the udistortion and rectification transformation map
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(self.w,self.h),0,(self.w,self.h))
        self.mapx,self.mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None,self.newcameramtx,(self.w,self.h),5)


        self.statepoints = ''
        self.printme = True

        self.state_constraints = barc_state()
        self.reference_trajectory = barc_state()

        # Initialize publishers and subscribers
        self.moving_pub = rospy.Publisher("moving", Moving, queue_size=1)
        self.moving_pub.publish(True)
        #self.reference_trajectory_pub = rospy.Publisher("reference_trajectory", barc_state, queue_size = 1)
        #self.reference_image_pub = rospy.Publisher("image_reference_trajectory", Image, queue_size = 10)
        self.uOpt_pub = rospy.Publisher("uOpt", Input, queue_size=1)


        while not rospy.is_shutdown():
            try:
                self.count = self.count +1
                self.rel,self.dst = self.vid.read() # gets the current frame from the camera
                self.dt = time.time() - self.timeprev
                self.timeprev = time.time()

                self.cv_image = cv2.remap(self.dst,self.mapx,self.mapy,cv2.INTER_LINEAR) #Undistorts the fisheye image to rectangular
                self.x,self.y,self.w,self.h = self.roi
                self.dst = self.dst[self.y:self.y+self.h, self.x:self.x+self.w]

                # yellow = True makes the edge detection search for a yellow track using HSV. False will use grayscale and search for any edge regardless of color
                yellow = True
                kernel_size = 7
                if yellow:
                    cropped = self.cv_image
                    cropped[0:280,0:640] = 0
                    cropped = cv2.GaussianBlur(cropped,(kernel_size,kernel_size),0) #0.017s
                    # cropped = cv2.medianBlur(cropped,kernel_size)

                    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV) #.004 #Changes Color profile

                    #hsv = cv2.GaussianBlur(hsv,(kernel_size,kernel_size),0)
                    cv2.imshow('hsv',hsv[270:480,:])

                    # define range of yellow color in HSV (B,G,R)
                    lower_yellow = np.array([0,180,100])
                    upper_yellow = np.array([50,255,255])

                    # Threshold the HSV image to get only yellow colors
                    edges = cv2.inRange(hsv, lower_yellow, upper_yellow) #0.03s
                    
                    # define range of edge color in HSV (B,G,R)
                    #lower_red = np.array([0,0,180])
                    #upper_red = np.array([130,80,255])
                    #                                                             
                    #lower_white = np.array([0,200,200])
                    #upper_white = np.array([80,255,255])

                    # # Threshold the HSV image to get only edge colors
                    #reds = cv2.inRange(hsv, lower_red, upper_red)
                    #whites = cv2.inRange(hsv, lower_white, upper_white) #0.03s
                    #edges = cv2.bitwise_or(reds,whites)

                    edges = cv2.GaussianBlur(edges,(kernel_size+6,kernel_size+6),0)
                    #edges = cv2.Canny(edges,10,200)
                    cv2.imshow("Edges2",edges[270:480,:])
                    edgescropped = edges
                else:
                    # Convert Color Image to Grayscale
                    gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                    gray_image[0:270,0:640] = 0
                    gray_image = cv2.GaussianBlur(gray_image, (kernel_size, kernel_size), 0)
                    #cv2.imshow("blurred Image", gray_image)
                    #in MPC lab, 13 15 100
                    #edges = cv2.Canny(gray_image,40,80)
                    #cv2.imshow("Advanced Lane Detection ed", edges[270:480,:])
                   # whitecount = cv2.countNonZero(edges)

                    edgescropped = edges
                closest=np.argmax(np.flipud(np.ravel(np.transpose(edges))))%480
                # minimum yellow value, searches in vertical bands starting from the bottom right
                # rospy.logwarn('closest={}'.format(closest)) # Print the closest value
                distance = self.convert_Distance(closest)
                if closest == 0:
                    distance = 0
                # rospy.logwarn('dist={}\n'.format(distance))
                self.uOpt_pub.publish(distance, 0)
                if (self.count >100 and self.count%50==0):
                    self.incount +=1
                    self.timenext = time.time()
                    self.timeElapsed = self.timenext - self.timeprev
                    self.total2 = self.total2+self.timeElapsed
                    self.avg2 = self.total2/(self.incount)
                    print('Average Time: ',self.avg2)

                # Waitkey is necesarry to update image
                cv2.waitKey(3)

                self.rate.sleep()
            except IOError, (ErrorNumber, ErrorMessage):
                print('HERE')
                print('HERE')
                print(ErrorMessage)
                pass
    #################################################################################


    def convert_Distance(self, point):
        distance=self.calc_y_newPixel_to_x_Inertial(point)
        return distance

    def calc_y_newPixel_to_x_Inertial(self,y_newPixel):
        # Transforms the ynewpixel into xinertial frame
        #x_Inertial = 8.917E-9*y_newPixel**4 + -7.8875E-7*y_newPixel**3 + -0.0001258*y_newPixel**2 + 0.028327*y_newPixel
        #x_Inertial_test = 3.767535E-9*y_newPixel**4 + -1.8816308E-6*y_newPixel**3 + 0.0003606144*y_newPixel**2 + -0.024835717*y_newPixel**1 + 0.92770404385
        theirs = 6.411145E-9*y_newPixel**4 -1.98328E-6*y_newPixel**3 + 0.0002174*y_newPixel**2 - 0.00187*y_newPixel + 1.2013
        # x_Inertial = theirs
        # rospy.logwarn('{}, {}, {}'.format(x_Inertial, x_Inertial_test, theirs))
        x_Inertial = theirs # x_Inertial*0.3048 #convert ft to m
        return x_Inertial
    #########################################################################

def shutdown_func():
    cv2.destroyAllWindows()

def main(args):
    rospy.on_shutdown(shutdown_func)
    global image_processor_global
    global offset_global

    # Intialize the node
    rospy.init_node('image_processing_node', anonymous=True)

    image_processor_global = image_processing_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
