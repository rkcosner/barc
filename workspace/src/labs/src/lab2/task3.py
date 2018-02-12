#!/usr/bin/env python

import roslib
# roslib.load_manifest('teleop_twist_keyboard')
import rospy
import time
import sys, select, termios, tty, time
from barc.msg import ECU
from labs.msg import Z_DynBkMdl 
	
# from teleop module
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key
	
def loopfn():
	while not rospy.is_shutdown():
	 	
	 	key = getKey()

		if (key == '\x03'):
			break

		# calculate ECU commands
		if key == 'w':
			acc = 1
		elif key == 'a':
			d_f = -1
		elif key == 's':
			acc = -1
		elif key == 'd':
			d = 1


		# publish information
	 	state_pub.publish( ECU(acc, d_f) )
	
		# wait
		rate.sleep()
	
if __name__ == '__main__':
    	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('task3', anonymous=True)

	state_pub = rospy.Publisher('ecu', ECU, queue_size = 10)

		# set node rate
	loop_rate = 50
	rate = rospy.Rate(loop_rate)

	try:
		loopfn()
	except rospy.ROSInterruptException:
		pass

	finally:
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


