#!/usr/bin/env python
from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from std_msgs.msg import String, Int32, Int8, Float32, Float64
from sensor_msgs.msg import Image, CompressedImage
from numpy import pi
import rospy
import time
import cv2

esc_pwm = 1500
steer_pwm = 1580
    
def arduino_interface():
#	global esc_pwm, steer_pwm

	rospy.init_node('arduino_interface')

	loop_rate   = 10
	dt          = 1.0 / loop_rate
	rate        = rospy.Rate(loop_rate)

	esc_pub = rospy.Publisher('esc_pwm', Float32, queue_size=1)
	steer_pub = rospy.Publisher('steer_pwm', Float32, queue_size=1)

	while not rospy.is_shutdown():

		esc_pwm = 1600.0
		steer_pwm = 2200
		steer_pub.publish(steer_pwm)
		esc_pub.publish(esc_pwm)
		
        rate.sleep()



#############################################################
if __name__ == '__main__':
	try:
		arduino_interface()
	except rospy.ROSInterruptException:
		pass
