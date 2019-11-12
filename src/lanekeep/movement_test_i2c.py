#!/usr/bin/env python
import rospy
import time
import cv2
import RPi.GPIO as GPIO
from std_msgs.msg import String, Int32, Int8, Float32, Float64
from sensor_msgs.msg import Image, CompressedImage
from board import SCL, SDA
from numpy import pi
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

i2c_bus = GPIO.I2C(SCL, SDA)

pca.frequency = 50
servo = adafruit_motor.servo.Servo(servo_channel)
kit.servo[0].set_pulse_width_range(1000, 2000)

tick = 200
b = 6000

def enc_cb(enc_data):

	encoder = enc_data

def enc_cb(vel_data):

	velocity = vel_data

def to_arduino():

	rospy.init_node('to_arduino', anonymous=True)

	esc_pub = rospy.Publisher('esc_pwm', Float32, queue_size=1)
	steer_pub = rospy.Publisher('steer_pwm', Float32, queue_size=1)

	while not rospy.is_shutdown():
		try:
			
			steer_pwm = 1200
			steer_pub.publish(steer_pwm)
			kit.servo[0].angle = 120

			esc_pwm = 1560
			esc_pub.publish(esc_pwm)
		except:
			pass

def from_arduino():

	rospy.init_node('from_arduino', anonymous=True)

	rospy.Subscriber("encoder", Float32 , enc_cb)
	rospy.Subscriber("velocity", Float32 , vel_cb)


if __name__ == '__main__':
	try:
		to_arduino()
		from_arduino()
	except rospy.ROSInterruptException:
		pass
