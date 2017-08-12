#!/usr/bin/env python

import rospy
from two_wheel_robot.msg import Speeds
import RPi.GPIO as GPIO

def callback(cmd):
    if cmd.s1 > 0:
        m1_0.ChangeDutyCycle(cmd.s1)
        m1_1.ChangeDutyCycle(0)
        print "pin 5 set to ", cmd.s1
        print "pin 6 set to ", 0 
    else:
        m1_1.ChangeDutyCycle(-cmd.s1)
        m1_0.ChangeDutyCycle(0)
        print "pin 5 set to ", 0 
        print "pin 6 set to ", -cmd.s1 

    if cmd.s2 > 0:
        m2_0.ChangeDutyCycle(cmd.s2)
        m2_1.ChangeDutyCycle(0)
        print "pin 13 set to ", cmd.s2
        print "pin 19 set to ", 0 
    else:
        m2_1.ChangeDutyCycle(-cmd.s2)
        m2_0.ChangeDutyCycle(0)
        print "pin 13 set to ", 0 
        print "pin 19 set to ", -cmd.s2 



def init_pins():
    global m1_0
    global m1_1
    global m2_0
    global m2_1
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(5,GPIO.OUT)
    GPIO.setup(6,GPIO.OUT)
    GPIO.setup(13,GPIO.OUT)
    GPIO.setup(19,GPIO.OUT)
    m1_0 = GPIO.PWM(5,100)
    m1_1 = GPIO.PWM(6,100)
    m2_0 = GPIO.PWM(13,100)
    m2_1 = GPIO.PWM(19,100)
    m1_0.start(0)
    m1_1.start(0)
    m2_0.start(0)
    m2_1.start(0)

def cleanup_pins():
	m1_0.stop()
	m1_1.stop()
	m2_0.stop()
	m2_1.stop()

	GPIO.cleanup()

def pwm():

    rospy.init_node('pwm')
    rospy.Subscriber('pwm/speeds', Speeds, callback)

    init_pins()

    while not rospy.is_shutdown():
        rospy.spin()

    cleanup_pins()


if __name__ == '__main__':
    pwm()
