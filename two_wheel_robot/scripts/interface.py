#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from two_wheel_robot.msg import Speeds


# Note: Hereafter, pwm duty cycle will be referred to as 'dc'

def rpm2pwm(rpm):
    if rpm < 133.6:
        pwm = 10
    elif rpm > 347.5:
        pwm = 100
    else:
        pwm = int((2.1/100000)*pow(rpm, 3) - 0.013*pow(rpm, 2) + 2.6*rpm - (1.6*100)) #cubic fit from data
 
    return pwm

def convert2dc(Vright,Vleft):
    radius = (19.05 / 1000)  # (in meters) large wheel
    rpmleft = Vleft/(radius) # wheel velocity = rpm * radius
    rpmright = Vright/(radius)

    pwmleft = rpm2pwm(rpmleft)
    pwmright = rpm2pwm(rpmright)
    print('PWM: Left:',pwmleft,'Right:',pwmright)

    pwm = Speeds()
    pwm.s1 = pwmleft
    pwm.s2 = pwmright
    pub.publish(pwm)
    return

def callback(ctrlr_twist):
    base_length = 48 # mm
    maxdc1 = 100 # maximum dc 1
    maxdc2 = 60 # maximum dc 2
    leftdc = 0  # initialization for left wheel
    rightdc = 0 # inintialization for right wheel
    
    Vlinear = ctrlr_twist.linear.x
    Vangular = ctrlr_twist.angular.z

    if Vangular == 0:
        Vleft = Vlinear
        Vright = Vlinear
        convert2dc(Vright,Vleft)
        #print('Vangular: 0; Vl:',Vleft,'Vr:',Vright)
    else:                
        Vleft = Vlinear
        Vright = Vlinear + (Vangular * base_length)
        #print('Vangular != 0; Vl:',Vleft,'Vr:',Vright)
        convert2dc(Vright,Vleft)
    return

def main():
    global robotmaxVelocity 
    robotmaxVelocity = 0 # (m/s)

    rospy.init_node('interface', anonymous= True)
    global pub
    global sub
    pub = rospy.Publisher("pwm/speeds", Speeds,queue_size = 1)
    sub = rospy.Subscriber('interface/cmd_vel', Twist, callback)
    
    while not rospy.is_shutdown():
        rospy.spin()
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
