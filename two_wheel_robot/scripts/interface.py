#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from two_wheel_robot.msg import Speeds


# Note: Hereafter, pwm duty cycle will be referred to as 'dc'

def rpm2pwm(rpm):
    original_rpm = rpm
    rpm = abs(rpm)
    if rpm <= 5:
        pwm = 0
    elif rpm > 5 and rpm < 133.6:
        pwm = 10
    elif rpm > 113.6 and rpm < 350:
        # Eqn of cubic fit for data points obtained for timeperiod vs. pwm
        z = (rpm - 290)/67 # intermediate variable
        pwm = int((6.4)*pow(z, 3) + 25*pow(z, 2) + 36*z + 40)
    else:
        pwm = 100
    if original_rpm >= 0:
        return pwm
    else:
        return (-1) * pwm

def convert2dc(Vleft, Vright, Vangular):
    radius = (19.05/1000)  # (in mm) large wheel
    rpmleft = Vleft/(radius) # wheel velocity = rpm * radius
    rpmright = Vright/(radius)

    pwmleft = rpm2pwm(rpmleft)
    pwmright = rpm2pwm(rpmright)
    print('L V',round(Vleft,4),round(rpmleft,4),'pwm',pwmleft,'R V',round(Vright,4),round(rpmright,4),'pwm:',pwmright, 'Vang',Vangular)
    pwm_publish(pwmleft,pwmright)
    return

def pwm_publish(pwmleft,pwmright):
    pwm = Speeds()
    pwm.s1 = pwmleft
    pwm.s2 = pwmright
    pub.publish(pwm)
    return

def callback(ctrlr_twist):
    base_length = (48) # mm
    Vlinear = ctrlr_twist.linear.x
    Vangular = ctrlr_twist.angular.z

    if Vangular == 0:
        if Vlinear == 0:
            pwm_publish(0,0)            
        else:
            Vleft = Vlinear
            Vright = Vleft
            convert2dc(Vright,Vleft,Vangular)                    
    else:
        if Vlinear == 0:
            Vleft = - (Vangular * 0.5 * base_length)
            Vright = (Vangular * 0.5 * base_length)
            convert2dc(Vleft, Vright, Vangular)            
        else:
            Vleft = Vlinear
            Vright = Vlinear + (Vangular * base_length)
            convert2dc(Vleft,Vright,Vangular)
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
