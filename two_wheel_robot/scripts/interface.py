#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from two_wheel_robot.msg import Speeds


# Note: Hereafter, pwm duty cycle will be referred to as 'dc'
class Interface():
    def __init__(self):
        self.pub = rospy.Publisher("interface/speeds", Speeds,queue_size = 1)
        self.sub = rospy.Subscriber('controller/cmd_vel', Twist, self.callback)
        self.VangularCurrent = 0
        return
    
    def rpm2pwm(self,rpm):
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
            return (-1)*pwm
        
    def convert2dc(self, Vleft, Vright, Omega):
        radius = (19.05/1000)  # (in mm) large wheel
        rpmleft = Vleft/(radius) # wheel velocity = rpm * radius
        rpmright = Vright/(radius)

        pwmleft = self.rpm2pwm(rpmleft)
        pwmright = self.rpm2pwm(rpmright)
        print('L V',round(Vleft,4),round(rpmleft,4),'pwm',pwmleft,'R V',round(Vright,4),round(rpmright,4),'pwm:',pwmright, 'Omega',Omega)
        self.pwm_publish(pwmleft,pwmright)
        return

    def pwm_publish(self, pwmleft, pwmright):
        pwm = Speeds()
        pwm.s1 = pwmleft
        pwm.s2 = pwmright
        self.pub.publish(pwm)
        return

    def callback(self, ctrlr_twist):
        base_length = (48) # mm
        Vlinear = ctrlr_twist.linear.x
        Vangular = ctrlr_twist.angular.z
        Omega = Vangular - self.VangularCurrent

        if Omega == 0:
            if Vlinear == 0:
                self.pwm_publish(0,0)            
            else:
                Vleft = Vlinear
                Vright = Vleft
                self.convert2dc(Vright,Vleft,Omega)                    
        else:
            if Vlinear == 0:
                Vleft = - (Omega * 0.5 * base_length)
                Vright = (Omega * 0.5 * base_length)
                self.convert2dc(Vleft, Vright, Omega)            
            else:
                Vleft = Vlinear
                Vright = Vlinear + (Omega * base_length)
                self.convert2dc(Vleft,Vright,Omega)

        self.VangularCurrent = Vangular
        return

def testfunc():
    speed = Speeds()
    a = 10
    speed.s1 = a
    speed.s2 = a

    while (True):
        pub.publish(speed)
    return

#def testmain():
    #rospy.init_node('interface', anonymous= True)
    #global pub
    #pub = rospy.Publisher("interface/speeds", Speeds,queue_size = 1)
    #testfunc()
    #while not rospy.is_shutdown():
    #    rospy.spin()
    #return

    


def main():
    rospy.init_node('interface', anonymous= True)     
    #interface1 = Interface()
    
    while not rospy.is_shutdown():
        rospy.spin()
    return

if __name__ == '__main__':
    try:
        main()
        #testmain()
    except rospy.ROSInterruptException:
        pass
