#!/usr/bin/env python

import rospy
from math import atan2, sqrt
from two_wheel_robot.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist 

class turtle():
    def __init__(self):
        rospy.init_node('velocity_controller', anonymous=True)
        self.pose = Pose()
        self.twist_msg = Twist()
        self.vel_publisher = rospy.Publisher('controller/cmd_vel', Twist, queue_size=1)
        self.test_publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
        self.pose_subscriber = rospy.Subscriber('turtle1/pose', Pose, self.pose_callback)
        self.rate = rospy.Rate(10)
   
    def pose_callback(self, pose_msg):
        self.pose.x = round(pose_msg.x,3)
        self.pose.y = round(pose_msg.y,3)
        self.pose.theta = round(pose_msg.theta,3)
        return

    def getdistance(self,goalpose):
        distance = sqrt((goalpose.x - self.pose.x)**2 + (goalpose.y - self.pose.y)**2)
        return distance
    
    def inputgoal(self):
        pi = 3.14159
        goalpose = Pose()
        goalpose.x = 10 #int(input('Enter X goal: '))
        goalpose.y = 10 #int(input('Enter Y goal: '))
        goalpose.theta = 1.5*pi #int(input('Enter theta goal: '))
        goaltolerance = 0.5 #int(input('Enter tolerance (0 < tol. < 0.1): '))
        return [goalpose, goaltolerance]


    def velocity_controller(self,goal):
        goalpose = goal[0]
        goaltolerance = goal[1]
        print('Inside velocity_controller()')
        Kp_linear = 0.1
        Kp_angular1 = .5
        goalvelocity = Twist()
        while (self.getdistance(goalpose) >= goaltolerance) and (not rospy.is_shutdown()):
            goalvelocity.linear.x = round( Kp_linear*sqrt((goalpose.x - self.pose.x)**2 + (goalpose.y - self.pose.y)**2) ,5)
            goalvelocity.linear.y = 0
            goalvelocity.linear.z = 0
            goalvelocity.angular.x = 0
            goalvelocity.angular.y = 0
            goalvelocity.angular.z = round( Kp_angular1*((atan2((goalpose.y-self.pose.y),(goalpose.x-self.pose.x))) - self.pose.theta) ,5)
            self.vel_publisher.publish(goalvelocity)
            self.test_publisher.publish(goalvelocity)
            self.rate.sleep()

        print('Linear controller terminated') 
        Kp_angular2 = .25
        while (abs(goalpose.theta - self.pose.theta) >= 0.05) and (not rospy.is_shutdown()):
            goalvelocity.linear.x = 0
            goalvelocity.linear.y = 0
            goalvelocity.angular.z = round( Kp_angular2*(goalpose.theta - self.pose.theta)  , 5)
            self.vel_publisher.publish(goalvelocity)
            self.test_publisher.publish(goalvelocity)
            self.rate.sleep()
        print('Angular controller terminated')

if __name__ == '__main__':
    try:
        Bot1 = turtle()
        goal = Bot1.inputgoal()
        Bot1.velocity_controller(goal)
    except rospy.ROSInterruptException:
        pass
