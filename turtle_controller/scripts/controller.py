#!/usr/bin/env python

import rospy
from math import atan2, sqrt
from turtle_controller.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist 

class turtle():
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        self.pose = Pose()
        self.twist_msg = Twist()
        self.vel_publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('turtle1/pose', Pose, self.pose_callback)
        self.rate = rospy.Rate(2)
   
    def pose_callback(self, pose_msg):
        self.pose.x = round(pose_msg.x,3)
        self.pose.y = round(pose_msg.y,3)
        self.pose.theta = round(pose_msg.theta,3)
        return

    def getdistance(self,goalpose):
        distance = sqrt((goalpose.x - self.pose.x)**2 + (goalpose.y - self.pose.y)**2)
        return distance
    
    def inputgoal(self):
        goalpose = Pose()
        goalpose.x = 10 #int(input('Enter X goal: '))
        goalpose.y = 10 #int(input('Enter Y goal: '))
        goalpose.theta = 1.5*3.14159 #int(input('Enter theta goal: '))
        #goaltolerance = int(input('Enter tolerance (0 < tol. < 0.1): '))
        return goalpose


    def velocity_controller(self,goalpose,goaltolerance):
        print('Inside velocity_controller()')
        Kp_linear = 0.10
        Kp_angular1 = 0.50
        goalvelocity = Twist()
        while (self.getdistance(goalpose) >= goaltolerance) and (not rospy.is_shutdown()):
        #while (self.getdistance(goalpose) >= goaltolerance) and (not rospy.is_shutdown):
            goalvelocity.linear.x = Kp_linear*sqrt((goalpose.x - self.pose.x)**2 + (goalpose.y - self.pose.y)**2)
            goalvelocity.linear.y = 0
            goalvelocity.linear.z = 0
            goalvelocity.angular.x = 0
            goalvelocity.angular.y = 0
            goalvelocity.angular.z = Kp_angular1*((atan2((goalpose.y-self.pose.y),(goalpose.x-self.pose.x))) - self.pose.theta)
            #goalvelocity.angular.z = Kp_angular1*(goalpose.theta - self.pose.theta)
            X = round(goalvelocity.linear.x, 4)
            Y = round(goalvelocity.linear.y, 4)
            Angle = round(goalvelocity.angular.z, 4)
            print('V_x: ', X,'V_y: ', Y,'V_theta: ', Angle) 
            

            self.vel_publisher.publish(goalvelocity)

            self.rate.sleep()

        Kp_angular2 = 0.25

        while (abs(goalpose.theta - self.pose.theta) >= 0.05) and (not rospy.is_shutdown()):
            goalvelocity.linear.x = 0
            goalvelocity.linear.y = 0
            goalvelocity.angular.z = Kp_angular2*(goalpose.theta - self.pose.theta)
            X = round(goalvelocity.linear.x, 4)
            Y = round(goalvelocity.linear.y, 4)
            Angle = round(goalvelocity.angular.z, 4)
            print('V_x: ', X,'V_y: ', Y,'V_theta: ', Angle)
            

            self.vel_publisher.publish(goalvelocity)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Bot1 = turtle()
        goal_pose = Bot1.inputgoal()
        Bot1.velocity_controller(goal_pose, 1)
    except rospy.ROSInterruptException:
        pass
