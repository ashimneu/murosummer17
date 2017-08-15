#!/usr/bin/env python

from math import atan2, sqrt

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from two_wheel_robot.msg import Pose


class kf():
    def __init__(self):
        rospy.init_node('kf', anonymous=True)
        self.pose_publisher = rospy.Publisher('kf/pose', Pose, queue_size=1)
        self.controller_cmdVel_subscriber = rospy.Subscriber('controller/cmd_vel', Twist, self.controller_cmdVel_callback)
        self.sensorPose_subscriber = rospy.Subscriber('turtle1/pose', Pose, self.sensorPose_callback)
        self.rate = rospy.Rate(5)
        self.pose = Pose()
        self.twist_msg = Twist()
   
    def sensorPose_callback(self, pose):
        self.pose.x = round(pose.x,4)
        self.pose.y = round(pose.y,4)
        self.pose.theta = round(pose.theta,4)
        return 

    def controller_cmdVel_callback(self,twist):
        return

    def compute(self):
        print('initiating kalman filter')
        pose = Pose()
        A = np.array([0,0,0],[0,0,0],[0,0,0])
        B = np.array([0,0],[0,0])
        H = np.array([0,0,0],[0,0,0],[0,0,0])

        X_T = np.array([0,0,0]) # true X at time t
        X_T_T = np.array([0,0,0]) # a posteriori at time t
        X_T_t = np.array([0,0,0]) # a priori at time t
        X_t = np.array([0,0,0]) # true X at time t-1
        X_t_t = np.array([0,0,0]) # a posteriori at time t-1

        P_T = np.array([0,0,0])
        P_T_t = np.array([0,0,0])
        P_t = np.array([0,0,0])

        k_T = 0

        H = np.array([0,0,0],[0,0,0],[0,0,0])
        
        Z_T = np.array([0,0,0])

        Q = np.array([0,0,0],[0,0,0],[0,0,0])

        R = np.array([0,0,0],[0,0,0],[0,0,0])

        Z_T = np.array([0,0,0])

        I = np.array([1,0,0],[0,1,0],[0,0,1]) 


        while (not rospy.is_shutdown()):

            #Predictor Step
            X_T_t = np.dot(A,X_t_t) + np.dot(B,U_T)
            Asquared = np.dot(A,A)
            P_T_t = np.dot(Asquared, P_t) + Q

            #Corrector Step
            numerator = np.dot(H, P_T_t)
            Hsquared = np.dot(H,H)
            denominator = np.dot(Hsquared, P_T_t) + R
            inversed_denominator = np.linalg.inv(inversed_denominator)
            k_T = np.dot(numerator, inversed_denominator)

            innovation = Z_T - np.dot(H, X_T_t)
            X_T_T = X_T_t + np.dot(K_T, innovation)

            P_T = np.dot(P_T_t, (I - np.dot(H,K_T)))

            
            self.pose_publisher.publish(pose)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        filter = kf()
        filter.compute()
    except rospy.ROSInterruptException:
        pass
