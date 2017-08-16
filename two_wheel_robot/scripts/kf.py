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
        
        self.cmd_vel = Twist()
   
    def sensorPose_callback(self, measurement_pose):
        Z[0] = round(camera_pose.x,4)
        Z[1] = round(camera_pose.y,4)
        Z[2] = round(camera_pose.theta,4)
        got_pose = True
        return 

    def controller_cmdVel_callback(self,cmd_vel):
        U[0] = cmd_vel.linear.x 
        U[1] = cmd_vel.linear.y
        U[2] = cmd_vel.angular.z
        return

    def compute(self):
        print('initiating kalman filter')
        pose = Pose()
        A = np.identity(3)
        B = np.identity(3)
        H = np.identity(3)

        X_T = np.array([0,0,0]) # true X at time t
        X_T_T = np.array([0,0,0]) # a posteriori at time t
        X_T_t = np.array([0,0,0]) # a priori at time t
        X_t = np.array([0,0,0]) # true X at time t-1
        X_t_t = np.array([0,0,0]) # a posteriori at time t-1

        P_T_T = np.array([0,0,0],[0,0,0],[0,0,0])
        P_T_t = np.array([0,0,0],[0,0,0],[0,0,0])
        P_t_t = np.array([0,0,0],[0,0,0],[0,0,0])

        k_T = np.array([0,0,0],[0,0,0],[0,0,0])

        Q = np.identity(3)
        R = np.identity(3) 
        I = np.array([1,0,0],[0,1,0],[0,0,1])
        
        got_pose = False
        
        while (not rospy.is_shutdown()):

            if (got_pose):
                R[0,0] = 1
                R[1,1] = 1
                R[2,2] = 1
            else:
                R[0,0] = 1000
                R[1,1] = 1000
                R[2,2] = 1000

            # in static view 
            U[0] = cmd_vel.linear.x
            U[1] = cmd_vel.linear.y
            U[2] = cmd_vel.angular.z           


            #Predictor Step Part I
            X_T_t = np.dot(A,X_t_t) + np.dot(B,U)
            
            if (got_pose):
                P_T_t = np.dot(np.dot(A, P_t_t),A.transpose()) + Q

                #Corrector Step Part II
                Y_T_t = Z_T - np.dot(H, X_T_t)

                numerator = np.dot(P_T_t, H.transpose())            
                denominator = np.dot(np.dot(H, P_T_t) , H.transpose()) + R
                K_T = np.dot(numerator, np.linalg.inv(inversed_denominator))

                X_T_T = X_T_t + np.dot(K_T, Y_T_t)
                P_T_T = np.dot(P_T_t, (I - np.dot(H,K_T)))
                Y_T_T = Z_T - np.dot(H, X_T_T)

                X_t_t = X_T_T
                P_t_t = P_T_T

                got_pose = False
            else:              
                X_t_t = X_T_t




            
            self.pose_publisher.publish(pose)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        #global variables
        global got_pose
        global Z
        global U

        Z = np.array([0,0,0])
        U = np.array([0,0,0])  

        filter = kf()
        filter.compute()
    except rospy.ROSInterruptException:
        pass
