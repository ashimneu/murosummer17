#!/usr/bin/env python

from math import pow, atan2, sqrt, cos, sin, asin, pi
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped
import time
import tf
from std_msgs.msg import String
from two_wheel_robot.msg import Pose


# Thanks to Ramon for his camera_listener class
#-----------------------------------------------------------------------------------------------------
# listener class (comes from camera node)
class camera_listener(object):

	def __init__(self):
		
        #world frame position
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		#quaternions
		self.quaternion_x = 0.0
		self.quaternion_y = 0.0
		self.quaternion_z = 0.0
		self.quaternion_w = 0.0
		#world frame orientation
		self.theta_x = 0.0
		self.theta_y = 0.0
		self.theta_z = 0.0
		#covariance matrix
		self.cov = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
				[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

	def callback(self,data):	

        #free space position 
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.z = data.pose.pose.position.z
        #angular orientation in quaternions
		self.quaternion_x = data.pose.pose.orientation.x
		self.quaternion_y = data.pose.pose.orientation.y
		self.quaternion_z = data.pose.pose.orientation.z
		self.quaternion_w = data.pose.pose.orientation.w
        # define an array with quaternions
		quaternion = (self.quaternion_x,
							self.quaternion_y,
							self.quaternion_z,
							self.quaternion_w)

        # define obtain euler angles in radians
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.theta_x = euler[0]
		self.theta_y = euler[1]
		self.theta_z = euler[2]

        #covarince matrix from camera
		self.cov = [data.pose.covariance[0:6],
				[data.pose.covariance[6:12]],
				[data.pose.covariance[12:18]],
				[data.pose.covariance[18:24]],
				[data.pose.covariance[24:30]],
				[data.pose.covariance[30:36]]]
        #kalman.getmeasurement_pose()
#-----------------------------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------------------------

class kf(object):
    def __init__(self):
        rospy.init_node('kf', anonymous=True)
        self.pose_publisher = rospy.Publisher('kf/pose', Pose, queue_size=1)
        self.controller_cmdVel_subscriber = rospy.Subscriber('controller/cmd_vel', Twist, self.cmd_vel_callback)
        #self.sensorPose_subscriber = rospy.Subscriber('turtle1/pose', Pose, self.sensorPose_callback)
        self.rate = rospy.Rate(10)
        self.cmd_vel = Twist()
                
        self.robotPose_G = np.zeros(3) #Pose in global reference frame
        self.cmd_Vel_G = np.zeros(3)

        self.Z_T = np.zeros(0)
        


    def getmeasurement_pose(self):
        #self.Z_T = [[measurement_pose.x],
		#	[measurement_pose.y],
		#	[measurement_pose.theta_z]]
        got_pose = True
        return 

    def cmd_vel_callback(self,cmd_vel_R):
        #Assigning cmd_vel to local vector variable V
        V = np.array([cmd_vel_R.linear.x, cmd_vel_R.linear.y, cmd_vel_R.angular.z])

        # Angle between robot reference & global reference frame
        theta = self.robotPose_G[2]

        # Transformation Matrix to convert cmd_vel from robot  
        # reference frame to global reference frame
        TM = np.array([[cos(theta),-sin(theta),0],[sin(theta),cos(theta),0],[0,0,1]])

        self.cmd_Vel_G = np.dot(TM,V)

        return

    def compute(self):
        print('initiating kalman filter')
        robotPose_G = Pose()
        dt = 0
        A = np.identity(3)
        B = np.array([[dt,0,0],[0,dt,0],[0,0,dt]])
        H = np.identity(3)
        

        X_T = np.array([0,0,0]) # true X at time t
        X_T_T = np.array([0,0,0]) # a posteriori at time t
        X_T_t = np.array([0,0,0]) # a priori at time t
        X_t = np.array([0,0,0]) # true X at time t-1
        X_t_t = np.array([0,0,0]) # a posteriori at time t-1

        P_T_T = np.array([[0,0,0],[0,0,0],[0,0,0]])
        P_T_t = np.array([[0,0,0],[0,0,0],[0,0,0]])
        P_t_t = np.array([[0,0,0],[0,0,0],[0,0,0]])

        k_T = np.array([[0,0,0],[0,0,0],[0,0,0]])

        Q = np.identity(3)
        Q[0,0] = 0.01
        Q[1,1] = 0.01
        Q[2,2] = 0.01
        
        R = np.identity(3) 
        I = np.array([[1,0,0],[0,1,0],[0,0,1]])
        
        got_pose = False
        
        t1 = time.time()
        t2 = time.time()
        
        while (not rospy.is_shutdown()):

            if (got_pose):
                R[0,0] = 0.01
                R[1,1] = 0.01
                R[2,2] = 0.01
            else:
                R[0,0] = 1000
                R[1,1] = 1000
                R[2,2] = 1000

             
            U = self.cmd_Vel_G
            Z_T = self.Z_T
            
            t2 = time.time()
            dt = t2 - t1

            #Predictor Step Part I
            X_T_t = np.add(np.dot(A,X_t_t) , np.dot(B,U))

            t1 = time.time()
            
            if (got_pose):
                P_T_t = np.subtract( np.dot(np.dot(A, P_t_t),A.transpose()) , Q)

                #Corrector Step Part II
                Y_T_t = Z_T - np.dot(H, X_T_t)

                numerator = np.dot(P_T_t, H.transpose())            
                denominator = np.add( np.dot(np.dot(H, P_T_t) , H.transpose()) , R)
                K_T = np.dot(numerator, np.linalg.inv(inversed_denominator))

                X_T_T = np.add( X_T_t , np.dot(K_T, Y_T_t) )
                P_T_T = np.dot(P_T_t, (I - np.dot(H,K_T)))
                Y_T_T = np.subtract(Z_T , np.dot(H, X_T_T) )

                X_t_t = X_T_T
                P_t_t = P_T_T
                self.robotPose_G = X_t_t

                got_pose = False
            else:              
                X_t_t = X_T_t
                self.robotPose_G = X_t_t

         
            robotPose_G.x = self.robotPose_G[0]
            robotPose_G.y = self.robotPose_G[1]
            robotPose_G.theta = self.robotPose_G[2]
            self.pose_publisher.publish(robotPose_G)
            self.rate.sleep()

def main():
    #global variables
    global got_pose
    global kalman
    kalman = kf()

    global measurement_pose
    measurement_pose = camera_listener()
    rospy.Subscriber('/ram/amcl_pose',PoseWithCovarianceStamped,measurement_pose.callback)
        
    #global t1, t2, dt
    
    
    kalman.compute()
    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
