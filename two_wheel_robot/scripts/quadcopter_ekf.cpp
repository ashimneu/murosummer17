// Name this file quadcopter_ekf.cpp
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
//	Credit to
//	Eigen for their opensource linear algebra library and headers
//	Allen for previous iteration of Kalman filter
//	ROS opensource
//	Aaron for his original quadcopter_ekf.cpp onto which I made modifications
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "eigen/Eigen/Dense"
#include <cmath>
#include <fstream>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include "ardrone_autonomy/Navdata.h"
#include <tf/tf.h>
#include <time.h>

////////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////////

using namespace Eigen;

// Variable Declarations
Vector4f X_k_k = Vector4f::Zero();  // State vector X_k|k
Vector4f X_k_i = Vector4f::Zero(); // State vector X_k|k-1
Vector4f X_i_i = Vector4f::Zero();  // State vector X_k-1|k-1
Vector4f X_i_i_returned = Vector4f::Zero();  // State vector X_k|k

Vector4f U = Vector4f::Zero(); // Input
Matrix4f B = Matrix4f::Identity(); // Coefficient of Input(u)


Vector4f Z = Vector4f::Zero(); // sensor measurement
Matrix4f R = Matrix4f::Zero(); // Measurement error Covariance
Matrix4f Q = Matrix4f::Zero(); // Process error Covariance
Matrix4f F = Matrix4f::Identity(); // Jacobian Matrix
Matrix4f H = Matrix4f::Identity(); // Coefficient Matrix

Matrix4f K; // Kalman Gain
Vector4f Y; // measurement residual
Matrix4f S; // residual covariance

Matrix4f P_k_k = Matrix4f::Zero(); // P_k|k
Matrix4f P_k_i = Matrix4f::Zero(); // P_k|k-1
Matrix4f P_i_i = Matrix4f::Zero(); // P_k-1|k-1
Vector4f P_i_i_returned = Vector4f::Zero();  // State vector X_k|k
Matrix4f I = Matrix4f::Identity(); // 2 x 2 Identity Matrix

Vector3f V = Vector3f::Zero();
Vector3f V_avg = Vector3f::Zero(); // Average velocity of 3 steps
MatrixXf Vmatrix(5,3); // Queue, holds velocity values


// Additional variable declaration, no initialization
bool got_pose_, got_vel_, stationary_;
double yaw;
double Xold, Yold, Zold;
double T = 30; // ROS loop rate
double T1,T2;
 

// Position and movement messages
geometry_msgs::PoseStamped measurementPose; //measurement Position
geometry_msgs::Twist twist; // Input for quadcopter actions


MatrixXf InputQueue(4,30);
Vector4f bak_X = Vector4f::Zero();
Matrix4f bak_P = Matrix4f::Zero();
Vector2f F2 = Vector2f::Zero();


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// Begins : FUNCTIONS /////////////////////////////////////////////

// Updates input 
void inputCallback(const geometry_msgs::Twist::ConstPtr& inputPtr)
{
	twist.linear=inputPtr->linear;
	twist.angular=inputPtr->angular;
}



// Updates position coordinatesgeometry_msgs::Twist twistEstimation;
void poseCallback(const tf2_msgs::TFMessage::ConstPtr& posePtr)
{
	const tf2_msgs::TFMessage& msg=*posePtr;
	if (msg.transforms[0].header.frame_id.compare("ORB_SLAM/World")==0){
	got_pose_ = true;
	std::cout<<"pass";
	
	// FIXME: Set found agent's position
	// FIXME: NOT SURE ABOUT PITCH AND ROLL

	measurementPose.header.frame_id=msg.transforms[0].child_frame_id;
	Xold = measurementPose.pose.position.x;
	Yold = measurementPose.pose.position.y;
	Zold = measurementPose.pose.position.z;
	measurementPose.pose.position.x = msg.transforms[0].transform.translation.z;
	measurementPose.pose.position.y = msg.transforms[0].transform.translation.x;
	measurementPose.pose.position.z = -msg.transforms[0].transform.translation.y;
	measurementPose.pose.orientation.x = msg.transforms[0].transform.rotation.z;
	measurementPose.pose.orientation.y = msg.transforms[0].transform.rotation.x;
	measurementPose.pose.orientation.z = -msg.transforms[0].transform.rotation.y;
	measurementPose.pose.orientation.w = msg.transforms[0].transform.rotation.w;
	// measurementPose.pose.orientation = posePtr->transforms.transform.rotation;

	yaw = tf::getYaw(measurementPose.pose.orientation);
   
	T1=T2; 
	T2=ros::Time::now().toSec();
    }
}

void runEKF(const MatrixXf &InputQueue, Vector4f Z, Vector4f X_i_i, Matrix4f P_i_i, Vector2f F2)
 {
//Vector4f X_k_i = Vector4f::Zero(); // State vector X_k|k-1
Vector4f X_k_k = Vector4f::Zero();  // State vector X_k|k
Vector4f U2 = Vector4f::Zero(); // Input
//Matrix4f B = Matrix4f::Identity(); // Coefficient of Input(u)
//Matrix4f S2; // residual covariance
Matrix4f P_k_k = Matrix4f::Zero(); // P_k|k


	Q(0,0)=1;
	Q(1,1)=1;
	Q(2,2)=1;
	Q(3,3)=1;
	
	got_pose_ = true;
	
	int count = 0;
	while (count <= InputQueue.cols())
	 {	
	

	if (got_pose_ == true)
        {
            std::cout<<"got_pose_: "<<got_pose_<<"\n";
            R(0,0)=1;
            R(1,1)=1;
            R(2,2)=1;
            R(3,3)=1;
        }
        else
        {
            R(0,0)=10000;
            R(1,1)=10000;
            R(2,2)=10000;
            R(3,3)=10000;
        }		

		U2(0) = InputQueue(0,count);
		U2(1) = InputQueue(1,count);
		U2(2) = InputQueue(1,count);
		U2(3) = InputQueue(1,count);

		// Predictor 
		//X_k_i = F*X_i_i + B*U;	// a priori state estimate
		X_k_i = X_i_i + B*U2;		// a priori state estimate
	
		if (got_pose_)
		{		
		
		F << 1, 0,0, F2(0),0, 1,0, F2(1),0, 0, 1,0, 0,0,0,-1;
	
		// P_k|k-1
		P_k_i = F*P_i_i*F.transpose() + H*Q*H.transpose();	// a priori covariance
		
		// Corrector
		Y = Z - H*X_k_i; 			// Measurement Residual
		S = H*P_k_i*H.transpose() + R;		// M. Residual Covariance
		K = (P_k_i*H.transpose())*S.inverse();	// Kalman Gain
		
		X_k_k = X_k_i + K*Y; 			// a posteriori state estimate
		P_k_k = (I - K*H)*P_k_i;		// a posetriori covariance
	
		}

		X_i_i = X_k_k;
		P_i_i = P_k_k;
		
		bak_X = X_k_k;
		bak_P = P_k_k;
		
		got_pose_ = false;	
	 }

	X_i_i_returned = X_i_i;
	//P_k_i_returned = P_k_i;
 } 




////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Begins : MAIN /////////////////////////////////////////////



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Quadrotor_EKF_version_2");
	ros::start();

	void poseCallback(const tf2_msgs::TFMessage::ConstPtr& pose);
	void inputCallback(const geometry_msgs::Twist::ConstPtr&);

	got_pose_ = false;
	stationary_ = false;

	T2 = ros::Time::now().toSec(); 	
		
//	Q(0,0)=0;
//	Q(1,1)=0;
//	Q(2,2)=0;
//	Q(3,3)=0;
//	R(0,0)=.01;
//	R(1,1)=.01;
//	R(2,2)=.01;
//	R(3,3)=.01;
	P_i_i(0,0)=1000;
	P_i_i(1,1)=1000;
	P_i_i(2,2)=1000;
	P_i_i(3,3)=1000;

	geometry_msgs::PoseStamped poseEstimation;
	poseEstimation.pose.position.x=0;
	poseEstimation.pose.position.y=0;
	poseEstimation.pose.position.z=0;	
	geometry_msgs::Twist velocityEstimation;
	poseEstimation.pose.orientation=tf::createQuaternionMsgFromYaw(0);
	

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// Begins : ROS ///////////////////////////////////////////////////

 	ros::NodeHandle n;
	ros::Subscriber sub_pose_ , sub_input_ ;
	ros::Publisher pub_pose_, pub_vel_;
	ros::Rate loop_rate(T);

	
	sub_pose_ = n.subscribe<tf2_msgs::TFMessage>("/tf", 1,poseCallback);	
	sub_input_ = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1,inputCallback);

	pub_pose_ = n.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	pub_vel_ = n.advertise<geometry_msgs::Twist>("/velocityEstimation", 1000, true);
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////


while (ros::ok())
  {
	got_pose_ = false; 
	//got_vel_ = false;
	ros::spinOnce();

 //Conditionals
        if (got_pose_)
        {
            //cout<<"got_pose_: "<<got_pose_<<"\n";
            R(0,0)=1;
            R(1,1)=1;
            R(2,2)=1;
            R(3,3)=1;
        }
        else
        {
            R(0,0)=10000;
            R(1,1)=10000;
            R(2,2)=10000;
            R(3,3)=10000;
        }
        
        if (stationary_)
        {
            Q(0,0)=0;
            Q(1,1)=0;
            Q(2,2)=0;
            Q(3,3)=0;
        }
        else
        {
            Q(0,0)=100;
            Q(1,1)=100;
            Q(2,2)=100;
            Q(3,3)=100;
        }


// Array variable initializations
	U(0) = twist.linear.x*cos(yaw)-twist.linear.y*sin(yaw)/T;
	U(1) = -twist.linear.y*cos(yaw)+twist.linear.x*sin(yaw)/T;
	U(2) = twist.linear.z/T;
	U(3) = twist.angular.z;
	

	for (int column = 0; column <= Vmatrix.cols(); ++column)
	 {
		for (int row = Vmatrix.rows(); row > 0; --row)
		 {
	 		Vmatrix(row,column) = Vmatrix(row-1,column);
		 }
	  }

        Vmatrix(0,0)=(measurementPose.pose.position.x-Xold)/(T2-T1);
        Vmatrix(0,1)=(measurementPose.pose.position.y-Yold)/(T2-T1);
        Vmatrix(0,2)=(measurementPose.pose.position.z-Zold)/(T2-T1);


	V_avg(0) =(Vmatrix(2,0)+Vmatrix(1,0)+Vmatrix(0,0))/3;
	V_avg(1) =(Vmatrix(2,1)+Vmatrix(1,1)+Vmatrix(0,1))/3;
	V_avg(2) =(Vmatrix(2,2)+Vmatrix(1,2)+Vmatrix(0,2))/3;
	
	Z(0)= measurementPose.pose.position.x;
	Z(1) = measurementPose.pose.position.y;
	Z(2) = measurementPose.pose.position.z;
	Z(3) = yaw;

	
	if (got_pose_)
	 {
		F2(0) = -twist.linear.x/T*sin(yaw)-twist.linear.y/T*cos(yaw);
		F2(1) =  twist.linear.x/T*cos(yaw)+twist.linear.y/T*sin(yaw);
		runEKF(InputQueue, Z, bak_X, bak_P, F2);
		
		X_i_i = X_i_i_returned;
		//P_i_i = P_i_i_returned;	
	 }

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////// Extended-Kalman Filter ///////////////////////////////////////

// Predictor 
	//X_k_i = F*X_i_i + B*U;	// a priori state estimate
	X_k_k = X_i_i + B*U;		// a priori state estimate

//if (got_pose_)
// {
//
///	
//	// P_k|k-1
//	P_k_i = F*P_i_i*F.transpose() + H*Q*H.transpose();	// a priori covariance
//
//	// Corrector
//	Y = Z - H*X_k_i; 			// Measurement Residual
//	S = H*P_k_i*H.transpose() + R;		// M. Residual Covariance
//	K = (P_k_i*H.transpose())*S.inverse();	// Kalman Gain
//
//	X_k_k = X_k_i + K*Y; 			// a posteriori state estimate
//	P_k_k = (I - K*H)*P_k_i;		// a posetriori covariance
//	
//	V = V_avg;
//	
// }

V = V_avg;

//////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

	velocityEstimation.linear.x= V(0);
	velocityEstimation.linear.y= V(1);
	velocityEstimation.linear.z= V(2);				 
	
	//X_i_i = X_k_k;
	P_i_i = P_k_k;


	poseEstimation.pose.position.x = X_k_k(0);
	poseEstimation.pose.position.y = X_k_k(1);
	poseEstimation.pose.position.z = X_k_k(2);
	poseEstimation.pose.orientation = tf::createQuaternionMsgFromYaw(X_k_k(3));
	
	pub_pose_.publish(poseEstimation);
        pub_vel_.publish(velocityEstimation);

	std::cout<<"//////////////////////////////////////////////////////////////////////";
	std::cout<<"\n Measured: \n"<<measurementPose<<"\n";
        std::cout<<"Best Position Estimation :\n"<<poseEstimation<<"\n //////////////// \n\n";
        std::cout<<"Yaw: "<<yaw<<"\n---------\n\n";
        std::cout<<"//////////////////////////////////////////////////////////////////////";

	

	for (int column = 0; column < InputQueue.cols(); ++column)
	 {
		for (int row = 0; row <= InputQueue.rows(); ++row)
		 {
	 		InputQueue(row,column) = InputQueue(row,column + 1);
		 }
	  }
	
	
	InputQueue(0,InputQueue.cols()) = U(0);
	InputQueue(1,InputQueue.cols()) = U(1);
	InputQueue(2,InputQueue.cols()) = U(2);
	InputQueue(3,InputQueue.cols()) = U(3);

	
	
	loop_rate.sleep();
	
  }
	return 0;
}

