// SPDX-License-Identifier: CC-BY-NC-ND-4.0
// Copyright (c) Javadian. All rights reserved.

//============================================================================
// Name        : control.cpp
// Description : Human Robot interface with gesture demonstration techniques
//============================================================================
/*This program tracks the human and receives the commands from the user and 
controls the robot. Kinect camera is used to observe the gestures. The user
can control the translational velocity of the robot by moving toward it, and 
control the rotational velocity by simulating the steering wheel movement.
*/
//edit= pan_size, vel, user_lost, left_right
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include "std_msgs/Float32.h"
#include <turtlesim/Velocity.h>
#include <rosgraph_msgs/Log.h>
#include <iostream>
#include <math.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <sound_play/sound_play.h>
#include <algorithm>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#include <sstream>
#include <string>
#include "std_msgs/String.h"
#include <fstream>
//#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <iomanip>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ctime>
#include <time.h>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif
#define PI 3.14159290
#include "std_msgs/Bool.h"
using namespace std;

double dot(double x1, double y1 , double x2, double y2);
double angle(double,double,double,double,double,double,double,double,double ,double );
void maestro(int );
void init_maestro(void);
const double init_tilt(double, double);
int maestroGetPosition(int , unsigned char );
int maestroSetTarget(int , unsigned char , unsigned short );
int maestroSetSpeed(int , unsigned char , unsigned short );
//global variables
int counter_filter = 0;
double filter[5]={0,0,0,0,0}, distance_Max;
bool flag_calibration=false;
int Velcount = 0;
int desired_pan;
double initial_offset=5;
clock_t start;
double duration;
double real_angle, ranged_angle;


class Sonar
{
public:

  void sonar(const sensor_msgs::PointCloud::ConstPtr& sonar_msg)
  {
	for(int i(0);i<=15;i++)
	{
		sonar_x[i] = (sonar_msg->points[i].x)*100;	
		sonar_y[i] = (sonar_msg->points[i].y)*100;	
	}
  }
  float sonar_x[16], sonar_y[16];
};

class robot_pose
{
public:

  void vel_odo(const nav_msgs::Odometry::ConstPtr& odo_msg)
  {
	angular_odo = odo_msg->twist.twist.angular.z;
	linear_odo = odo_msg->twist.twist.linear.x;
	odo_x = odo_msg->pose.pose.position.x;
	odo_y = odo_msg->pose.pose.position.y;
//Quaternion
	odo_w = odo_msg->pose.pose.orientation.w;
	odo_z = odo_msg->pose.pose.orientation.z;
	
  }
  void pose_localization(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
  {
		px = pose_msg->pose.pose.position.x;
		py = pose_msg->pose.pose.position.y;
		qz = pose_msg->pose.pose.orientation.z;
		qw = pose_msg->pose.pose.orientation.w;
  }
  float linear_odo, angular_odo , px, py, qw, qz , odo_x , odo_y , odo_z , odo_w;
};

class lost_user
{
public:

  void recognition(const rosgraph_msgs::Log::ConstPtr& message)
  {
	string str1 = message->msg.c_str();
	for (int i(1);i<=4;i++)
	{
		string str2 ("Lost user %d",i);
		if(str1.compare(str2) == 0) user_number=i;
	}
	string str3 ("Lost user 1");
	string str4 ("Lost user 2");
	string str5 ("Lost user 3");
	string str6 ("Lost user 4");
	if(str1.compare(str3) == 0) user_number=1;
	if(str1.compare(str4) == 0) user_number=2;
	if(str1.compare(str5) == 0) user_number=3;
	if(str1.compare(str6) == 0) user_number=4;
  }
  int user_number;
};

int main(int argc, char** argv)
{
	double rotational_velocity=0, distance_Min , pan_angle_acc , Gesture_angle;
//positions in Camera Farme
	double head_CF[3] , neck_CF[3], left_hand_CF[3], right_hand_CF[3], torso_CF[3];
//positions in Image Frame (Y,X)
	double head_IM[2] , neck_IM[2], left_hand_IM[2], right_hand_IM[2], torso_IM[2];
	double rotational_velocity_max = 7;
	double rotation_range_max = 75;
	double translational_velocity_max=0.3, translational_velocity , gradient , decreasing_velocity , real_distance, gained_error;
	double error , error_l , error_r;
	int pan_angle=0 , init_obst = 0;
	float sonar_distance[8], sonar_min = 100;
	std_msgs::Float32MultiArray msgs_;
	float msgs[9];
	std_msgs::Float64 tilt_angle;
	std_msgs::Bool flag;
	flag.data = false;
	clock_t cknow;
	time_t current;
	char* local_date;

	CvMat* object_points	= cvCreateMat( 3, 1, CV_64FC1 );
	CvMat* rotation_vector	= cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* translation_vector = cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* intrinsic_matrix	= cvCreateMat( 3, 3, CV_64FC1 );
	CvMat* distortion_coeff	= cvCreateMat( 5, 1, CV_32FC1 );
	CvMat* xy	= cvCreateMat( 2, 1, CV_64FC1 );
  
	double e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,u1[5]={0,0,0,0,0},u2[5]={0,0,0,0,0},u3[5]={0,0,0,0,0},u4[5]={0,0,0,0,0},u5[5]={0,0,0,0,0},u6[5]={0,0,0,0,0},u7[5]={0,0,0,0,0},u8[5]={0,0,0,0,0},u9[5]={0,0,0,0,0},u10[5]={0,0,0,0,0},u11[5]={0,0,0,0,0},u12[5]={0,0,0,0,0},u13[5]={0,0,0,0,0},u14[5]={0,0,0,0,0},u15[5]={0,0,0,0,0} , velct = 0;
	int user_flag = 0 , checking = 0;
	double robot_linear, robot_angular;

	Sonar sensor;
	lost_user fig;
	robot_pose odo;

	ros::init(argc, argv, "set_rotation");
	ros::NodeHandle node;

	tf::TransformListener listener;
//	ros::Publisher robotrotationvel = node.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
	ros::Publisher robotrotationvel = node.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
	ros::Publisher image_    = node.advertise<std_msgs::Float32MultiArray>("control/frames", 1);
	ros::Publisher record    = node.advertise<std_msgs::Bool>("control/record", 1);
	ros::Publisher gaze_tilt    = node.advertise<std_msgs::Float64>("tilt_angle", 1);
	ros::Subscriber sub = node.subscribe("/RosAria/sonar", 1, &Sonar::sonar, &sensor);
	ros::Subscriber sub_ = node.subscribe("rosout_agg", 100, &lost_user::recognition, &fig);
	ros::Subscriber sub_odo = node.subscribe("/RosAria/pose", 1, &robot_pose::vel_odo, &odo);
	ros::Subscriber sub_odos = node.subscribe("/amcl_pose", 1, &robot_pose::pose_localization, &odo);
	sound_play::SoundClient state;
	current = time(0);
	local_date = ctime(&current);

	tilt_angle.data = 7.0;
		//translational velocity: m/s
//		turtlesim::Velocity vel;
		geometry_msgs::Twist vel;
		vel.angular.x = 0;
		vel.angular.z = 0;
		vel.angular.y = 0;
		vel.linear.x = 0;
		vel.linear.z = 0;
		vel.linear.y = 0;

	if (argc != 4 )
	{
		ROS_ERROR("Give the name of the folder and the file you want to create to save the demonstration");
		return 0;
	}

//Creating the folder
	std::string folder_name(argv[1]);
	//std::string path_name = "${HOME}/ros_workspace/save_Demonstrations/" + folder_name + "/";
	std::string path_name = "${ROS_WORKSPACE}/HRI_Gesture_Demo/" + folder_name + "/";
	boost::filesystem::create_directories(path_name);
	std::string frame_name;


// Changing the string from the argument to character so it can be used by the output file stream (ofstream)
	std::string file_name(argv[2]);
	file_name = path_name + file_name;
	const char *p;
	p = file_name.c_str();
	static std::ofstream out_file(p);
	desired_pan = (boost::lexical_cast<int>(argv[3]));
	//int  desired_pan(boost::lexical_cast<int>(argv[3]));
	//cout << "Chiz" << desired_pan;
	//printf("\n chis %d\n",desired_pan);
	init_maestro();

	ros::Rate rate(10.0);
	while (node.ok()){
		cknow = clock();
		if (tilt_angle.data == 7) 
		{
			//robotrotationvel.publish(vel);
			gaze_tilt.publish(tilt_angle);
			//sleep(3);

		}
		tf::StampedTransform transform_head;
		tf::StampedTransform transform_torso;
		tf::StampedTransform transform_right_hand;
		tf::StampedTransform transform_left_hand;
		tf::StampedTransform transform_neck;
		try{
			listener.lookupTransform("/head_1", "/openni_depth_frame",  ros::Time(0), transform_head);
			listener.lookupTransform("/left_hand_1", "/openni_depth_frame",  ros::Time(0), transform_left_hand);
			listener.lookupTransform("/right_hand_1", "/openni_depth_frame",  ros::Time(0), transform_right_hand);
			listener.lookupTransform("/neck_1", "/openni_depth_frame",  ros::Time(0), transform_neck);
			listener.lookupTransform("/torso_1", "/openni_depth_frame",  ros::Time(0), transform_torso);
			}
		catch (tf::TransformException ex){
			//ROS_ERROR("%s",ex.what());
			}
			if(transform_head.getOrigin().y()==0&&transform_head.getOrigin().z()==0&&transform_left_hand.getOrigin().y()==0&&transform_left_hand.getOrigin().z()==0&&transform_right_hand.getOrigin().y()==0&&transform_right_hand.getOrigin().z()==0&transform_torso.getOrigin().y()==0&&transform_torso.getOrigin().z()==0&&transform_left_hand.getOrigin().x()==0&&transform_right_hand.getOrigin().x()==0&&transform_head.getOrigin().y()==0)
			{
				try{
					listener.lookupTransform("/head_2", "/openni_depth_frame",  ros::Time(0), transform_head);
					listener.lookupTransform("/left_hand_2", "/openni_depth_frame",  ros::Time(0), transform_left_hand);
					listener.lookupTransform("/right_hand_2", "/openni_depth_frame",  ros::Time(0), transform_right_hand);
					listener.lookupTransform("/neck_2", "/openni_depth_frame",  ros::Time(0), transform_neck);
					listener.lookupTransform("/torso_2", "/openni_depth_frame",  ros::Time(0), transform_torso);
			}
				catch (tf::TransformException ex){
					//ROS_ERROR("%s",ex.what());
				}	
			if(transform_head.getOrigin().y()==0&&transform_head.getOrigin().z()==0&&transform_left_hand.getOrigin().y()==0&&transform_left_hand.getOrigin().z()==0&&transform_right_hand.getOrigin().y()==0&&transform_right_hand.getOrigin().z()==0&transform_torso.getOrigin().y()==0&&transform_torso.getOrigin().z()==0&&transform_left_hand.getOrigin().x()==0&&transform_right_hand.getOrigin().x()==0&&transform_head.getOrigin().y()==0)
			{
			try{
					listener.lookupTransform("/head_3", "/openni_depth_frame",  ros::Time(0), transform_head);
					listener.lookupTransform("/left_hand_3", "/openni_depth_frame",  ros::Time(0), transform_left_hand);
					listener.lookupTransform("/right_hand_3", "/openni_depth_frame",  ros::Time(0), transform_right_hand);
					listener.lookupTransform("/neck_3", "/openni_depth_frame",  ros::Time(0), transform_neck);
					listener.lookupTransform("/torso_3", "/openni_depth_frame",  ros::Time(0), transform_torso);
			}
				catch (tf::TransformException ex){
					//ROS_ERROR("%s",ex.what());
				}
			if(transform_head.getOrigin().y()==0&&transform_head.getOrigin().z()==0&&transform_left_hand.getOrigin().y()==0&&transform_left_hand.getOrigin().z()==0&&transform_right_hand.getOrigin().y()==0&&transform_right_hand.getOrigin().z()==0&transform_torso.getOrigin().y()==0&&transform_torso.getOrigin().z()==0&&transform_left_hand.getOrigin().x()==0&&transform_right_hand.getOrigin().x()==0&&transform_head.getOrigin().y()==0)
			{
			try{
					listener.lookupTransform("/head_4", "/openni_depth_frame",  ros::Time(0), transform_head);
					listener.lookupTransform("/left_hand_4", "/openni_depth_frame",  ros::Time(0), transform_left_hand);
					listener.lookupTransform("/right_hand_4", "/openni_depth_frame",  ros::Time(0), transform_right_hand);
					listener.lookupTransform("/neck_4", "/openni_depth_frame",  ros::Time(0), transform_neck);
					listener.lookupTransform("/torso_4", "/openni_depth_frame",  ros::Time(0), transform_torso);
			}
				catch (tf::TransformException ex){
					//ROS_ERROR("%s",ex.what());
				}
				
			}
			
			}
			
			}
			if(listener.canTransform("/head_1", "/openni_depth_frame", ros::Time(0), NULL)==true) user_flag = 1;
			if(listener.canTransform("/head_2", "/openni_depth_frame", ros::Time(0), NULL)==true) user_flag = 2;
			if(listener.canTransform("/head_3", "/openni_depth_frame", ros::Time(0), NULL)==true) user_flag = 3;
			if(listener.canTransform("/head_4", "/openni_depth_frame", ros::Time(0), NULL)==true) user_flag = 4;
		// calibration projection
		// camera calibration matrix
		const double intrinsic[3][3]={{587.04607160, 0, 317.39001517},{0, 587.04607160, 234.30080720},{0,0,1}};
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				cvSetReal2D(intrinsic_matrix, i, j, intrinsic[i][j]);
				float val = cvGetReal2D(intrinsic_matrix,i,j);//check val
			}
		}
		for(int k=0;k<5;k++){
			cvSetReal2D(distortion_coeff, k, 0, 0);
		}
		for(int l=0;l<3;l++){
			cvSetReal2D(rotation_vector, l, 0, 0);
		}
		for(int z=0;z<3;z++){
			cvSetReal2D(translation_vector, z, 0, 0);
		}
		// set transforms to matrix
		head_CF[0]=-1 * transform_head.getOrigin().z();
		head_CF[1]=transform_head.getOrigin().y();
		head_CF[2]=transform_head.getOrigin().x();
		cvSetReal2D(object_points, 0, 0, head_CF[1]);
		cvSetReal2D(object_points, 1, 0, head_CF[2]);
		cvSetReal2D(object_points, 2, 0, head_CF[0]);
		cvProjectPoints2(object_points,rotation_vector,translation_vector,intrinsic_matrix,distortion_coeff,xy);
		
		// positions in the image frame
		head_IM[0]=cvGetReal2D(xy, 0, 0);
		head_IM[1]=cvGetReal2D(xy, 1, 0)+100;

		torso_CF[0]=-1 *transform_torso.getOrigin().z();
		torso_CF[1]=transform_torso.getOrigin().y();
		torso_CF[2]=transform_torso.getOrigin().x();
		cvSetReal2D(object_points, 0, 0, torso_CF[1]);
		cvSetReal2D(object_points, 1, 0, torso_CF[2]);
		cvSetReal2D(object_points, 2, 0, torso_CF[0]);
		cvProjectPoints2(object_points,rotation_vector,translation_vector,intrinsic_matrix,distortion_coeff,xy);
		
		// positions in the image frame
		torso_IM[0]=cvGetReal2D(xy, 0, 0);
		torso_IM[1]=cvGetReal2D(xy, 1, 0)+100;

		neck_CF[0]=-1 *transform_neck.getOrigin().z();
		neck_CF[1]=transform_neck.getOrigin().y();
		neck_CF[2]=transform_neck.getOrigin().x();
		cvSetReal2D(object_points, 0, 0, neck_CF[1]);
		cvSetReal2D(object_points, 1, 0, neck_CF[2]);
		cvSetReal2D(object_points, 2, 0, neck_CF[0]);		
		cvProjectPoints2(object_points,rotation_vector,translation_vector,intrinsic_matrix,distortion_coeff,xy);
		//positions in the image frame
		neck_IM[0]=cvGetReal2D(xy, 0, 0);
		neck_IM[1]=cvGetReal2D(xy, 1, 0)+100;

		right_hand_CF[0]=-1 *transform_right_hand.getOrigin().z();
		right_hand_CF[1]=transform_right_hand.getOrigin().y();
		right_hand_CF[2]=transform_right_hand.getOrigin().x();
		cvSetReal2D(object_points, 0, 0, right_hand_CF[1]);
		cvSetReal2D(object_points, 1, 0, right_hand_CF[2]);
		cvSetReal2D(object_points, 2, 0, right_hand_CF[0]);
		cvProjectPoints2(object_points,rotation_vector,translation_vector,intrinsic_matrix,distortion_coeff,xy);

		//positions in the image frame
		right_hand_IM[0]=cvGetReal2D(xy, 0, 0)-70;
		right_hand_IM[1]=cvGetReal2D(xy, 1, 0)+80;

		left_hand_CF[0]=-1 *transform_left_hand.getOrigin().z();
		left_hand_CF[1]=transform_left_hand.getOrigin().y();
		left_hand_CF[2]=transform_left_hand.getOrigin().x();
		cvSetReal2D(object_points, 0, 0, left_hand_CF[1]);
		cvSetReal2D(object_points, 1, 0, left_hand_CF[2]);
		cvSetReal2D(object_points, 2, 0, left_hand_CF[0]);
		cvProjectPoints2(object_points,rotation_vector,translation_vector,intrinsic_matrix,distortion_coeff,xy);

		//positions in the image frame
		left_hand_IM[0]=cvGetReal2D(xy, 0, 0)-70;
		left_hand_IM[1]=cvGetReal2D(xy, 1, 0)+80;
		
		//rotational velocity: angle/s 
		rotational_velocity = angle(head_IM[1],head_IM[0],left_hand_IM[1],left_hand_IM[0],right_hand_IM[1],right_hand_IM[0],torso_IM[1],torso_IM[0],rotational_velocity_max,rotation_range_max);

		Gesture_angle = rotational_velocity;

		if(transform_head.getOrigin().y()==0&&transform_head.getOrigin().z()==0&&transform_left_hand.getOrigin().y()==0&&transform_left_hand.getOrigin().z()==0&&transform_right_hand.getOrigin().y()==0&&transform_right_hand.getOrigin().z()==0&transform_torso.getOrigin().y()==0&&transform_torso.getOrigin().z()==0&&transform_left_hand.getOrigin().x()==0&&transform_right_hand.getOrigin().x()==0&&transform_head.getOrigin().y()==0) rotational_velocity=0;
		//filtering to smooth the angle: mean filter of order 5
		for(counter_filter=4;counter_filter>0;counter_filter--)
		{
			filter[counter_filter]=filter[counter_filter-1];
		}
		filter[0]=rotational_velocity;
		rotational_velocity=(filter[0]+filter[1]+filter[2]+filter[3]+filter[4])/5;
		
		//Safety		
		for(int k(0);k<=8;k++)
		{
			sonar_distance[k] = sqrt((sensor.sonar_x[k]*sensor.sonar_x[k])+(sensor.sonar_y[k]*sensor.sonar_y[k]));
		}
		sonar_min = *min_element(sonar_distance+2,sonar_distance+6);
		robot_angular = odo.angular_odo;
		robot_linear = odo.linear_odo;
//		printf("sonar %f\n", sonar_min);
//		printf("--------------------sonar min %f\n",sonar_min);
//		printf("sonar distance %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",sonar_distance[0],sonar_distance[1],sonar_distance[2],sonar_distance[3],sonar_distance[4],sonar_distance[5],sonar_distance[6],sonar_distance[7],sonar_distance[8],sonar_distance[9],sonar_distance[10],sonar_distance[11],sonar_distance[12],sonar_distance[13],sonar_distance[14],sonar_distance[15]);
//		printf("sonar X %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",sensor.sonar_x[0],sensor.sonar_x[1],sensor.sonar_x[2],sensor.sonar_x[3],sensor.sonar_x[4],sensor.sonar_x[5],sensor.sonar_x[6],sensor.sonar_x[7],sensor.sonar_x[8],sensor.sonar_x[9],sensor.sonar_x[10],sensor.sonar_x[11],sensor.sonar_x[12],sensor.sonar_x[13],sensor.sonar_x[14],sensor.sonar_x[15]);
//		printf("sonar Y  %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",sensor.sonar_y[0],sensor.sonar_y[1],sensor.sonar_y[2],sensor.sonar_y[3],sensor.sonar_y[4],sensor.sonar_y[5],sensor.sonar_y[6],sensor.sonar_y[7],sensor.sonar_y[8],sensor.sonar_y[9],sensor.sonar_y[10],sensor.sonar_y[11],sensor.sonar_y[12],sensor.sonar_y[13],sensor.sonar_y[14],sensor.sonar_y[15]);


		if((listener.canTransform("/head_1", "/openni_depth_frame", ros::Time(0), NULL)==true || listener.canTransform("/head_2", "/openni_depth_frame", ros::Time(0), NULL)==true || listener.canTransform("/head_3", "/openni_depth_frame", ros::Time(0), NULL)==true || listener.canTransform("/head_4", "/openni_depth_frame", ros::Time(0), NULL)==true) && flag_calibration==false) 
		{		
			distance_Max=((head_CF[0]+neck_CF[0]+torso_CF[0])/3);
                        distance_Min = 0.3 * distance_Max;
			if(distance_Max!=0)
			{
				flag_calibration = true;
				state.say("The Robor is ready........!");
				sleep(1);
				//state.say("Start!");
				tilt_angle.data = init_tilt(right_hand_CF[1],right_hand_CF[0]);//hight,distance
				//printf("tilt %f \n", init_tilt(right_hand_CF[1],right_hand_CF[0]));
				//printf("X %f Y %f\n", right_hand_CF[1],right_hand_CF[0]);
				
				flag.data = true;
				record.publish(flag);
				gaze_tilt.publish(tilt_angle);
				sleep(2);
				state.say("Start!");
				sleep(1);
				start = clock();
				if (not out_file) std::perror(p);
				else
				{
				out_file << std::setw(20) << "\n" << "The local date and time is: " << local_date << "\n";
				out_file << std::setw(20) << "\n" << "Initial tilt angle " << tilt_angle.data << "\n";
				out_file << std::setw(20) << "Max Distance (Initial distance): " << distance_Max << "\n";
				out_file << "Min Distance: " << distance_Min << "\n\n";
				out_file << std::setw(20) << "set len_vel" 
						 << std::setw(20) << "set rot_vel"
						 << std::setw(20) << "robot lin_vel"
						 << std::setw(20) << "robot rot_vel"
						 << std::setw(20) << "error left"
						 << std::setw(20) << "error right"
						 << std::setw(20) << "visual error"
						 << std::setw(20) << "delta Pan"
						 << std::setw(20) << "real angle"
						 << std::setw(20) << "ranged angle"
						 << std::setw(20) << "distance"
						 << std::setw(20) << "Local Pose X"
						 << std::setw(20) << "Local Pose Y"
						 << std::setw(20) << "Local Quat w"
						 << std::setw(20) << "Local Quat z"
						 << std::setw(20) << "od Pose X"
						 << std::setw(20) << "od Pose Y"
						 << std::setw(20) << "od Quat w"
						 << std::setw(20) << "od Quat z"
						 << std::setw(20) << "time" << "\n";
				}
			}			
		}
				
		gradient =  (translational_velocity_max)/(distance_Min-distance_Max); 
//		printf("gradient  %f \n",gradient);
		real_distance = ((head_CF[0]+neck_CF[0]+torso_CF[0])/3); 
		float intercept = translational_velocity_max - (gradient*distance_Min); 
//		printf("intercept  %f \n",intercept);
//		printf("trans max  %f \n",translational_velocity_max);
//		printf("Distance Max %f \n",distance_Max);
//		printf("Distance Min %f \n",distance_Min);
//		printf("Real distance %f",real_distance);

		if(real_distance<distance_Max && real_distance>distance_Min)
		{
			translational_velocity = gradient*real_distance + intercept; 
			if(translational_velocity > translational_velocity_max){ translational_velocity = translational_velocity_max; }
			if(translational_velocity<0){translational_velocity = 0;}
//			printf(" p1 velocity%f \n", translational_velocity);
		}
		else if (real_distance>distance_Max)  
		{
			rotational_velocity=0;
			translational_velocity=0;
//			printf("p2 velocity point ");
		}
		else if (real_distance<distance_Min) 
		{
			rotational_velocity = 0; // translational velocity is taken from previous value
			translational_velocity=0; 
//			printf("p3 velocity point ");
			/*decreasing_velocity = translational_velocity;
			translational_velocity = decreasing_velocity / 2;
			Velcount++;
			if (Velcount>10)  
			{
				rotational_velocity=0;
				translational_velocity=0;
			}
			start = clock();
			while (translational_velocity>0)// && sonar_min>80)
			{
				duration = ( clock() - start )/CLOCKS_PER_SEC;
				translational_velocity = translational_velocity_max - (translational_velocity_max*duration)/3;
				vel.angular.z=0;
				vel.linear.x=translational_velocity;
//				vel.angular = 0;
//				vel.linear=translational_velocity;
				robotrotationvel.publish(vel);
				printf("velocity pos 3 %f \n", translational_velocity);
			}*/
		}
		if((listener.canTransform("/head_1", "/openni_depth_frame", ros::Time(0), NULL)==false && listener.canTransform("/head_2", "/openni_depth_frame", ros::Time(0), NULL)==false && listener.canTransform("/head_3", "/openni_depth_frame", ros::Time(0), NULL)==false && listener.canTransform("/head_4", "/openni_depth_frame", ros::Time(0), NULL)==false) && (flag_calibration==true)) 
		{				
			rotational_velocity=0;
			translational_velocity=0;
			vel.angular.z = 0;
			vel.linear.x = 0;
			robotrotationvel.publish(vel);
			state.say("Lost user");
			sleep(2);
			state.say("byebye");
			sleep(1);
			duration = ( clock() - start )/CLOCKS_PER_SEC;
			out_file << std::setw(20) <<  "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << cknow << "\n";
			out_file << std::setw(20) << "lost demonstrator"
				 << std::setw(20) << "Duration  " << duration <<"  in mili:  " << ( clock() - start ) << "\n";
			break;
//			printf("velocity pos 4 %f \n", rotational_velocity);
		}

		e1=head_CF[0]-((u1[0]+u1[1]+u1[2]+u1[3]+u1[4])/5);
		e2=head_CF[1]-((u2[0]+u2[1]+u2[2]+u2[3]+u2[4])/5);
		e3=head_CF[2]-((u3[0]+u3[1]+u3[2]+u3[3]+u3[4])/5);
		e4=neck_CF[0]-((u4[0]+u4[1]+u4[2]+u4[3]+u4[4])/5);
		e5=neck_CF[1]-((u5[0]+u5[1]+u5[2]+u5[3]+u5[4])/5);
		e6=neck_CF[2]-((u6[0]+u6[1]+u6[2]+u6[3]+u6[4])/5);
		e7=torso_CF[0]-((u7[0]+u7[1]+u7[2]+u7[3]+u7[4])/5);
		e8=torso_CF[1]-((u8[0]+u8[1]+u8[2]+u8[3]+u8[4])/5);
		e9=torso_CF[2]-((u9[0]+u9[1]+u9[2]+u9[3]+u9[4])/5);
		e10=right_hand_CF[0]-((u10[0]+u10[1]+u10[2]+u10[3]+u10[4])/5);
		e11=right_hand_CF[1]-((u11[0]+u11[1]+u11[2]+u11[3]+u11[4])/5);
		e12=right_hand_CF[2]-((u12[0]+u12[1]+u12[2]+u12[3]+u12[4])/5);
		e13=left_hand_CF[0]-((u13[0]+u13[1]+u13[2]+u13[3]+u13[4])/5);
		e14=left_hand_CF[1]-((u14[0]+u14[1]+u14[2]+u14[3]+u14[4])/5);
		e15=left_hand_CF[2]-((u15[0]+u15[1]+u15[2]+u15[3]+u15[4])/5);

			//printf("data %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12,u13,u14,u15);
			//printf("datas %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",head_CF[0],head_CF[1],head_CF[2],neck_CF[0],neck_CF[1],neck_CF[2],torso_CF[0],torso_CF[1],torso_CF[2],right_hand_CF[0],right_hand_CF[1],right_hand_CF[2],left_hand_CF[0],left_hand_CF[1],left_hand_CF[2]);
			//printf("error %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \nn\n", e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15);
			//printf("data %f %f %f %f %f %f \n",u1[0],u1[1],u1[2],u1[3],u1[4],head_CF[0]);

		
		u1[0]=u1[1];
		u1[1]=u1[2];
		u1[2]=u1[3];
		u1[3]=u1[4];		
		u1[4] = head_CF[0];
		u2[0]=u2[1];
		u2[1]=u2[2];
		u2[2]=u2[3];
		u2[3]=u2[4];		
		u2[4] = head_CF[1];
		u3[0]=u3[1];
		u3[1]=u3[2];
		u3[2]=u3[3];
		u3[3]=u3[4];		
		u3[4] = head_CF[2];	
		u4[0]=u4[1];
		u4[1]=u4[2];
		u4[2]=u4[3];
		u4[3]=u4[4];		
		u4[4] = neck_CF[0];
		u5[0]=u5[1];
		u5[1]=u5[2];
		u5[2]=u5[3];
		u5[3]=u5[4];		
		u5[4] = neck_CF[1];
		u6[0]=u6[1];
		u6[1]=u6[2];
		u6[2]=u6[3];
		u6[3]=u6[4];		
		u6[4] = neck_CF[2];
		u7[0]=u7[1];
		u7[1]=u7[2];
		u7[2]=u7[3];
		u7[3]=u7[4];		
		u7[4] = torso_CF[0];
		u8[0]=u8[1];
		u8[1]=u8[2];
		u8[2]=u8[3];
		u8[3]=u8[4];		
		u8[4] = torso_CF[1];
		u9[0]=u9[1];
		u9[1]=u9[2];
		u9[2]=u9[3];
		u9[3]=u9[4];		
		u9[4] = torso_CF[2];
		u10[0]=u10[1];
		u10[1]=u10[2];
		u10[2]=u10[3];
		u10[3]=u10[4];		
		u10[4] = right_hand_CF[0];
		u11[0]=u11[1];
		u11[1]=u11[2];
		u11[2]=u11[3];
		u11[3]=u11[4];		
		u11[4] = right_hand_CF[1];
		u12[0]=u12[1];
		u12[1]=u12[2];
		u12[2]=u12[3];
		u12[3]=u12[4];		
		u12[4] = right_hand_CF[2];
		u13[0]=u13[1];
		u13[1]=u13[2];
		u13[2]=u13[3];
		u13[3]=u13[4];		
		u13[4] = left_hand_CF[0];
		u14[0]=u14[1];
		u14[1]=u14[2];
		u14[2]=u14[3];
		u14[3]=u14[4];		
		u14[4] = left_hand_CF[1];
		u15[0]=u15[1];
		u15[1]=u15[2];
		u15[2]=u15[3];
		u15[3]=u15[4];		
		u15[4] = left_hand_CF[2];

/*		if( (e1==0) && (e2==0) && (e3==0) && (e4==0) && (e5==0) && (e6==0) && (e7==0) && (e8==0) && (e9==0) && (e10==0) && (e11==0) && (e12==0) && (e13==0) && (e14==0) && (e15==0) && (flag_calibration==true)) 
		{				
			
			state.say("Lost user");
			vel.angular.z = 0;
			vel.linear.x = 0;
			robotrotationvel.publish(vel);
			sleep(3);
			state.say("goodbye!");
			sleep(2);
			//printf("\n");
			duration = ( clock() - start )/CLOCKS_PER_SEC;
			out_file << std::setw(20) <<  "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << ros::Time::now() << "\n";
			out_file << std::setw(20) << "\n lost demonstrator"
				 << std::setw(20) << "Duration" << duration << "\n";
			break;

		}*/
		printf("\nuser from print %d\n",fig.user_number);
		printf("\nuser being tracked %d\n",user_flag);
		if(fig.user_number == user_flag)
		{
			state.say("Lost user now");
			vel.angular.z = 0;
			vel.linear.x = 0;
			robotrotationvel.publish(vel);
			sleep(3);
			state.say("stop bye!");
			sleep(2);
			printf("\n");
			duration = ( clock() - start )/CLOCKS_PER_SEC;
			out_file << std::setw(20) <<  "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << cknow << "\n";
			out_file << std::setw(20) << "lost demonstrator"
				 << std::setw(20) << "Duration  " << duration <<"  in mili:  " << ( clock() - start ) << "\n";
			break;
		}

		//if(flag_calibration==true) checking = checking + 1;(checking>10) &&


//		printf("rot Velocity set to the robot  is  %f _____ %f\n", rotational_velocity , translational_velocity);
//		printf("*********************************************************\n");
//		vel.angular=0;
//		vel.angular.z=0;
//		vel.linear=translational_velocity;
//apply safety
		if (sonar_min<80) 
		{
			//rotational_velocity=0;
			translational_velocity=0;
			//if(init_obst>5) state.say("Obstacle");
			state.say("Obstacle");
			sleep(2);
			init_obst = init_obst+1;
			//sleep(1);
//			printf("sonar inside %f\n", sonar_min);
//			vel.angular = 0;
//			vel.linear = 0;
		}
		velct = -1 * rotational_velocity*PI/180;
		vel.angular.z = -1 * rotational_velocity*PI/180;
		vel.linear.x=translational_velocity;
		//vel.angular.z = 0;
		//vel.linear.x=0;
//		printf("  set to the robot  is  %f vs  %f\n", vel.angular.z , vel.linear.x);
		robotrotationvel.publish(vel);
		record.publish(flag);

		//publish the frames
		msgs[0] = right_hand_IM[1];
		msgs[1] = right_hand_IM[0];
		msgs[2] = left_hand_IM[1];
		msgs[3] = left_hand_IM[0];
		msgs[4] = head_IM[1];
		msgs[5] = head_IM[0];
		msgs[6] = translational_velocity;
		msgs[7] = real_angle;
		msgs[8] = rotational_velocity;
		msgs_.data.assign(msgs,msgs+9);
		image_.publish(msgs_);





//gaze control (visual servoing)
		if(listener.canTransform("/head_1", "/openni_depth_frame", ros::Time(0), NULL)==true || listener.canTransform("/head_2", "/openni_depth_frame", ros::Time(0), NULL)==true || listener.canTransform("/head_3", "/openni_depth_frame", ros::Time(0), NULL)==true || listener.canTransform("/head_4", "/openni_depth_frame", ros::Time(0), NULL)==true) 
		{

//center of the camera frame: 234.300807, Y 317.3  (Pixels)
			if (right_hand_IM[1]>450 || left_hand_IM[1]<180)
			{
				if(left_hand_IM[1]<180)
				{
					error_r = 180 - left_hand_IM[1];
				}
				else if(right_hand_IM[1]>450)
				{
					error_l = right_hand_IM[1]-450;
				}
				else
				{
					error = 0;
				}
			error = (error_l - error_r)/2;
//landa coeff in the Image Jacobian matrix= 0.1
				gained_error = error * 0.1;
				modf(gained_error, &pan_angle_acc);
				pan_angle = pan_angle_acc;
				if((listener.canTransform("/head_1", "/openni_depth_frame", ros::Time(0), NULL)==false && listener.canTransform("/head_2", "/openni_depth_frame", ros::Time(0), NULL)==false && listener.canTransform("/head_3", "/openni_depth_frame", ros::Time(0), NULL)==false && listener.canTransform("/head_4", "/openni_depth_frame", ros::Time(0), NULL)==false)) pan_angle = 0;
				maestro(pan_angle);
			}
			else maestro(0);
		}

		
//empty the buffer    
		transform_head.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform_left_hand.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform_right_hand.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform_torso.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform_neck.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		

//stop Gesture
		if ((fabs(left_hand_IM[0]-right_hand_IM[0])<10) && (fabs(left_hand_IM[1]-right_hand_IM[1])<10) && left_hand_CF[0]!=0 && left_hand_CF[1]!=0 && right_hand_CF[0]!=0 && right_hand_CF[1]!=0)
		{
			vel.angular.z = 0;
			vel.linear.x = 0;
			robotrotationvel.publish(vel);	
			state.say("Stop Gesture!");
			sleep(3);
			state.say("goodbye");
			sleep(1);
			//printf("Gesture");
			duration = ( clock() - start )/CLOCKS_PER_SEC;
			out_file << std::setw(20) <<  "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << "0"
				 << std::setw(20) << cknow << "\n";
			out_file << std::setw(20) << "\n stop gesture"
				 << std::setw(20) << "Duration" << duration << "\n";
			break;
					
		}
		if(flag_calibration==true)
		{
			if (not out_file) std::perror(p);
			else 
			{
				out_file << std::setw(20) <<  translational_velocity
					     << std::setw(20) << velct
					     << std::setw(20) << odo.linear_odo
					     << std::setw(20) << odo.angular_odo
					     << std::setw(20) << error_l
					     << std::setw(20) << error_r
					     << std::setw(20) << error
					     << std::setw(20) << pan_angle
					     << std::setw(20) << real_angle
					     << std::setw(20) << ranged_angle
					     << std::setw(20) << real_distance
					     << std::setw(20) << odo.px
					     << std::setw(20) << odo.py
					     << std::setw(20) << odo.qw
					     << std::setw(20) << odo.qz
						 << std::setw(20) << odo.odo_x
						 << std::setw(20) << odo.odo_y
						 << std::setw(20) << odo.odo_w
						 << std::setw(20) << odo.odo_z
					     << std::setw(20) << cknow << "\n";
			}
		}


		ros::spinOnce();
		rate.sleep();
		
	}
//pointer deconstructor
	cvReleaseMat(&object_points);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&translation_vector);
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeff);
	cvReleaseMat(&xy);
	return 0;
};
//supplementary functions
//dot product calculations
double dot(double x1, double y1 , double x2, double y2)
{
	double	ans;
	ans = (x1 * x2) + (y1 * y2);
	return ans;
}
//rotational angle calculation
double angle(double head_x,double head_y,double left_x,double left_y,double right_x,double right_y,double torso_x,double torso_y, double max_velocity, double max_range)
{
	double rotation_angle , t1_x , t1_y , t2_x , t2_y, temp, size_a, size_z;
//calculation vectors
	t1_x= head_x - torso_x;
	t1_y= head_y - torso_y;
	t2_x= right_x - left_x;
	t2_y= right_y - left_y;
	size_a = sqrt((t1_x*t1_x)+(t1_y*t1_y));
	size_z = sqrt((t2_x*t2_x)+(t2_y*t2_y));
	rotation_angle = acos (dot(t1_x,t1_y,t2_x,t2_y)/(size_a*size_z));       
	rotation_angle = rotation_angle * (180 / PI);
	temp = 90 - rotation_angle;
	real_angle = temp;
//ranging the angle
	if (temp>max_range) rotation_angle=max_range;
	else if (temp<-max_range) rotation_angle=-max_range;
	else if (temp>-initial_offset && temp<initial_offset) rotation_angle=0;
	else rotation_angle = temp;
	ranged_angle = rotation_angle;
	rotation_angle = (rotation_angle*max_velocity)/max_range;
	return rotation_angle;
}
// Gets the position of a Maestro channel
int maestroGetPosition(int fd, unsigned char channel)
{
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }   
  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }   
  return response[0] + 256*response[1];
}
// Sets the target of a Maestro channel.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}
// Sets the speed of a Maestro channel.
// The units of 'speed' are 0.25 μs / (10 ms)
int maestroSetSpeed(int fd, unsigned char channel, unsigned short speed)
{
  unsigned char command[] = {0x87, channel, speed & 0x7F, speed >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}
// Target and Speed are sent to the maestro channel
void maestro(int w)
{
  const char * device = "/dev/ttyACM0";
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    perror(device);
    return;
  }
#ifndef _WIN32
  struct termios options;
  tcgetattr(fd, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(fd, TCSANOW, &options);
#endif   
  int position = maestroGetPosition(fd, 0);
  int  position_now = (position - 6000)/45;
  int degree = position_now + w ; 
//target practical limitation -18 and +38
  if (degree>90) degree = 90;
  if (degree<-90) degree = -90;
  int target = ( degree*45 ) +  6000;
  maestroSetSpeed(fd, 0, 5);
  maestroSetTarget(fd, 0, target);
  close(fd);
}
void init_maestro(void)
{
  const char * device = "/dev/ttyACM0";
  int fd = open(device, O_RDWR | O_NOCTTY);
   int pan_ini_angle=(45*desired_pan)+6000;
  printf("\n %d\n",desired_pan);
  if (fd == -1)
  {
    perror(device);
  }
#ifndef _WIN32
  struct termios options;
  tcgetattr(fd, &options);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~(ONLCR | OCRNL);
  tcsetattr(fd, TCSANOW, &options);
#endif   
 
  maestroSetSpeed(fd, 0, 5);
  maestroSetTarget(fd, 0,    pan_ini_angle); 
  close(fd);
}
const double init_tilt(double y, double d)
{
	double init = (d+0.5)/4;
	double z = -y + init;
	double z_ = init + 0.3;//desired offset
	d = sqrt((d*d)-(y*y));//real
	//printf("init %f 	 %f\n",init,d);
	double tilt_angle = (atan(z/d)-atan(z_/d))*180/PI;
	return tilt_angle;

	
}
