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
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/package.h>
#include "std_msgs/Float32.h"
#include <turtlesim/Velocity.h>
#include <iostream>
#include <math.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <ctime>
#include <time.h>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif
#define PI 3.14159290
using namespace std;

double dot(double x1, double y1 , double x2, double y2);
double angle(double,double,double,double,double,double,double,double,double ,double );
void maestro(int );
void init_maestro(void);
int maestroGetPosition(int , unsigned char );
int maestroSetTarget(int , unsigned char , unsigned short );
int maestroSetSpeed(int , unsigned char , unsigned short );
//global variables
int counter_filter = 0;
double filter[5]={0,0,0,0,0}, distance_Max;
bool flag_calibration=false;
int Velcount = 0;
double initial_offset=5;
clock_t start;
double duration;


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

int main(int argc, char** argv)
{
	double rotational_velocity=0, distance_Min , pan_angle_acc;
//positions in Camera Farme
	double head_CF[3] , neck_CF[3], left_hand_CF[3], right_hand_CF[3], torso_CF[3];
//positions in Image Frame (Y,X)
	double head_IM[2] , neck_IM[2], left_hand_IM[2], right_hand_IM[2], torso_IM[2];
	double rotational_velocity_max = 10;
	double rotation_range_max = 75;
	double translational_velocity_max=0.1, translational_velocity , gradient , decreasing_velocity , real_distance, gained_error , error;
	int pan_angle;
	float sonar_distance[8], sonar_min;
	std_msgs::Float32MultiArray msgs_;
	float msgs[6];

	CvMat* object_points	= cvCreateMat( 3, 1, CV_64FC1 );
	CvMat* rotation_vector	= cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* translation_vector = cvCreateMat( 3, 1, CV_32FC1 );
	CvMat* intrinsic_matrix	= cvCreateMat( 3, 3, CV_64FC1 );
	CvMat* distortion_coeff	= cvCreateMat( 5, 1, CV_32FC1 );
	CvMat* xy	= cvCreateMat( 2, 1, CV_64FC1 );
  
	Sonar sensor;

	ros::init(argc, argv, "set_rotation");
	ros::NodeHandle node;

	tf::TransformListener listener;
//	ros::Publisher robotrotationvel = node.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
	ros::Publisher robotrotationvel = node.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
	ros::Publisher image_    = node.advertise<std_msgs::Float32MultiArray>("frames", 1);
	ros::Subscriber sub = node.subscribe("/RosAria/sonar", 1, &Sonar::sonar, &sensor);

	init_maestro();

	ros::Rate rate(10.0);
	while (node.ok()){
printf("check 1");
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
			ROS_ERROR("%s",ex.what());
			}


//calibration projection
//camera calibration matrix
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
//set transforms to matrix
		head_CF[0]=-1 * transform_head.getOrigin().z();
		head_CF[1]=transform_head.getOrigin().y();
		head_CF[2]=transform_head.getOrigin().x();
		cvSetReal2D(object_points, 0, 0, head_CF[1]);
		cvSetReal2D(object_points, 1, 0, head_CF[2]);
		cvSetReal2D(object_points, 2, 0, head_CF[0]);
		cvProjectPoints2(object_points,rotation_vector,translation_vector,intrinsic_matrix,distortion_coeff,xy);
//positions in the image frame
		head_IM[0]=cvGetReal2D(xy, 0, 0);
		head_IM[1]=cvGetReal2D(xy, 1, 0);

		torso_CF[0]=-1 *transform_torso.getOrigin().z();
		torso_CF[1]=transform_torso.getOrigin().y();
		torso_CF[2]=transform_torso.getOrigin().x();
		cvSetReal2D(object_points, 0, 0, torso_CF[1]);
		cvSetReal2D(object_points, 1, 0, torso_CF[2]);
		cvSetReal2D(object_points, 2, 0, torso_CF[0]);
		cvProjectPoints2(object_points,rotation_vector,translation_vector,intrinsic_matrix,distortion_coeff,xy);
//positions in the image frame
		torso_IM[0]=cvGetReal2D(xy, 0, 0);
		torso_IM[1]=cvGetReal2D(xy, 1, 0);

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
//publish the frames
		msgs[0] = right_hand_IM[1];
		msgs[1] = right_hand_IM[0];
		msgs[2] = left_hand_IM[1];
		msgs[3] = left_hand_IM[0];
		msgs[4] = torso_IM[1];
		msgs[5] = torso_IM[0];
		msgs_.data.assign(msgs,msgs+6);
		image_.publish(msgs_);


//rotational velocity: angle/s 
		rotational_velocity = angle(head_IM[1],head_IM[0],left_hand_IM[1],left_hand_IM[0],right_hand_IM[1],right_hand_IM[0],torso_IM[1],torso_IM[0],rotational_velocity_max,rotation_range_max);
		printf("Angle computed  is %f", rotational_velocity);
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
		sonar_min = *min_element(sonar_distance,sonar_distance+8);
//		printf("--------------------sonar min %f\n",sonar_min);
//		printf("sonar distance %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",sonar_distance[0],sonar_distance[1],sonar_distance[2],sonar_distance[3],sonar_distance[4],sonar_distance[5],sonar_distance[6],sonar_distance[7],sonar_distance[8],sonar_distance[9],sonar_distance[10],sonar_distance[11],sonar_distance[12],sonar_distance[13],sonar_distance[14],sonar_distance[15]);
//		printf("sonar X %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",sensor.sonar_x[0],sensor.sonar_x[1],sensor.sonar_x[2],sensor.sonar_x[3],sensor.sonar_x[4],sensor.sonar_x[5],sensor.sonar_x[6],sensor.sonar_x[7],sensor.sonar_x[8],sensor.sonar_x[9],sensor.sonar_x[10],sensor.sonar_x[11],sensor.sonar_x[12],sensor.sonar_x[13],sensor.sonar_x[14],sensor.sonar_x[15]);
//		printf("sonar Y  %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",sensor.sonar_y[0],sensor.sonar_y[1],sensor.sonar_y[2],sensor.sonar_y[3],sensor.sonar_y[4],sensor.sonar_y[5],sensor.sonar_y[6],sensor.sonar_y[7],sensor.sonar_y[8],sensor.sonar_y[9],sensor.sonar_y[10],sensor.sonar_y[11],sensor.sonar_y[12],sensor.sonar_y[13],sensor.sonar_y[14],sensor.sonar_y[15]);

//translational velocity: m/s
//		turtlesim::Velocity vel;
		geometry_msgs::Twist vel;
		vel.angular.x = 0;
		vel.angular.y = 0;
		vel.linear.y = 0;
		vel.linear.z = 0;

		if(listener.canTransform("/head_1", "/openni_depth_frame", ros::Time(0), NULL)==true && flag_calibration==false) 
		{
//printf("check 2");
			
			distance_Max=((head_CF[0]+neck_CF[0]+torso_CF[0])/3);
                        distance_Min = 0.7 * distance_Max;
			if(distance_Max!=0 && distance_Min!=0)
			{
				flag_calibration = true;
			}			
		}
		
////distance_Max = 1.5;
////distance_Min = 1;			
		gradient =  (translational_velocity_max)/(distance_Min-distance_Max);
		printf("gradient %f \n",gradient);
		real_distance = ((head_CF[0]+neck_CF[0]+torso_CF[0])/3);
		float intercept = translational_velocity_max - (gradient*distance_Min);
		printf("intercept %f",intercept);
		printf("Distance Max %f \n",distance_Max);
		printf("Distance Min %f \n",distance_Min);
		printf("Real distance %f",real_distance);
		if(real_distance<distance_Max && real_distance>distance_Min)
		{
//			translational_velocity = fabs(gradient*real_distance + (gradient*distance_Max));
			translational_velocity = gradient*real_distance + intercept;
			if(translational_velocity > translational_velocity_max){ translational_velocity = translational_velocity_max; }
			if(translational_velocity<0){translational_velocity = 0;}
			printf("velocity pos 1 %f \n", translational_velocity);
		}
		else if (real_distance>distance_Max)  
		{
			rotational_velocity=0;
			translational_velocity=0;
			printf("velocity pos 2 %f \n", rotational_velocity);
		}
		else if (real_distance<distance_Min) 
		{
			rotational_velocity=0;
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
		if(listener.canTransform("/head_1", "/openni_depth_frame", ros::Time(0), NULL)==false) 
		{	
//printf("check 3");			
			rotational_velocity=0;
			translational_velocity=0;
			printf("velocity pos 4 %f \n", rotational_velocity);
		}


		vel.angular.z = -1 * rotational_velocity*PI/180;
//		vel.angular.z=0;
		vel.linear.x=translational_velocity;
		printf("rot Velocity set to the robot  is %f", rotational_velocity);
		printf("*********************************************************");
//		vel.angular=0;
//		vel.linear=translational_velocity;
//apply safety
		/*if (sonar_min<80) 
		{
			vel.angular.z=0;
			vel.linear.x=0;
//			vel.angular = 0;
//			vel.linear = 0;
double velocity_set = 0;
			printf("velocity %f", velocity_set);
		}*/

		robotrotationvel.publish(vel);

//gaze control (visual servoing)
		if(listener.canTransform("/head_1", "/openni_depth_frame", ros::Time(0), NULL)==true) 
		{

//center of the camera frame: 234.300807, Y 317.3  (Pixels)
			if (right_hand_IM[1]>450 || right_hand_IM[1]<257)
			{
				if(right_hand_IM[1]<257)
				{
					error = right_hand_IM[1]-257;
				}
				else if(right_hand_IM[1]>450)
				{
					error = right_hand_IM[1]-450;
				}
				else
				{
					error = 0;
				}
//landa coeff in the Image Jacobian matrix= 0.1
				gained_error = error * 0.1;
				modf(gained_error, &pan_angle_acc);
				pan_angle = pan_angle_acc;
				if(listener.canTransform("/neck_1", "/openni_depth_frame", ros::Time(0), NULL)==false) pan_angle = 0;
				maestro(pan_angle);
			}
			else maestro(0);
		}
		ros::spinOnce();
		rate.sleep();
//empty the buffer    
		transform_head.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform_left_hand.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform_right_hand.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform_torso.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
		transform_neck.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
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
//ranging the angle
	if (temp>max_range) rotation_angle=max_range;
	else if (temp<-max_range) rotation_angle=-max_range;
	else if (temp>-initial_offset && temp<initial_offset) rotation_angle=0;
	else rotation_angle = temp;
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
  if (degree>38) degree = 38;
  if (degree<-18) degree = -18;
  int target = ( degree*45 ) +  6000;
  maestroSetSpeed(fd, 0, 10);
  maestroSetTarget(fd, 0, target);
  close(fd);
}
void init_maestro(void)
{
  const char * device = "/dev/ttyACM0";
  int fd = open(device, O_RDWR | O_NOCTTY);
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
 
  maestroSetSpeed(fd, 0, 10);
  maestroSetTarget(fd, 0,    6900); 
  close(fd);
}
