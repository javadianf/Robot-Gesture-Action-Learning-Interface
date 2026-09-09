// SPDX-License-Identifier: CC-BY-NC-ND-4.0
// Copyright (c) Javadian. All rights reserved.

/* Jesus Rodriguez
This node subscribes to two topics, 
one to get an image 
and another to get the position of the robot with the time
*/
/* edit
Compatible for using in kinect HRI
Fatemeh Javadian
*////
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <sstream>
#include <string>
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include "iostream"
#include <fstream>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Bool.h"
#include <boost/filesystem.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <iomanip>
#include <sensor_msgs/PointCloud.h>


// --------------Callback to save the image------------------
class ImageListener
{
public:
	ImageListener(): cv_image(NULL) {}
	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			cv_image = bridge_.imgMsgToCv(msg, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}

	}
IplImage *cv_image;
sensor_msgs::CvBridge bridge_;
};


class PosListener
{
public:
PosListener(): px(0), py(0), pz(0), oz(0), ow(0) {}


  void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
		px = msg->pose.pose.position.x;
		py = msg->pose.pose.position.y;
		oz = msg->pose.pose.orientation.z;
		ow = msg->pose.pose.orientation.w;
		//dt = ros::Time::now();
}
float px;
float py;
float pz;
float oz;
float ow;
//ros::Time dt;
};


class VelListener
{
public:
VelListener(): v(0), w(0){}

  void callback(const geometry_msgs::Twist::ConstPtr& msg)

{
		v = msg->linear.x;
		w = msg->angular.z;;
}
float v;
float w;
};

/*
class ButtonsListener
{
public:
// ButtonsListener(): {}

  void callback(const sensor_msgs::Joy::ConstPtr& msg)
{
		buttons = msg->buttons[0];
		//std::cout << "\n\n" << buttons << "\n\n";		
}
int buttons;
};
*/

class Demo
{
public:

  void callback(const std_msgs::Bool::ConstPtr& msg)
{
		start_demo = msg->data;		
}
bool start_demo;
};

class SonarListener
{
public:

  void callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
for(int i(0);i<=15;i++)
{	x[i] = msg->points[i].x;	
	y[i] = msg->points[i].y;
	//std::cout << sonar_points;	
}
}
float x[16];
float y[16];
//geometry_msgs::Point32_<std::allocator<void> > sonar_points;
};

//boost
// ------------------------Main Function-----------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	if (argc != 2 )
	{
		ROS_ERROR("Give the name of the folder you want to create to save the demonstration");
		return 0;
	}
	ros::Rate loop_rate(5);		// Rate to subscribe and take the data from topics
	//cvNamedWindow("view");
	cvStartWindowThread();
	image_transport::ImageTransport it(nh);
	PosListener pos_listener;		// Listener subscribes to the position topic and saves the data and time
	ImageListener image_listener;
	VelListener vel_listener;
//	ButtonsListener buttons_listener;
	SonarListener sonar_listener;
	Demo Start_;

	int number = 1;
	std::string num_in_str;
	std::ostringstream ostr;
	
	//Creating the folder
	std::string folder_name(argv[1]);
	std::string path_name = "${ROS_WORKSPACE}/HRI_Gesture_Demo/" + folder_name + "/";
	boost::filesystem::create_directories(path_name);

	std::string frame_name;

	// Changing the string from the argument to character so it can be used by the output file stream (ofstream)
	std::string file_name("data");
	file_name = path_name + file_name;
	const char *p;
	p = file_name.c_str();
	static std::ofstream out_file(p);

	// The same for the sonar file
	std::string sonar_file_name("sonar");
	sonar_file_name = path_name + sonar_file_name;
	const char *p2;
	p2 = sonar_file_name.c_str();
	static std::ofstream sonar_out_file(p2);

	image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, &ImageListener::imageCallback, &image_listener);
	ros::Subscriber sub3 = nh.subscribe("/amcl_pose", 1, &PosListener::callback, &pos_listener);
	ros::Subscriber sub4 = nh.subscribe("/ARIA/cmd_vel", 1, &VelListener::callback, &vel_listener);
//	ros::Subscriber sub5 = nh.subscribe("/joy", 1, &ButtonsListener::callback, &buttons_listener);
	ros::Subscriber sub6 = nh.subscribe("/ARIA/sonar", 1, &SonarListener::callback, &sonar_listener);
	ros::Subscriber sub7 = nh.subscribe("/control/record", 1, &Demo::callback, &Start_);


	if (not out_file)
		std::perror(p);
	else 
	{
		while (ros::ok())
		{
//			if (buttons_listener.buttons == 1)
			//std::cout << "\n Start_.start_demo \n" << Start_.start_demo << "\n\n";
			//printf("rr");
			if (Start_.start_demo == true)
			//if (1==1)
			{
				//std::cout << "\n" << "X: " << pos_listener.px << " Y: " << pos_listener.py << " time: " << pos_listener.dt ;
				/*out_file << "px: " << pos_listener.px << " py: " << pos_listener.py << "\n"
					<< "oz: " << pos_listener.oz << " ow: " << pos_listener.oz << "\n"
					<< "v: " << vel_listener.v << " w: " << vel_listener.w << "\n"
					<< "time: " << pos_listener.dt << "\n"; */
				out_file << std::setw(20) << pos_listener.px
					 << std::setw(20) << pos_listener.py
					 << std::setw(20) << pos_listener.oz
					 << std::setw(20) << pos_listener.ow 
					 << std::setw(20) << vel_listener.v
					 << std::setw(20) << vel_listener.w
					 << std::setw(20) << ros::Time::now() << "\n";
				
				// for the sonar data
				for (int i(0); i <= 15 ; i++) 
				{
					sonar_out_file << std::setw(15) << sonar_listener.x[i] ;
					//std::cout << sonar_listener.x[i] << " " ;
				}
				sonar_out_file << "\n";
				//std::cout << "\n";
				for (int i(0); i <= 15 ; i++) 
				{
					sonar_out_file << std::setw(15) << sonar_listener.y[i] ;
					//std::cout << sonar_listener.y[i] << " " ;
				}									
				//sonar_out_file << "\n\n";

	
				// for the images
				//ostr << number;
				num_in_str = ostr.str();
				frame_name = path_name + num_in_str + ".jpg";
				//cvShowImage("view", image_listener.cv_image);
				number++;
				ostr.str("");
				//Convert from string to char					
				sonar_out_file << "\n\n";

	
				// for the images
				ostr << number;
				const char *p;
				p=frame_name.c_str();
				// Saving the image
				cvSaveImage(p, image_listener.cv_image);
			}
			ros::spinOnce();
			loop_rate.sleep();	
		}	
	}
	out_file.close();
	sonar_out_file.close();
	//cvDestroyWindow("view");
	return 0;
}
