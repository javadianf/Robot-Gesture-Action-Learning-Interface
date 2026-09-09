// SPDX-License-Identifier: CC-BY-NC-ND-4.0
// Copyright (c) Javadian. All rights reserved.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float32MultiArray.h"
#include <sstream>
#include <stereo_msgs/DisparityImage.h>
#include <cv.h>
#include <opencv/cv.h>
#include <boost/lexical_cast.hpp>

namespace enc = sensor_msgs::image_encodings;


float right_hand_x, right_hand_y, left_hand_x, left_hand_y, n_x, n_y,head_x , head_y, speeding, angle, swirl;
static const char WINDOW[] = "Demo RST";
class frames_
{
  public:
	void frames_callback(const std_msgs::Float32MultiArray& msg)
		{
			right_hand_x=*(msg.data.data());
			right_hand_y=*(msg.data.data()+1);
			left_hand_x=*(msg.data.data()+2);
			left_hand_y=*(msg.data.data()+3);
			head_x=*(msg.data.data()+4);
			head_y=*(msg.data.data()+5);
			speeding=*(msg.data.data()+6);
			angle=*(msg.data.data()+7);
			swirl=*(msg.data.data()+8);
printf("s %f a %f s %f \n",speeding, angle , swirl);
		}
};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);


    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
      cv::line(cv_ptr->image,cv::Point(180, 0)  , cv::Point(180, 480), CV_RGB(0,0,255),5);
      cv::line(cv_ptr->image,cv::Point(443, 0)  , cv::Point(443, 480), CV_RGB(0,0,255),5);
      //cv::circle(cv_ptr->image, cv::Point(317, 234), 10, CV_RGB(0,255,0),10);
      cv::circle(cv_ptr->image, cv::Point(right_hand_x, right_hand_y), 10, CV_RGB(0,255,0),10);
      cv::circle(cv_ptr->image, cv::Point(left_hand_x, left_hand_y), 10, CV_RGB(0,255,0),10);
      cv::circle(cv_ptr->image, cv::Point(n_x, n_y), 10, CV_RGB(255,0,0),10);
//      printf("%d %d\n",cv_ptr->image.rows,cv_ptr->image.cols);
//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0),10);
    cv::flip(cv_ptr->image , cv_ptr->image , 1);
      cv::rectangle(cv_ptr->image, cv::Point(20, 470), cv::Point(200, 350),CV_RGB(255,0,100), 10);
      //CvFont font;
      //cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA);
     //CvMat matrixTr;
     //CvMat matrix1= cv_ptr->image;
	//cvTranspose(&matrix1,&matrixTr);

	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 3;
	std::string text_speeding = boost::lexical_cast<std::string>( speeding );
	std::string text_angle = boost::lexical_cast<std::string>( angle );
	std::string text_swirl = boost::lexical_cast<std::string>( swirl );
      cv::putText(cv_ptr->image, text_speeding, cv::Point(25, 390), fontFace,fontScale, CV_RGB(255, 255, 255),thickness,5);
      cv::putText(cv_ptr->image, text_angle, cv::Point(25, 420), fontFace,fontScale, CV_RGB(255, 255, 255),thickness,5);
      cv::putText(cv_ptr->image, text_swirl, cv::Point(25, 450), fontFace,fontScale, CV_RGB(255, 255, 255),thickness,5);
     // cv::putText(cv_ptr->image, "12345", cv::Point(25, 390), fontFace,fontScale, CV_RGB(255, 255, 255),thickness,5);
     // cv::putText(cv_ptr->image, "0012345", cv::Point(25, 420), fontFace,fontScale, CV_RGB(255, 255, 255),thickness,5);
    //  cv::putText(cv_ptr->image, "00012345", cv::Point(25, 450), fontFace,fontScale, CV_RGB(255, 255, 255),thickness,5);
     //int baseline;
	//cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
     

    cv::imshow(WINDOW, cv_ptr->image);
  //It handles any windowing events, such as creating windows with cv::namedWindow(), or showing images with cv::imshow()
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  frames_ image_;
  ros::Subscriber sub = n.subscribe("/control/frames", 1000, &frames_::frames_callback, &image_);

  ImageConverter ic;
  ros::spin();

  
  return 0;
}
