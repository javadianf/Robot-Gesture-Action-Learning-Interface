// SPDX-License-Identifier: CC-BY-NC-ND-4.0
// Copyright (c) Javadian. All rights reserved.

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/package.h>

int main(int argc, char** argv){
  
//this is where we define the name for the node and type of node is name of the execut
  ros::init(argc, argv, "listener_name");

  ros::NodeHandle node;


//this is NOT the name for the node here
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
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


//    ROS_INFO("This is info for position of the head:%f %f %f", transform_head.getOrigin().y(),transform_head.getOrigin().x(),transform_head.getOrigin().z());
//    ROS_INFO("This is info for Oriantation:%f %f %f %f", transform_head.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
//    ROS_INFO("This is info for position of the left hand:%f %f %f", transform_left_hand.getOrigin().y(),transform_left_hand.getOrigin().x(),transform_left_hand.getOrigin().z());
//    ROS_INFO("This is info for position of the right hand:%f %f %f", transform_right_hand.getOrigin().y(),transform_right_hand.getOrigin().x(),transform_right_hand.getOrigin().z());    

//    ROS_INFO("This is info for position of the t:%f %f %f", transform_torso.getOrigin().y(),transform_torso.getOrigin().x(),transform_torso.getOrigin().z());
//    ROS_INFO("This is info for position of the ck:%f %f %f", transform_neck.getOrigin().y(),transform_neck.getOrigin().x(),transform_neck.getOrigin().z());

//printf("This is info for position of the head:%f %f %f \n", transform_head.getOrigin().y(),transform_head.getOrigin().x(),transform_head.getOrigin().z());
//printf("This is info for position of the left hand:%f %f %f \n", transform_left_hand.getOrigin().y(),transform_left_hand.getOrigin().x(),transform_left_hand.getOrigin().z());
//printf("This is info for position of the right hand:%f %f %f \n", transform_right_hand.getOrigin().y(),transform_right_hand.getOrigin().x(),transform_right_hand.getOrigin().z());
//printf("This is info for position of the torso:%f %f %f \n", transform_torso.getOrigin().y(),transform_torso.getOrigin().x(),transform_torso.getOrigin().z());
//printf("This is info for position of the neck:%f %f %f \n", transform_neck.getOrigin().y(),transform_neck.getOrigin().x(),transform_neck.getOrigin().z());  

printf("%f %f %f \n", transform_head.getOrigin().y(),transform_head.getOrigin().x(),transform_head.getOrigin().z());
printf("%f %f %f \n", transform_left_hand.getOrigin().y(),transform_left_hand.getOrigin().x(),transform_left_hand.getOrigin().z());
printf("%f %f %f \n", transform_right_hand.getOrigin().y(),transform_right_hand.getOrigin().x(),transform_right_hand.getOrigin().z());
printf("%f %f %f \n", transform_torso.getOrigin().y(),transform_torso.getOrigin().x(),transform_torso.getOrigin().z());
printf("%f %f %f \n", transform_neck.getOrigin().y(),transform_neck.getOrigin().x(),transform_neck.getOrigin().z());  

    rate.sleep();
  }
  return 0;
};
