#!/bin/sh


echo "installing ROSARIA"
hg clone "https://amor-ros-pkg.googlecode.com/hg" ./Packages/ROSARIA

cd ./Packages/ROSARIA/ROSARIA
mkdir src
cd ..
cd ..
cd ..


echo "installing transform file into ROSARIA"
cp ./Files/transform.cpp ./Packages/ROSARIA/ROSARIA/src/transform.cpp
cp ./Files/CMakeLists.txt ./Packages/ROSARIA/ROSARIA/CMakeLists.txt
echo "making ROSARIA"
cd ./Packages/ROSARIA/ROSARIA
rosmake
cd ..
cd ..
cd ..


#echo "installing joystick support"	
#svn co "https://code.ros.org/svn/ros-pkg/stacks/joystick_drivers/trunk/joy" ./Packages/joy
#cd ./Packages/joy
#rosmake
#cd ..
#cd ..


echo "installing joy2Robot"
cd ./Packages/joy2Robot
rosmake
cd ..
cd ..


echo "installing sicktoolbox"
svn co https://code.ros.org/svn/ros-pkg/stacks/laser_drivers/trunk/sicktoolbox ./Packages/sicktoolbox
cd ./Packages/sicktoolbox
rosmake
cd ..
cd ..


echo "installing sicktoolbox_wrapper"
svn co  "https://code.ros.org/svn/ros-pkg/stacks/laser_drivers/trunk/sicktoolbox_wrapper" ./Packages/sicktoolbox_wrapper
cd ./Packages/sicktoolbox_wrapper
rosmake
cd ..
cd ..


#echo "installing navigation stack" #no need to install since available in full ros installation

#echo "installing RVIZ"

#rosdep install rviz
#rosmake rviz
