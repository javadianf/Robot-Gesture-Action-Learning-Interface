#!/bin/sh




echo "installing ROSARIA"
hg clone "https://amor-ros-pkg.googlecode.com/hg" hg

roscd
cd hg/ROSARIA
mkdir src
roscd


echo "installing transform file into ROSARIA the folder FILES should be there in the ros_workspace"
cp ./Files/transform.cpp hg/ROSARIA/src/transform.cpp
cp ./Files/CMakeLists.txt hg/ROSARIA/CMakeLists.txt
echo "making ROSARIA"
cd hg/ROSARIA
rosmake ROSARIA
roscd

echo "installing teleopt_base"
svn co https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/teleop_base teleop_base
rosmake teleop_base
#to move the robot teleop_base should ebe installed



echo "control_toolbox"
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_controllers/branches/pr2_controllers-1.4/control_toolbox control_toolbox
rosmake control_toolbox

echo "install joystick_drivers"
sudo apt-get install ros-electric-joystick-drivers


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
