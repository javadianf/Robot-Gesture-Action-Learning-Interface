#!/bin/sh




echo "camera part"
cd /dev/
ls -l
sudo chmod 777 /dev/raw1394
#coriander


echo ""
roslaunch HRI_Gesture_Demo gesture_demo_with_localization.launch


#sleep 10s
#rosrun openni_tracker openni_tracker
echo "localization section"
rosservice call global_localization
