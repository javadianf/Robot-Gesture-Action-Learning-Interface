#!/bin/sh






echo "camera part"
cd /dev/
ls -l
sudo chmod 777 /dev/raw1394
coriander


chmod +x scripts/global_localization_wrapper.sh

roslaunch HRI_Gesture_Demo omnicam_recording.launch

