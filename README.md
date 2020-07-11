# common_camera_application

Common camera (usb-camera, web-cam, raspberry pi, etc) application packages.

**Notes**
Require(s):
1. https://github.com/OTL/cv_camera.git

**HOW TO USE**


1. run camera package(only turn on the camera)
 command : roslaunch common_camera_application camera_robot.launch 

2. run preview package(to display the preview)
 command : rosrun common_camera_application camera_preview.py



format of rosrun/roslaunch :
roslaunch/rosrun "package" "file"


step to overcome the error :

sb : source ~/.bashrc
eb :
