# common_camera_application

Common camera (usb-camera, web-cam, raspberry pi, etc) application packages.

**Notes**
Require(s):
1. https://github.com/OTL/cv_camera.git

**STEP TO TURN ON THE CAMERA**


1. run camera package(only turn on the camera)
 command : roslaunch common_camera_application camera_robot.launch 

2. run preview package(to display the preview)
 command : rosrun common_camera_application camera_preview.py

3. jump to the preview package 
 command : roslaunch common_camera_application camera_preview.launch

**STEP TO UPDATE INTO GITHUB**

1. go to the package 
  command : roscd comman_camera_application

2. run the github command
   command : git push

3. enter the username and password



format of rosrun/roslaunch :
roslaunch/rosrun "package" "file"


step to overcome the error :

sb : source ~/.bashrc
eb : gedit ~/.bashrc

if we combine 2 version of python, we need to sb first


