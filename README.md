# common_camera_application

├── CMakeLists.txt
├
├── launch
├	├
│	├── camera_preview.launch
├	├
│	├── camera_robot1.launch
├	├
│	├── camera_robot2.launch
├	├
│	└── camera_robot.launch
├
├── package.xml
├
├── README.md
├
└── script
	├
	├── camera_preview.py
	├
	├── camera_robot1_preview.py
	├
	└── camera_robot2_preview.py

Created package useful in order for testing the basic usage of camera; previewing.
Applicable for various types of camera -- usb-camera, web-cam, raspberry pi, etc

**Notes**
Require(s):
1. https://github.com/OTL/cv_camera.git

**STEP TO TURN ON THE CAMERA**

1. **Turn ON the camera**

	*command* : roslaunch common_camera_application camera_robot.launch 

2. **Preview-ing**

	*command* : rosrun common_camera_application camera_preview.py

3. **Simplified -- Turn On Camera + Preview** 

	*command* : roslaunch common_camera_application camera_preview.launch

**Extra(s)**

*STEP TO UPDATE INTO GITHUB*

1. **change directory (cd) directly to a package or a stack.**

	*command* : roscd [locationname[/subdir]]

	*example* : roscd common_camera_application

2. **push the repository** -- only if you are invited to the repository

	**warning** you may need to update *./bashrc* with source it:

		alias gs='git status'

		alias gp='git pull'

		NOW=$(date +"%m-%d-%Y-%T")

		alias gu='gp && git add . && git commit -m "Updated on $NOW" && git push origin master'

	*command* : gu

3. **enter the username and password**
