# common_camera_application

1. Useful for testing the basic usage of a camera previewing.

2. Applicable for various types of a camera
	1. USB-camera
	2. web-cam
	3. raspberry pi

3. Required Package(s):
	1. https://github.com/OTL/cv_camera.git

		```
		
		$ cd catkin_ws/src
		$ git clone https://github.com/OTL/cv_camera.git
		$ catkin_make

		```

4. Code tree:
```

.
├── CMakeLists.txt
├── launch
│   ├── camera_preview.launch
│   ├── camera_robot1.launch
│   ├── camera_robot2.launch
│   └── camera_robot.launch
├── package.xml
├── README.md
└── script
    ├── camera_preview.py
    ├── camera_robot1_preview.py
    └── camera_robot2_preview.py

```

**STEP TO TURN ON THE CAMERA**

1. **Bringup the camera**

```

$ roslaunch common_camera_application camera_bringup.launch 

```

2. **Preview the bringup camera**

```

$ roslaunch common_camera_application camera_preview.launch

```

3. **Simplified -- Bringup Camera + Preview** 

```

$ roslaunch common_camera_application camera_bringup_preview.launch

```

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