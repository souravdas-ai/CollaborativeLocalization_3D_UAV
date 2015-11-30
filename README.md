Collaborative Drone Localization
==============================
3D localization system in 6 DoF based on 2 onboard cameras and a minimal set of markers in a outdoor environment for the [Parrot Ar-Drones 2.0]. The localization algorithm uses this paper : [6DoF Cooperative Localization for Mutually
Observing Robots](http://www2.ift.ulaval.ca/~pgiguere/papers/isrr2013.pdf). The code is inspired by [RPG Monocular Pose Estimator].

#Installation#
##Material required##
This package require:

 * Two [Parrot Ar-Drones 2.0] with visual marker on either side of camera.
 * An unprotected wifi router that does not require an AC power source.
 * Any Linux compatible gamepad joystick. We use the [F710 Wireless Gamepad] from Logitech. We suggest [remapping] the buttons to the desire configuration that suit your own gamepad.

##Dependencies##
This package require Robotic Operating System (ROS). In order to install ROS follow these instructions on the [ROS website]. This branch of the project was tested on ROS Indigo with Ubuntu 14.04 LTS. 

The Collaborative Drone Localization also require OpenCV and Eigen. They can be installed by following the instruction on the [OpenCV] and [Eigen] website.

Some of the code in this project uses the glut library. To compile, ensure the freeglut3-dev package is installed. It can be installed with the command:
```shell
sudo apt-get install freeglut3-dev
```
To manualy control the drone with a gamepad we use `joy/joy_node`. It can be installed and configure following [these instructions](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).

The [ardrone_autonomy] node is used as a bridge between the Parrot AR-Drone 2.0 SDK and the ROS environment. It can be install by entering the following command. Assuming you are in the src directory of your catkin [workspace].
```shell
cd ~/catkin_ws/src
git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel
cd ~/catkin_ws
catkin_make
```

The [tum_ardrone] package is used for keyboard teleoperation and PID controlled autopilot of the drones. Some minor changes were made to its source code, so our version of it is included in this package.

##Multiple drone setup##

To get your two drones so they connect to a router, follow these instructions on the `ardrone_autonomy` [wiki page](https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones).


##Main installation##

In order to install our version of Collaborative Drone Localization, clone the latest version of our GitHub repository:
```shell
cd catkin_ws/src
git clone https://bitbucket.org/yiannisr/drone3dcl.git
cd ..
catkin_make
```

#Launching demo#

The demo launch file launches two instances of `dot_finder` and an instance of `particle_filter`. You need to manually download (http://www.mediafire.com/download/5gz43oigpd3jdod/demo.bag) and play the demo's rosbag using the following commands:
```Shell
rosbag play -l -d 1 demo.bag
```
In another terminal launch the demo's launch file:
```Shell
roslaunch drone_nav demo.launch 
```
In order to watch the demo, run [rqt](http://wiki.ros.org/rqt) and [rviz](http://wiki.ros.org/rviz) the correct perspective for both software are included at the root of this repository. To open the correct perspective in rqt, go to Perspectives->Import... and import "rqt_vision.perspective" at the root of this repository. To open the correct rviz configuration, in rviz go to File->Open Config and open "rviz_config.rviz". 

#Node diagram#

![nodes diagram](http://i.imgur.com/kL3yGYl.jpg)

The above diagram shows the interactions between nodes in the Collaborative Drone Localization. In order to do autonomous flight you need to run multiple instances `ardrone_autonomy`, `dot_finder` and `drone_gui` for the leader and follower. To prevent unwanted interaction between those instances, the leader's and follower's nodes must run in their own [ROS namespace](http://wiki.ros.org/Names). 

##tum_ardrone##
The `tum_ardrone` nodes are used as controllers for the drones.

Below is a sample launch file for the leader (aka king).
It launches only a GUI to allow keyboard teleoperation of the drone. (See [drone_gui]  for key assignments)
```xml
<launch>
  <group ns="king">
    <node name="drone_gui" pkg="tum_ardrone" type="drone_gui"/>
  </group>
</launch>
```
Below is a sample launch file for the follower (aka mamba). It can be autopiloted by loading the 'CL_mamba.txt' file in the GUI then sending those commands to the follower after takeoff.
```xml
<launch>
  <group ns="mamba">
    <node name="drone_autopilot" pkg="tum_ardrone" type="drone_autopilot"/>
    <node name="drone_gui" pkg="tum_ardrone" type="drone_gui"/>
  </group>
</launch>
```

##dot_finder##
The `dot_finder` node extracts the marker positions from the drone's camera using the red channel from RGB color space and blob analysis.

#### How to launch ####
To run in command line without launch file dot_finder enter the following command:
```shell
rosrun dot_finder dot_finder _topic:="/mamba"
```
This command will run a `dot_finder` instance that subscribes to a drone in the namespace "mamba". The same action can be done with the following [launch file](http://wiki.ros.org/roslaunch/XML):

```xml
<launch>
        <group ns="mamba">
		<node name="dot_finder" pkg="dot_finder" type="dot_finder" output="screen">
			<param name="topic" value="/mamba" />
		</node>
        </group>
</launch>
```

##### Static parameters settings #####
The `dot_finder` node require some parameters to be set before launching
 * topic (string)

Name of the drone's namespace. Every topic subscription/publication will replace (input namespace) by the content of this parameter.

##### Subscribed Topics #####
`dot_finder` subscribes to the following topics:
 * (input namespace)/ardrone/image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    
The image from the drone. The markers will be detected from this image.

 * (input namespace)/ardrone/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

The camera calibration parameters. It's mainly used to compensate the camera distortion.
  
##### Published Topics #####
`dot_finder` publishes to the following topics:
 * (input namespace)/dots (dot_finder/DuoDot)

The x, y position of every potential marker extracted from the image.
 * (input namespace)/ardrone/image_with_detections ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
   
A debugging visualization image of the computer vision.

##particle_filter##
The `particle_filter` node takes the marker hypothesis of the two cameras, finds the correct hypothesis and compute the pose in 6DoF.

Note: Some of the calculations performed require a camera calibration file for the drones.
	  Make sure the calibration file ~/.ros/camera_info/ardrone_front.yaml is present.
	  If it isn't, the resulting pose will be 'nan' (not a number).

#### How to launch ####
To run in command line without launch file dot_finder enter the following command:
```Shell
rosrun particle_filter particle_filter _leader:="/king" _follower:="/mamba"
```
This command will run a `particle_filter` instance that subscribes to two dot_finders nodes, one in the namespace "king" which is the leader, the other in the namespace "mamba" which is the follower. The same action can be done with the following [launch file](http://wiki.ros.org/roslaunch/XML):

```xml
<launch>
	<node name="particle_filter" pkg="particle_filter" type="particle_filter" output="screen">
		<param name="leader" value="/king" />
		<param name="follower" value="/mamba" />
	</node>
</launch>
```

##### Static parameters settings #####
The `particle_filter` node require some parameters to be set before launching
 * leader (string)

Name of the leader's namespace. Every topic subscription/publication will replace <leader namespace> by the content of this parameter.
 * follower (string)

Name of the follower's namespace. Every topic subscription/publication will replace <follower namespace> by the content of this parameter.

##### Dynamic parameters settings #####
The following parameters can be set dynamically during runtime. (Use `rqt_reconfigure` to adjust the parameters).

* pos_XXXX_led_cam_x (double, default: 0.19, min: 0, max: 2)

These parameters set the distance between the marker and the camera on the drone in meter.
*These values are always positive.*

##### Subscribed Topics #####
`dot_finder` subscribes to the following topics:
 * (leader and follower namespace)/dots (dot_finder/DuoDot)
    
The x, y positions of every marker hypothesis extracted from the image.

 * (leader and follower namespace)/ardrone/imu ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))

Imu information from the drone.

* (leader and follower namespace)/ardrone/image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    
The image from the front camera of the drone. Only use for visualization/debugging.

##### Published Topics #####
`dot_finder` publishes to the following topics:
 * (follower namespace)/ardrone/predictedPose ([tum_ardrone/filter_state])

Relative pose in 6DoF of the follower.

 * (follower namespace)/pose_array_candidates ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))

Every hypothesis of the position of the follower.

 * (follower namespace)/pose ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

Relative pose in 6DoF of the follower.

 * (leader and follower namespace)/ardrone/image_ROI ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
   
A debugging visualization image of Region of Interest.



[Parrot Ar-Drones 2.0]:http://ardrone2.parrot.com/
[F710 Wireless Gamepad]:http://gaming.logitech.com/en-ca/product/f710-wireless-gamepad
[remapping]:http://wiki.ros.org/joystick_remapper/Tutorials/UsingJoystickRemapper
[RPG Monocular Pose Estimator]:https://github.com/uzh-rpg/rpg_monocular_pose_estimator
[ROS website]:http://wiki.ros.org/indigo/Installation 
[OpenCV]:http://opencv.org/documentation.html
[Eigen]:http://eigen.tuxfamily.org/index.php?title=Main_Page
[workspace]:http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[ardrone_autonomy]:https://github.com/AutonomyLab/ardrone_autonomy
[tum_ardrone]:https://github.com/tum-vision/tum_ardrone
[drone_gui]:https://github.com/tum-vision/tum_ardrone/#drone_gui

