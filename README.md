# Eddiebot
### In this document, we explore Eddiebot setup, 2D and 3D map generation, and the utilization of the navigation stack for guiding the robot within the map using ROS2 and Gazebo.

## Exercise 0: Getting to know Eddie
Please watch the introductory videos to Eddie that are available on Moodle.
* 01-Eddie-hardware-and-assembly
* 02-Eddie-command-set
* 03-Eddie-ROS2-Packages
* 04-Eddie-code-review
* 05-Eddie-hazards

Please keep in mind that you can make changes to the code in `eddiebot-ros` packages as you need.

## Exercise 1: Setting up the Kinect
Clone the [kinect_ros2](https://github.com/fadlio/kinect_ros2) in your repository. Follow the instructions to build the package and its dependencies.
You need to compile and install [libfreenect](https://github.com/OpenKinect/libfreenect) from its GitHub repository before building kinect_ros2. Read the instruction carefully. You need to install the required dependencies (both for the main libraries and examples) through `apt`. `freeglut3-dev` for example is a required library for examples.
Make sure you set the following `cmake` flags along with the other flags that you may nee ROS_DOMAIN_ID](https://www.theconstructsim.com/separating-ros2-environments-ros_domain_id-ros2-concepts-in-practice/)
After setting up the proper networking, try teleoperating Eddie using the two laptops. Bring up rviz on the remote laptop, try displaying the RGB image of the kinect.

**Answer**

We bring up depth image and RGB image data of `Microsoft Kinect xbox-360` using `Kinect_ros2` package with IPC support, based on Libfreenect. Libfreenect is a user-space driver for Microsoft Kinect. Using `freenect-glview` command, RGB/depth image can be seen.
We also use the `Kinect_ros2` executable node to publish the require data for other packages, the node has been spun in nav package of eddiebot, it can also execute and spin manually using `ros2 run kinect_ros2 kinect_ros2_node` command.
Here’s the output of RGB and depth image of Microsoft Kinect in rviz:
![](https://i.ibb.co/rMcZshH/3-1.png)

## Exercise 2: Add timestamp to RGB images
While kinect_ros2_node is running, open rqt and go to the topic monitor. Check the timestamps for image_raw and depth/image_raw topics. Do both of them have valid timestamps? Browse through the code of kinect_ros2 and find the place where it assigns a timestamp to depth images. Make sure that RGB images also get a timestamp. Open rqt and check to see if your changes have worked!

**Answer**

As shown below, when using rqt and listening to the related topics, the `/image_raw` does not have a valid timestamp.
![](https://i.ibb.co/XJKQJxf/3-2.png)
The timestamp of `/depth/image_raw` has been set in `Kinect_ros2_component.cpp` code, the missing timestamp of `/image_raw` has been added similarly to the available timestamp. As shown below the rgb_info topic header.stamp also has been set to the timestamp (it was needed for rtabmap Kinect in exercise 7).
![](https://i.ibb.co/6DyrFbv/3-3.png)
The result of the changes of the code can be seen below that the `/image_raw` topic has a timestamp in its header part.
![](https://i.ibb.co/Zf70qnP/3-4.png)


## Exercise 3: bringup the Robot
Pull in the latest changes from the eddiebot-ros repository. Make sure all the dependencies are installed using rosdep and then build all the packages.
Connect the USB port from Eddie to the laptop that is going to stay on the robot. If you’re using distrobox run the following command from outside the box: (change ttyUSB0 accordingly)
``` 
sudo chmod a+rw /dev/ttyUSB0
```
run the following launch file to bring up the robot:
```
ros2 launch eddiebot_bringup eddie.launch.yaml
```
Check out the output and make sure you don’t see any errors.
On a separate terminal run:
```
ros2 launch eddiebot_nav eddiebot.launch.py
```
This should start the nodes for Kinect, odometry, vel_controller, some static transforms, and fake laser scan.
Spin up another terminal and run:
``` 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/eddie/cmd_vel
```
You should see the instructions for how to use teleop. Reduce both linear and angular velocities to somewhere near 0.2. Now start running the robot using the keys. Try all the different ways you can control it using teleop.
Launch the view_model file from eddiebot rviz; make sure to also launch the robot description with it (by setting the proper arguments).
* Start proper static transforms until all the errors are gone.
* Add the PointCloud display in rviz and choose the proper topic. (you may need to change the QoS setting to BestEffort for example. Keep this in mind for the rest of the exercises).
Try to move around infront of the kinect and experiment with the cloud.
* Turn off the display for point cloud. Include a display for LaserScan and visualize the scan line. Experiment with the scan line, its FOV, and range.

Make sure you read all the launch files carefully. Questions will be asked regarding them during the interview.

**Answer**

`eddiebot_bringup` package launch file `eddie.launch.yaml` spin four distinct nodes for bringing up different data of robot to ros2. The nodes are:
```
1. eddie -> for connecting to eddie board
2. eddie_ping -> for reading distance sensors (infrared and ultrasonic) installed on robot
3. eddie_adc -> for reading battery voltage level
4. eddie_controller -> for interacting with robot velocity related parts
```
eddiebot_nav package launch file `eddiebot.launch.py` does the same as the `eddiebot_bringup` package launch file but for bringing up the data needed for navigating as below:
```
1. eddie_odom node from eddie_odom package -> explained in exercise 6
2. eddie_vel_controller node from eddie_vel_controller package -> published cmd_vel topic data to eddie/cmd_vel topic
3. kinect_ros2_node node from kinect_ros2 package -> explained in exercise 1
4. Does a static transformation to kinect_depth frame in respect of camera_depth_optical_frame
5. Does a static transformation to kinect_rgb frame in respect of camera_rgb_optical_frame
6. depthimage_to_laserscan_node from depthimage_to_laserscan package -> converting the rgbd-
camera data of Microsoft Kinect to be as same as laser scanner data
```
using `teleop_twist_keyboard` executable node in `teleop_twist_keyboard`, eddiebot can be controlled
using keyboard button pushes through `simple_velocity` commands which brought up by the eddiebot to
control its velocity.
Here’s the output of the view_model in rviz with using the argument `desctiption:=True`, it builds the tf
tree of the robot related frames (the transformation between frames and child parent relation between
them)

## Exercise 4: Networking
One laptop is going to stay on Eddie and the other laptop is going to do teleoperation. For this, both laptops need to be on the same network. You can use your phone’s hotspot or use a modem/access point/wireless router and create your own network that way. The nodes that are used for teleoperation are run on the static laptop, and the nodes interfacing with Eddie are going to be run on the laptop that is mounted on the robot. Watch the video for ROS2 Networking in Moodle and breeze through the following links:
* The Robotics Back-End: ROS2 Multiple Machines Tutorial
* Discovery in ROS2
* ROS Domain ID
* The Construct: Separating ROS2 environments – ROS_DOMAIN_ID

After setting up the proper networking, try teleoperating Eddie using the two laptops. Bring up rviz on the remote laptop, try displaying the RGB image of the kinect.

**Answer**

If `ROS_DOMAIN_ID` has not been set, default value is equal to 0, ros2 machines on the same network can communicate via different `ROS_DOMAIN_ID`’s (up-to 128).
In our case by using the `ros2 run teleop_twist_keyboard teleop_twist_keyboard` command eddiebot velocity controller related topics can be controlled by another ros2 machine.

## Exercise 5: 2D SLAM
You are now ready for running your first SLAM. Please review phase 0 if you don't remember what SLAM is and where it is used for. The role of Simultaneous Localization and Mapping is to use different types of sensors, including laser scanners, cameras, encoders, GPS and IMUs, to accurately map environments with dynamic obstacles and changing surroundings. Laser scanners are commonly used for localization and mapping in industrial environments and are considered the most robust sensor. However, not many open-source laser scanner SLAM algorithms can build accurate maps of big spaces and do so in real-time using mobile processors. SLAM Toolbox, a fully open-source ROS2 package, was proposed to solve this problem by providing accurate mapping algorithms, improved graph optimization, and reduced compute time. It also offers kinematic map merging, multi-session mapping, and prototype lifelong and distributed mapping applications. SLAM Toolbox was integrated into ROS 2 Navigation2 project and was selected as the new default SLAM vendor, replacing GMapping, to provide real-time positioning in dynamic environments for autonomous navigation.
In this section, you will perform SLAM and navigation using [slam_toolbox](https://github.com/ SteveMacenski/slam_toolbox.git) and [nav2]() the same way we did in simulation in phase 0. nav2 works with other SLAM methods too, not just slam_toolbox. In the next section, we will try to use nav2 with rtabmap.
You can read up on the following documents to have a more in depth understanding of nav2:
* [(SLAM) Navigating While Mapping](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html#slam-navigating-while-mapping)
* [First-Time Robot Setup Guide](https://navigation.ros.org/setup_guides/index.html)
* [Nav2 Paper](https://arxiv.org/pdf/2107.07589.pdf)

The most important task in this section is tuning the parameters for slam_toolbox using its [configuration](https://github.com/SteveMacenski/slam_toolbox#configuration) and for nav2 using its [configuration](https://navigation.ros.org/configuration/index.html). Their configuration parameters are available in a yaml file in the config directory of eddiebot_nav package. You need to do an in depth research on the internet and GitHub for fine-tuning the parameters for fake laserscans.
* The choice of where to conduct your SLAM is yours. You need to do a SLAM, save the map, and do a navigation just as in phase 0.
* Your location must contain at least two turns.
* You can perform it in a room or in the corridors.
* Make sure you record the parts of the session using a camera (your phone, for example). Both of the group members must be in the video.

**Answer**

By bringing up the `rgbd-image` of Microsoft Kinect data and convert it to fake laser scan data, slam can be done using slam_toolbox package.
After bringing up the required nodes for eddiebot (more explanation provided in previous exercises), robot odometry data calculated and published by eddiebot_nav package which also explained previously. 
using this data and the fake laser scanner, slam_toolbox package can do the localization and mapping simultaneously for eddie robot.
We improved the `slam_toolbox` configuration for making the mapping process more reliable using this link by changing the variance penalty of distance from 0.3 to 0.5 and angular from 0.5 to 1.0, there parameters are related to scan matching in slam_toolbox package.
Using the output map of slam_toolbox mapping (The mapping process also has been recorded and is attached) navigation can be done by `nav2` package.
`nav2` package launch two launch files, first launches the `nav2.yaml` config file for specifying the nav2
parameters then `nav2_bringup` package is launched using the given param files. `nav2_bringup` package spins multiple nodes related to navigating and planning the robot inside and outside the simulator.
For synchronizing the robot velocity and the trajectory of nav2 plan, robot velocity had to subscribe to the `cmd_vel` published topic of nav2 plan trajectory so it needed some changes.
`cmd_vel` topic’s frequency was too frequent for the eddie to execute the given command (mostly drive with speed command detected), so the `nav2.yaml` configuration file needed to be changed; This was implemented by changing the value of smoothing_frequency parameter (we figured the correct value by listening to the `cmd_vel` topic which was 20Hz by default and the set the value to the 0.2 for this part so the `cmd_vel` can be executed on eddie, this was done by experimenting different values).
`scale_velocities` parameter is set to true for getting better performance (it will try to adjust all components
to follow the same direction but still limits acceleration).
Min/Max velocity/acceleration are also decreased due to safety issues.
Xy/yaw_goal_tolerance can be changed to achieve goal state smoothly without being too precise.

## Exercise 6: Analyze Kinematics
In your report, write an in depth analysis of the `eddiebot_odom` packages. Carefully describe the kinematic model of Eddie and explain how `eddiebot_odom` computes odometry from encoder data. (you need to explain all the math involved).
Please also write another explanation for how wheel velocities are computed using the given velocity command for Eddie.

You can consult the following two papers in the docs directory (or other papers):
* [An implementation of ROS Autonomous Navigation on Parallax Eddie platform](https://github.com/arashsm79/eddiebot-ros/blob/main/docs/An%20implementation%20of%20ROS%20Autonomous%20Navigation%0Aon%20Parallax%20Eddie%20platform.pdf)
* [SLAM-BOT-California State University, Sacramento](https://github.com/arashsm79/eddiebot-ros/blob/main/docs/SLAM-BOT-California%20State%20University%2C%20Sacramento.pdf)

Make sure the code for odometry in `eddiebot_odom` is correct.

Try to improve the odometry calculation from wheel encoders, current heading, and instantaneous velocities through the services that the `eddie` node provides. You can also change the code for calculating the speed of wheels in the controller node of `eddiebot_bringup` package for more accurate movement (especially during pure rotation).

Explain how your changes have improved the overall performance.

**Answer**

Eddiebot uses the `edditbot_odom` package to compute its odometry and publish it to the "odom" topic. Commonly, odometry is calculated using wheel encoders, IMU, etc. In our case, with the lack of viable sensors, the package only uses wheel encoders data through subscribing to the "eddie/encoders_data" topic, which is provided by the `eddiebot_bringup` package.

The `Eddiebot_nav` package is used for navigation by creating multiple nodes such as `eddiebot_odom`. It executes the `eddie_odom` executable node from the `eddiebot_odom` package for further computation.

For every callback of the "eddie/encoders_data" topic in `eddiebot_odom` package, the node calculates $x_{\text{abs}}$ (absolute position in the x-axis), $y_{\text{abs}}$ (absolute position in the y-axis), and $\theta_{\text{abs}}$ (absolute rotation in the z-axis). With the parameters defined below in the provided library, $x_{\text{abs}}$, $y_{\text{abs}}$, and $\theta_{\text{abs}}$ can be calculated.

- `WHEEL_RADIUS` which determines the radius of the robot's wheels, set as 0.1524.
- `COUNTS_PER_REVOLUTION` is the encoder count per each wheel revolution, set to 36 as default.
- `WHEEL_BASE` which defines the distance between the centers of the wheels, set to 0.39 as default.
- `DISTANCE_PER_COUNT`, which can be calculated as follows, is the distance each wheel takes to be increased by one:
$$\frac{\pi \cdot 2 \cdot WHEEL\_RADIUS}{COUNTS\_PER\_REVOLUTION}$$

The formula can be explained by using the division of the perimeter of wheels and their revolution counts. By using these parameters, the goal of using odometry for computing navigation can be achieved.

First, the total number of ticks for each wheel needs to be calculated using the difference between the current and previous number of ticks on each wheel:




$$
\Delta_{\text{left\_count}} = \text{current number of ticks} - \text{previous number of ticks of the left encoder} \\
\Delta_{\text{right\_count}} = \text{current number of ticks} - \text{previous number of ticks of the right encoder}
$$

Secondly, the changes of $x$, $y$, and $\theta$ can be monitored and computed using the following formulas:

$$
\Delta_x = (\Delta_{\text{right\_count}} - \Delta_{\text{left\_count}}) . \text{DISTANCE\_PER\_COUNT}
$$
$$
\Delta_y = 0
$$
$$
\Delta_\theta = \frac{\Delta_{\text{right\_count}} + \Delta_{\text{left\_count}}}{\text{WHEEL\_BASE}}
$$

By calculating the difference between the left and right wheel's number of ticks, the whole number of ticks of the robot is computed, and the distance of the robot's movement is obtained by multiplying the `DISTANCE_PER_COUNT` parameter. By dividing by the `WHEEL_BASE` parameter, the angle of rotation of the robot (yaw) is calculated.

$$
\Delta_x = (\Delta_{\text{right\_count}} + \Delta_{\text{left\_count}}) . \text{DISTANCE\_PER\_COUNT} . \cos(\theta)
$$

With the summation of the number of ticks of the right and left wheels, the whole robot's movement in total is computed and converted to meters by multiplying by `DISTANCE_PER_COUNT`. As a result, the movement along the x-axis is calculated by multiplying the cosine of the $\theta$ angle to the whole robot's movement.

$$
\Delta_y = (\Delta_{\text{right\_count}} + \Delta_{\text{left\_count}}) . \text{DISTANCE\_PER\_COUNT} . \sin(\theta)
$$

To calculate the whole movement along the y-axis, the sine is used, and the result formula is shown above.

Now using the formulas above, $x_{\text{abs}}$, $y_{\text{abs}}$, and $\theta_{\text{abs}}$ can be defined as:

$$
x_{\text{abs}} = x_{\text{prev}} + \Delta_x
$$
$$
y_{\text{abs}} = y_{\text{prev}} + \Delta_y
$$



## Exercise 7: Visual SLAM
There are a bunch of VSLAM libraries for mobile robots. But only a handful of them are being actively developed and are compatible with ROS2. You are free to use any one of them:
* [stella_vslam](https://github.com/stella-cv/stella_vslam_ros) (previously known as OpenVSLAM)
* [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) with [this](https://github.com/zang09/ORB_SLAM3_ROS2) ROS2 wrapper
* [isaac_ros_visual_slam](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)

[rtabmap](https://github.com/introlab/rtabmap_ros/tree/humble-devel) is the recommended library for this section. As such, we will provide some instructions and pieces of code for this library. You are to build up on those and perform a successful SLAM along with navigation. RTABMap is a mixed-modality SLAM approach that was released in 2013 and is still being actively maintained and supported. It is a flexible SLAM approach that covers a wide variety of input sensors such as stereo cameras, RGB-D cameras, fisheye cameras, odometry and 2D/3D lidar data. Unlike other methods studied, instead of creating feature maps, RTABMap creates dense 3D and 2D representation of the environment which allows for it to be a drop-in visual SLAM replacement to existing methods without further post-processing. It is already being used in hobby, research, and small-scale service robot applications as a 2D SLAM **and contains all the basic features for utilization in mobile robot applications.**
You can take a look at [this](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf) paper to know more about rtabmap. The `rtabmap` launch file in the `eddiebot_nav` package is an example of how you could launch rtabmap nodes. You need to modify this file so that you could perform SLAM and navigation using rtabmap and nav2 packages. Consult the following links (as stated in the rtabmap's repository, most of the documentations for ROS1 version, such as the parameters for the ros wrapper and rtabmap itself is valid for ROS2):
* [rtabmap wiki.ros](http://wiki.ros.org/rtabmap)
* [rtabmap_slam wiki.ros](http://wiki.ros.org/rtabmap_slam)
* [rtabmap_ros humble branch turtlebot3 example](https://github.com/introlab/rtabmap_ros/tree/humble-devel)
* [rtabmap_ros humble branch demo](https://github.com/introlab/rtabmap_ros/tree/humble-devel/rtabmap_demos/launch)
* [InterbotiX X-Series LoCoBot ROS Packages](https://github.com/stephenadhi/interbotix_ros_rovers/tree/91e06bfcd72e3ca9252015a2f64578fced3cc31c/interbotix_ros_xslocobots/interbotix_xslocobot_nav)
* [rtabmap setup tutorials](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot)This is old and is for ROS1 and should be used only as an example of the overall process.
* [Mapping and Navigation with Turtlebot](http://wiki.ros.org/rtabmap_ros/Tutorials/MappingAndNavigationOnTurtlebot) This is old and is for ROS1 and should be used only as an example of the overall process.
* Do the same tasks as in exercise 4. Perform SLAM and do navigation this time using rtabmap. (the maps don't need to be the same place)

**Answer**

Other methods to perform SLAM (Simultaneous Localization and Mapping) for mobile robots involve using visual SLAM, which relies on data from the robot's camera, including depth images, RGB images, and more. There are multiple packages available to achieve visual SLAM, each with different approaches to feature detection, point clouds in 3D, 2D grid mapping, and so on.

In our case, we utilize the `rtabmap` package, which offers various ways to perform SLAM. The chosen approach is similar to Exercise 5, involving the conversion of the RGB-D image from the Microsoft Kinect sensor to a simulated laser scan and using odometry data (specifically, wheel encoders in our case) to achieve SLAM. This is achieved through the `rtabmap.launch.py` launch file, which launches several other launch files:

- The `rtabmap` node from the `rtabmap_slam` package. If the localization argument is set to false (the default value), the launch file with the desired set of parameters is launched, and the node performs SLAM, saving the map in the `/home/.ros` directory in `.db` format.
- If the localization argument is set to true, the previous map is loaded using the same launch file mentioned above.
- `rtabmap_viz`, a modified RViz-like platform, providing live feature point detection and a unique environment with unique IDs.
- We also include one more package to be launched, the `eddiebot_description` node, which sets up the TF tree of the robot's needed frames (with the description parameter to be used as a condition).
- Another challenge, in addition to the transformation between frames, was dealing with Kinect frames timestamps, which needed to be set in the `kinect_ros2` package.

Challenges specific to `rtabmap` include:

- The Kinect camera's sensitivity to image blurriness, which can be problematic due to the camera's placement on the robot. This issue can be addressed by adjusting parameters such as `RGBD/ProximityPathMaxNeighbors` to approximate between frames. Alternatively, moving the robot in a straight direction and orienting the camera's head downward can help reduce feature point matching problems and excessive shaking.
- During live mapping progress in RViz, the output may lack precision in rotations due to the following reasons:
  - Unreliable odometry data available for the Eddie robot.
  - Difficulty in matching feature points caused by a shaky camera and blurriness.
  As seen in the recordings, the rotations may not align with actual robot movements (e.g., getting a 180-degree rotation in RViz when the robot rotates only 90 degrees). Efforts to address this issue, such as using the `tf-delay` parameter, may not yield desirable results.
- The initial pose is always set by the `rtabmap_slam` package using the Kinect image data and cannot be set manually. Therefore, environmental changes (e.g., lighting conditions, starting pose relative to fixed obstacles) may pose challenges.
- For robot navigation, the same approach as in Exercise 5 is used, listening to the `/cmd_vel` topic while utilizing `rtabmap`. The topic frequency for linear velocity commands is set to 10Hz, while the frequency for pure rotation commands is 6Hz. Although the `smoothing_frequency` parameter is set to 0.1, the results may not be satisfying for rotation.
