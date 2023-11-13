First assignment of Experimental robotics
================================
**Simone Borelli S4662264** <br>
**Veronica Gavagna S5487110** <br>
**Alessio Mura S4861320** <br>
**Massimo Carlini S4678445** <br>

Requirements
----------------------

The purpose of the first assignment of Experimental robotics is to implement the control of a robot in **ROS** or **ROS 2**, initially in the **Gazebo** environment and subsequently with a real Rosbot, to reach 4 markers with the following IDs: 11, 12, 13, and 15.

The IDs of the markers have specific meanings: 

* Marker 11 instructs the robot to rotate until it locates marker 12, then reaches it; 
* Marker 12 instructs the robot to rotate until it finds marker 13, then reaches it; 
* Marker 13 instructs the robot to rotate until it finds marker 15, then reaches it; 
* Once marker 15 is found, the robot stops as it has completed its tasks.

Notice that “reach marker xxx” means that one side of the xxx marker must be at least 170 pixels in the camera frame.

Implementation
----------------------

This is a possible implementation of the first assignment of Experimental for Robotics course.

The marker detection can be considered complete when the center of the camera aligns with the center of the marker and, before moving on to the next search, the robot's camera must detect at least 170 pixels on one side of a marker. To achieve this, two ROS nodes have been developed: 

* The first is located within the **aruco_ros** package, named **marker_publisher**. This C++ node utilizes the [ArUco](http://wiki.ros.org/aruco) and [OpenCV](http://wiki.ros.org/opencv_apps) libraries to enable the camera for marker ID detection and publishes the first detected ID on the topic **/id_publisher**;
* The second node, written in Python, is part of the **rosbot_bringup** package and is named **realworld**. This node handles the control aspect of the robot to reach the predefined marker.

It is important to underline that, with the above-mentioned implementation, only the robot's control part is managed, not that of the camera. In other words, the camera, being fixed, rotates only when the robot rotates. To implement control of the camera as well, which is an optional task for the assignment, an additional node has been created: 

* The **simulation** python node, which is also part of the **rosbot_bringup** package. Additionally, it handles the rotation of the camera using a specific control topic, **/robot_exp/camera_velocity_controller/command**, and utilizes Euler angles control to align the camera with the robot when the marker is detected.

In addition to this, some modifications have been made to the **rosbot_gazebo.launch**, **rosbot_xacro**, and the addition of a **joint_state_controller** file to include, respectively, the plugins, hardware interface, and PID parameter values to enable camera control.

Installing and running
----------------------
For start the whole program, you have to do some several, but fundamental, step. First of all it is important to have **ROS noetic** version on your pc; the best simple suggestion is to have the [**Docker**](https://docs.docker.com/get-docker/) and then follow this [**ROS guide**](http://wiki.ros.org/ROS/Installation). In addition it is required to install **xterm** terminal; you can do that by using the command on your terminal:

```python
sudo apt-get install xterm
``` 

You can clone our repository, by clicking on the terminal:

```python
https://github.com/VeronicaG24/Assignment1_Exp
```

Make sure to execute the above command within the **src** folder of your workspace. Then execute ```catkin_make``` inside the root of your workspace for building our package. <br>
Once you have done that, you must choose whether you want to launch the simulation with the robot having a fixed camera, meaning it only rotates in conjunction with the robot's movement, or if you want to launch the one with the camera that moves thanks to a continuous joint, i.e., a rotational joint without rotation limits. 

For the first case, launch the following command from the terminal:

```python
roslaunch rosbot_bringup camera_fixed.launch
```

For the second one, use this other command:

```python
roslaunch rosbot_bringup camera_moving.launch
```

In any case, if everything works properly, you should visualize the **Gazebo** environment with the Rosbot and the markers, 2 windows that prints if the marker is reached and the id number, and the **/aruco_marker_publisher_result** which simply shows what the camera detects. 


<table><tr>
  <td> <img src="/rosobot_simulation.png" alt="Drawing" style="width: 850px;"/> </td>
  <td> <img src="./camera_window.png" alt="Drawing" style="width: 350px;"/> </td>
</tr>
<tr>
   <td>Gazebo window</td>
   <td>/aruco_publisher_result window</td>
  </tr>
</table>

Pseudocode 
----------------------

```python
Initialize and clean up ROS node
Create image_feature object
Compute the orientation data for camera and robot alignment
Loop until keyboard interrupt
    Check if marker list is empty
    if empty, shutdown node
    else, continue
    Check if current marker is the first marker in the list
    if yes, then marker found
        Compute error between marker center and camera center
        Compute yaw error between robot and camera
        if yaw error is within threshold or robot is aligned with camera
            if robot aligned and pixel side is greater than limit, marker is reached
                Set linear velocity and angular velocity to zero
                Remove marker from list
            elif, pixel side is less than threshold, robot aligned with marker
                Set linear velocity to forward and angular velocity to zero
            else, robot needs to align with marker
                Set linear velocity to proportional to distance error
                Set angular velocity to proportional to yaw error
        else, robot needs to align with camera
            Set linear velocity to zero
            Set angular velocity to proportional to yaw error
    else, robot is looking for the target marker
        Set linear velocity to zero
        Set angular velocity to positive
Update marker and parameters
Destroy all windows
```

```python
Initialize and clean up ROS node
Create image_feature object
Loop until keyboard interrupt
    Check if marker list is empty
    if empty, shutdown node
    else, continue
    Check if current marker is the first marker in the list
    if yes, then marker found
        Compute error between marker center and camera center
        if error is less than pixel threshold, robot is aligned
            Check if robot yaw is within threshold
            if yes, set linear velocity to forward and angular velocity to zero
            else, rotate robot until yaw is within threshold
        else, if current pixel side is greater than pixel limit, marker is reached
            Set linear velocity and angular velocity to zero
            Remove marker from list
        else, robot needs to align with marker
            Set linear velocity to proportional to error
            Set angular velocity to proportional to error
    else, robot is looking for the target marker
        Set linear velocity to zero
        Set angular velocity to positive
Update marker and parameters
Destroy all windows
```


Simulation videos
----------------------
You can see the videos of both simulations: the first one is related to the fix camera.

 <img source src="/video/camera_fixed.gif" alt="gif showing the behaviour of the Rosbot with fixed camera" width=800>

 
 
 The second one shows the rotation of the camera.

 <img source src="/video/camera_moving.gif" alt="gif showing the bahaviour of the Rosbot with rotating camera" width=800>
 

Real robot video
----------------------


Possible improvements
----------------------
Here are some possible improvements:

* Enhance the efficiency of the robot's marker detection process is to implement an exploration strategy during the initial phase of operation. Currently, the robot relies on rotating in place until it locates the desired marker. However, this approach might be suboptimal, especially when the target marker is not initially within the camera's field of view. To address this limitation, introducing a more sophisticated exploration strategy could significantly improve the robot's ability to discover and reach markers.
* Optimizing the controller responsible for aligning the robot with the center of the marker. The current implementation successfully achieves alignment but may not prioritize generating the most linear trajectory during this alignment process. To address this, a possible improvement is to fine-tune and optimize the controller to guide the robot in a more linear trajectory as it aligns with the marker's center.
