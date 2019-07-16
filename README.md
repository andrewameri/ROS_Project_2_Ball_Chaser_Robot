# Ball chaser robot

This project was done as part of assignments in Udacity Sofware Engineer Nanodegree. In this project, a custom world is created in Gazebo. Morever, a differential drive robot with a lidar and a camera is added to the world. The goal of this project was to write a plugin for this robot to follow a white ball. 

### Prerequisites

This project has been written and tested in Ubuntu 16.04LTS using ROS Kinetic. 

### Installing

1- Clone the project and make sure to place the "my_robot" and "ball_chaser" folders in src folder of your catkin_ws

```
$ git clone https://github.com/andrewameri/ROS_Project_2_Ball_Chaser_Robot.git
```
2- Go back to catkin_ws folder and make the project

```
$ cd ..
$ catkin_make
```

## Running the tests

1- Launch the nodes 
```
$ roslaunch my_robot world.launch
$ roslaunch ball_chaser ball_chaser.launch
```
2- To visualize the robot’s camera images, you can subscribe to camera RGB image topic from RViz. Or you can run the rqt_image_view node:
```
$ rosrun rqt_image_view rqt_image_view 
```
Now place a white ball at different positions in front of the robot and see how the robot is capable of chasing the ball!

## Authors

* **Udacity** - *Initial code snippets * - [Udacity](https://www.udacity.com)
* **Andrew Ameri** - *Finalizing, building, and testing the project* - [andrewameri](https://github.com/andrewameri)



