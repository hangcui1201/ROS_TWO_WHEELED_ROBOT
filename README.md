## ROS Differential-drive Robot in Gazebo and Rviz

### ROS version: Kinetic (Ubuntu 16.04)

#### Obstacle Avoidance
$ roslaunch m2wr_description spawn.launch y:=2  
$ rosrun m2wr_motion_plan obstacle_avoidance.py  

#### Go to Goal
$ roslaunch m2wr_description spawn.launch y:=2  
$ rosrun m2wr_motion_plan go_to_goal.py  
$ rosservice call /go_to_goal_switch "data: true"  
$ rosparam set /goal '[3, 1]'  
$ rosservice call /go_to_goal_switch "data: false"  

#### Right Wall Following
$ roslaunch m2wr_description spawn.launch y:=2  
$ rosrun m2wr_motion_plan right_wall_following.py  
$ rosservice call /right_wall_following_switch "data: true"  

<a href="url"><img src="./images/right_wall_following.gif" width="500"></a>  

$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py  
$ rosrun m2wr_motion_plan reading_laserscan.py  
$ roslaunch m2wr_description rviz.launch  
