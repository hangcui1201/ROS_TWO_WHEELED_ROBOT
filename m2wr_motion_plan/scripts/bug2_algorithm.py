#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *

import math

import numpy as np

srv_client_go_to_goal_ = None
srv_client_wall_follower_ = None

yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()

initial_position_ = Point()
initial_position_.x = rospy.get_param('init_x')
initial_position_.y = rospy.get_param('init_y')
initial_position_.z = 0

desired_position_ = Point()
desired_position_.x = rospy.get_param('goal_x')
desired_position_.y = rospy.get_param('goal_y')
desired_position_.z = 0

regions_ = None

state_desc_ = ['Go to goal', 'Right wall following']
state_ = 0

count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to goal
# 1 - wall following


# Odom callback
def odom_callback(msg):

    global position_, yaw_
    
    # Position
    position_ = msg.pose.pose.position
    
    # Yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)

    # rpy
    euler = transformations.euler_from_quaternion(quaternion)

    yaw_ = euler[2]


# Laser callback
def laser_callback(msg):

    global regions_

    regions_ = {
        'right': min(min(msg.ranges[0:35]), 5),   # right side of the robot
        'fright': min(min(msg.ranges[36:71]), 5),
        'front': min(min(msg.ranges[72:107]), 5),
        'fleft': min(min(msg.ranges[108:143]), 5),
        'left': min(min(msg.ranges[144:179]), 5)  # left side of the robot
    }



def change_state(state):

    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_goal_
    global count_state_time_

    count_state_time_ = 0
    state_ = state

    rospy.loginfo("state changed: %s" % state_desc_[state])

    if state_ == 0:
        resp = srv_client_go_to_goal_(True)
        resp = srv_client_wall_follower_(False)

    if state_ == 1:
        resp = srv_client_go_to_goal_(False)
        resp = srv_client_wall_follower_(True)


def distance_to_line(p0):

    # p0 is the current position
    # p1 and p2 points define the line

    global initial_position_, desired_position_

    p1 = initial_position_
    p2 = desired_position_

    up_eq = np.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = np.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq
    
    return distance

    
def normalize_angle(angle):
    if(np.fabs(angle) > np.pi):
        angle = angle - (2*np.pi*angle) / (np.fabs(angle))
    return angle


def main():

    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_goal_, srv_client_wall_follower_
    global count_state_time_, count_loop_
    
    rospy.init_node('bug2_algorithm')
    
    sub_laser = rospy.Subscriber('/m2wr/laserscan', LaserScan, laser_callback)
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rospy.wait_for_service('/go_to_goal_switch')
    rospy.wait_for_service('/right_wall_following_switch')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_go_to_goal_ = rospy.ServiceProxy('/go_to_goal_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/right_wall_following_switch', SetBool)
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # Set robot position
    model_state = ModelState()
    model_state.model_name = 'm2wr'
    model_state.pose.position.x = initial_position_.x
    model_state.pose.position.y = initial_position_.y
    resp = srv_client_set_model_state(model_state)
    
    # Initialize going to the point
    change_state(0)
    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if regions_ == None:
            continue
        
        distance_position_to_line = distance_to_line(position_)
        
        # 0 - go to goal
        if state_ == 0:
            if regions_['front'] > 0.15 and regions_['front'] < 1:
                change_state(1)
        
        # 1 - right wall following
        elif state_ == 1:
            if count_state_time_ > 5 and distance_position_to_line < 0.1:
                change_state(0)
                
        count_loop_ = count_loop_ + 1

        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        rospy.loginfo("Distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
       
        rate.sleep()

if __name__ == "__main__":
    main()
