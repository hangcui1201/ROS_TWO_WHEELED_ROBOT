#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *

import numpy as np

srv_client_go_to_goal_ = None
srv_client_wall_follower_ = None

yaw_ = 0
yaw_error_allowed_ = 5 * (np.pi / 180) # 5 degrees
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

state_desc_ = ['Go to point', 'Circumnavigate obstacle', 'Go to closest point']
state_ = 0
# 0 - go to point
# 1 - circumnavigate
# 2 - go to closest point

circumnavigate_starting_point_ = Point()
circumnavigate_closest_point_ = Point()

# Seconds the robot is in a state
count_state_time_ = 0 
count_loop_ = 0


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

    rospy.loginfo("State changed: %s" % state_desc_[state])

    # 0 - go to point
    if state_ == 0:
        resp = srv_client_go_to_goal_(True)
        resp = srv_client_wall_follower_(False)

    # 1 - circumnavigate
    if state_ == 1:
        resp = srv_client_go_to_goal_(False)
        resp = srv_client_wall_follower_(True)

    # 2 - go to closest point
    if state_ == 2:
        resp = srv_client_go_to_goal_(False)
        resp = srv_client_wall_follower_(True)


def calc_dist_points(point1, point2):
    dist = np.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
    return dist


def normalize_angle(angle):
    if(np.fabs(angle) > np.pi):
        angle = angle - (2*np.pi*angle) / (np.fabs(angle))
    return angle


def main():

    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_goal_, srv_client_wall_follower_
    global circumnavigate_closest_point_, circumnavigate_starting_point_
    global count_loop_, count_state_time_
    
    rospy.init_node('bug1_algorithm')
    
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
        
        # 0 - go to point
        if state_ == 0:

            if regions_['front'] > 0.15 and regions_['front'] < 1:
                circumnavigate_closest_point_ = position_
                circumnavigate_starting_point_ = position_
                change_state(1)

        # 1 - circumnavigate
        elif state_ == 1:

            # If current position is closer to the goal than the previous closest_position, assign current position to closest_point
            if calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_, desired_position_):
                circumnavigate_closest_point_ = position_
                # rospy.loginfo("New closest point update: %s " % circumnavigate_closest_point_)
                
            # Compare only after 5 seconds - need some time to get out of starting_point
            # If robot reaches (is close to) starting point
            if count_state_time_ > 5 and \
               calc_dist_points(position_, circumnavigate_starting_point_) < 0.2:
                change_state(2)

        # 2 - go to closest point
        elif state_ == 2:

            # If robot reaches (is close to) closest point
            if calc_dist_points(position_, circumnavigate_closest_point_) < 0.2:
                change_state(0)
                
        count_loop_ = count_loop_ + 1

        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        rate.sleep()

if __name__ == "__main__":
    main()
