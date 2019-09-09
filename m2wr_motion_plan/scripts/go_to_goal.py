#! /usr/bin/env python

import rospy
import numpy as np
from std_srvs.srv import *
from tf import transformations
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

# Publishers
pub = None

active_ = False

# Robot state variables
position_ = Point()
yaw_ = 0

# Finite machine state
state_ = 0

# Parameters
yaw_precision_ = np.pi/90  # +/- 2 allowed
dist_precision_ = 0.3

# Goal location
desired_position_ = Point()
goal_ = [-4, 3]
rospy.set_param('goal', goal_)

# Service callback
def go_to_goal_switch(request):

    global active_
    active_ = request.data

    request = SetBoolResponse()
    request.success = True
    request.message = 'Done!'
    return request


# Odom callback
def odom_callback(msg):

    global position_
    global yaw_
    
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


def change_state(state):

    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(np.fabs(angle) > np.pi):
        angle = angle - (2 * np.pi * angle) / (np.fabs(angle))
    return angle


# state = 0
def fix_yaw(des_pos):

    global pub, yaw_, yaw_precision_, state_

    desired_yaw = np.arctan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    rospy.loginfo("Yaw difference: " + str(err_yaw))
    
    twist_msg = Twist()

    if np.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
    pub.publish(twist_msg)
    
    # state change conditions
    if np.fabs(err_yaw) <= yaw_precision_:
        rospy.loginfo('Yaw difference: [%s]' % err_yaw)
        change_state(1)


# state = 1
def go_straight_ahead(des_pos):

    global pub, yaw_, yaw_precision_, state_

    desired_yaw = np.arctan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = np.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print ('Position difference: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if np.fabs(err_yaw) > yaw_precision_:
        print ('Yaw difference: [%s]' % err_yaw)
        change_state(0)


# state = 2
def done():

	global pub
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg)
	change_state(0)


def main():

    global pub, active_, desired_position_, goal_, state_
    
    rospy.init_node('go_to_goal')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)
    srv = rospy.Service('go_to_goal_switch', SetBool, go_to_goal_switch)
    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if not active_:
			print("Robot is disabled!")
			continue
        else:

        	goal_x = rospy.get_param('goal')[0] # x
        	goal_y = rospy.get_param('goal')[1] # y

        	print("Robot is running, goal: %s, %s" % (goal_x, goal_y))
        	print("Robot state: %s" % state_)

        	desired_position_.x = goal_x
        	desired_position_.y = goal_y
        	desired_position_.z = 0

        	if state_ == 0:
        		fix_yaw(desired_position_)
        	elif state_ == 1:
        		go_straight_ahead(desired_position_)
        	elif state_ == 2:
        		done()
        	else:
        		rospy.logerr('Unknown state!')
        
        rate.sleep()


if __name__ == '__main__':
    main()
