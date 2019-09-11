#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False
pub_ = None

state_ = 0

state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}


def right_wall_following_switch(request):

    global active_

    active_ = request.data
    request = SetBoolResponse()
    request.success = True
    request.message = 'Done!'
    return request


def laser_callback(msg):

    global regions_

    regions_ = {
        'right': min(min(msg.ranges[0:35]), 5),   # right side of the robot
        'fright': min(min(msg.ranges[36:71]), 5),
        'front': min(min(msg.ranges[72:107]), 5),
        'fleft': min(min(msg.ranges[108:143]), 5),
        'left': min(min(msg.ranges[144:179]), 5)  # left side of the robot
    }
    
    take_action()


def change_state(state):

    global state_, state_dict_

    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():

    global regions_

    regions = regions_

    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2) # follow the wall
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1) # turn left
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'Unknown case'
        rospy.loginfo(regions)


def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg


def follow_the_wall():    
    msg = Twist()
    msg.linear.x = 0.5
    return msg


def main():

    global pub_, active_
    
    rospy.init_node('right_wall_following')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    sub = rospy.Subscriber('/m2wr/laserscan', LaserScan, laser_callback)
    srv = rospy.Service('right_wall_following_switch', SetBool, right_wall_following_switch)
    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()

        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()