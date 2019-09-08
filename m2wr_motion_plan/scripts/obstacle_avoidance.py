#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np


pub = None

def laserscan_callback(msg):

    regions = {
    # 180/5 = 36
        'right': min(min(msg.ranges[0:35]), 5),   # right side of the robot
        'fright': min(min(msg.ranges[36:71]), 5),
        'front': min(min(msg.ranges[72:107]), 5),
        'fleft': min(min(msg.ranges[108:143]), 5),
        'left': min(min(msg.ranges[144:179]), 5)  # left side of the robot
    }

    # regions = {
    # # 180/5 = 36
    #     'right': min(np.average(msg.ranges[0:35]), 5),   # right side of the robot
    #     'fright': min(np.average(msg.ranges[36:71]), 5),
    #     'front': min(np.average(msg.ranges[72:107]), 5),
    #     'fleft': min(np.average(msg.ranges[108:143]), 5),
    #     'left': min(np.average(msg.ranges[144:179]), 5)  # left side of the robot
    # }

    take_action(regions) 
    
def take_action(regions):

    msg = Twist()

    linear_x = 0
    angular_z = 0
    
    state_description = ''

    thres_dist = 2
    
    # Consider obstacle 1m from the robot
    if regions['front'] > thres_dist and regions['fleft'] > thres_dist and regions['fright'] > thres_dist:
        state_description = 'case 1 - nothing'
        linear_x = 0.6
        angular_z = 0
    elif regions['front'] < thres_dist and regions['fleft'] > thres_dist and regions['fright'] > thres_dist:
        state_description = 'case 2 - front'
        linear_x = -0.5
        angular_z = 2
    elif regions['front'] > thres_dist and regions['fleft'] > thres_dist and regions['fright'] < thres_dist:
        state_description = 'case 3 - fright'
        linear_x = -0.5
        angular_z = 2
    elif regions['front'] > thres_dist and regions['fleft'] < thres_dist and regions['fright'] > thres_dist:
        state_description = 'case 4 - fleft'
        linear_x = -0.5
        angular_z = -2
    elif regions['front'] < thres_dist and regions['fleft'] > thres_dist and regions['fright'] < thres_dist:
        state_description = 'case 5 - front and fright'
        linear_x = -0.5
        angular_z = 2
    elif regions['front'] < thres_dist and regions['fleft'] < thres_dist and regions['fright'] > thres_dist:
        state_description = 'case 6 - front and fleft'
        linear_x = -0.5
        angular_z = -2
    elif regions['front'] < thres_dist and regions['fleft'] < thres_dist and regions['fright'] < thres_dist:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = -0.5
        angular_z = 4
    elif regions['front'] > thres_dist and regions['fleft'] < thres_dist and regions['fright'] < thres_dist:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


def main():

    global pub
    
    rospy.init_node('reading_laser')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/m2wr/laserscan', LaserScan, laserscan_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
