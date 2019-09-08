#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def laserscan_callback(msg):

	# Start reading from the right side to the left side
	
	regions = [
	# 180/5 = 36
		min(min(msg.ranges[0:35]), 5),   # right side of the robot
		min(min(msg.ranges[36:71]), 5),
		min(min(msg.ranges[72:107]), 5),
		min(min(msg.ranges[108:143]), 5),
		min(min(msg.ranges[144:179]), 5) # left side of the robot
	]

	rospy.loginfo(regions)

def main():

	rospy.init_node("reading_laserscan")

	sub_laserscan = rospy.Subscriber("/m2wr/laserscan", LaserScan, laserscan_callback)

	rospy.spin()


if __name__ == '__main__':
	main()