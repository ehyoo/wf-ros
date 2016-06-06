#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import radians

class FollowWall():
	def __init__(self):
		rospy.init_node('FollowWall', anonymous=True)
		rospy.on_shutdown(self.shutdown)

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.listener()
		r = rospy.Rate(10)

		# while not rospy.is_shutdown():
			

	def shutdown(self):
		rospy.loginfo("Stop Turtlebot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

	def callback(self, scanmsg):
		range_data = list(scanmsg.ranges)
		# There are 640 entries. 
		# I'll observe the first 10, last 10, and middle 10.
		def list_filter(list):
			return [x for x in list if x != 'nan']

		left_sensor_data =list_filter(range_data[0:9])
		
		low_bound = len(range_data)/2 - 5
		up_bound = low_bound + 10
		centre_sensor_data = list_filter(range_data[low_bound:up_bound])
		
		uup_bound = len(range_data) - 1
		ul_bound = uup_bound - 10
		right_sensor_data = list_filter(range_data[ul_bound:uup_bound])

		if centre_sensor_data:
			if centre_sensor_data[0] > 0.5:
				self.move('forward')
			else:
				self.move('right')
				self.follow_wall(left_sensor_data, centre_sensor_data, right_sensor_data)
		else:
			self.move_forward()
			
	def move(self, direction):
		move_cmd = Twist()
		if direction == 'forward':
			move_cmd.linear.x = 0.2
		elif direction == 'right':
			move_cmd.angular.z = radians(45)
		elif direction == 'left':
			move_cmd.angular.z = -radians(45)
		else:
			print('direction not implemented yet')
		self.cmd_vel.publish(move_cmd)
		rospy.sleep()

	def follow_wall(left, centre, right):
		if right: 
			if centre:
				average_distance = sum(centre)/float(len(centre))
				if average_distance > 0.5:
					# move forward
				else:
					# turn left
			else:
				# keep moving forward
		else:
			# move foward for like 5 seconds, then turn right 

	def listener(self):
		subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)
		rospy.spin()

if __name__=="__main__":
	FollowWall()
	# except:
	# 	rospy.loginfo("FollowWall Node terminated")
