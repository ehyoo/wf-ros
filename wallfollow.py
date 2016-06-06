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
		# while not rospy.is_shutdown():
			

	def shutdown(self):
		rospy.loginfo("Stop Turtlebot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

	def callback(self, scanmsg):
		range_data = list(scanmsg.ranges)
		# There are 640 entries. 
		# I'll observe the first 10, last 10, and middle 10.
		rate = rospy.Rate(10)

		def list_filter(list):
			return [x for x in list if x != 'nan' and x < 1] # if it doesnt exist or if it's closer than 3m
		
		low_bound = len(range_data)/2 - 5
		up_bound = low_bound + 10
		centre_sensor_data = list_filter(range_data[low_bound:up_bound])
		
		uup_bound = len(range_data) - 1
		ul_bound = uup_bound - 10
		right_sensor_data = list_filter(range_data[ul_bound:uup_bound])


		self.follow_wall(centre_sensor_data, right_sensor_data, rate)
			
	def move(self, direction, rate):
		move_cmd = Twist()
		if direction == 'forward':
			move_cmd.linear.x = 0.2
		elif direction == 'right':
			move_cmd.angular.z = radians(45)
		elif direction == 'left':
			move_cmd.angular.z = -radians(45)
		elif direction == 'stop':
			move_cmd.linear.x = 0.0
			move_cmd.angular.z = 0.0
		else:
			print('direction not implemented yet')
		self.cmd_vel.publish(move_cmd)
		rate.sleep()

	def follow_wall(self, centre, right, rate):
		if centre:
			if right:
				average_distance = sum(centre)/float(len(centre))
				print "centre recognized- average_distance: " + str(average_distance)
				if average_distance > 0.5:
					print "moving forward"
					self.move('forward', rate)
				else:
					print "stopping briefly"
					self.move('stop', rate)
					print "turning right"
					self.move('right', rate)
			else:		
				print "no right wall found"
				for i in range(2):
					self.move('forward', rate)
				print "stopping briefly"
				self.move('stop', rate)
				print "turning right"
				self.move('right', rate) 
		else:
			print "no centre, moving forward"
			self.move('forward', rate)

	def listener(self):
		subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)
		rospy.spin()

if __name__=="__main__":
	FollowWall()
	# except:
	# 	rospy.loginfo("FollowWall Node terminated")
