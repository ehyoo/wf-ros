#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class FollowWall():
	is_wall_found = False
	initial_turn_done = False
	def __init__(self):
		rospy.init_node('FollowWall', anonymous=True)
		rospy.on_shutdown(self.shutdown)

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		self.listener()
		# while not rospy.is_shutdown():
			

	def shutdown(self):
		rospy.loginfo('Stop Turtlebot')
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

	def callback(self, scanmsg):
		range_data = list(scanmsg.ranges)
		# There are 640 entries. 
		# I'll observe the first 10, last 10, and middle 10.
		rate = rospy.Rate(10)

		def list_filter(list):
			return [x for x in list if x != 'nan' and x < 1.0] 
			# if it doesnt exist or if it's closer than 1.0m

		low_bound = len(range_data)/2 - 15
		up_bound = low_bound + 30
		centre_sensor_data = list_filter(range_data[low_bound:up_bound])
		
		right_sensor_data = list_filter(range_data[0:30])

		uup_bound = len(range_data) - 1
		ul_bound = uup_bound - 30
		left_sensor_data = list_filter(range_data[ul_bound:uup_bound])

		self.go(left_sensor_data, centre_sensor_data, right_sensor_data, rate)
			
	def move(self, direction, rate):
		move_cmd = Twist()
		if direction == 'forward':
			move_cmd.linear.x = 0.2
		elif direction == 'right':
			move_cmd.angular.z = -math.radians(10)
		elif direction == 'left':
			move_cmd.angular.z = math.radians(10)
		elif direction == 'stop':
			move_cmd.linear.x = 0.0
			move_cmd.angular.z = 0.0
		elif direction == 'back':
			move_cmd.linear.x = -0.2
		else:
			print('direction not implemented yet')
		self.cmd_vel.publish(move_cmd)
		rate.sleep()

	def approach_wall(self, left_avg, centre_avg, right_avg, rate):
		# goes forward until reaches a certain wall
		if centre_avg == 0 or centre_avg > 0.5:
			print 'Wall not reached yet: continuing to move forward'
			self.move('forward', rate)
		else: 
			print 'Wall is found, adjusting to make turtlebot perpendicular.'
			if abs(left_avg-right_avg) < 0.1:
				# Not perfect but should be good enough
				print 'Adjustments complete. Ready to follow wall'
				print '===================='
				self.move('stop', rate)
				self.is_wall_found = True
				rospy.sleep(3) # stop for a few seconds to give yourself a pat on the back
			else:
				print 'Beginning adjustments...'
				print 'Current measurments:\n left: ' + str(left_avg) + ' right: ' + str(right_avg)
				if left_avg > right_avg:
					self.move('stop', rate)
					self.move('right', rate)
				else:
					self.move('stop', rate)
					self.move('left', rate)

	def init_turn(self, left_avg, centre_avg, right_avg, rate):
		print 'left: ' + str(left_avg) + ' right: ' + str(right_avg)
		if right_avg - left_avg <= 0.50 and right_avg != 0 and left_avg == 0:
			self.move('stop', rate)
			print 'Turned and ready to go.'
			print '===================='
			self.initial_turn_done = True
			rospy.sleep(3) # good boy
		else:
			print 'Turning...'
			self.move('left', rate)

	def follow_wall(self, left_avg, centre_avg, right_avg, rate):
		if right_avg > 0.5: 
			print 'Too far from ideal state: adjusting...'
			self.move('right', rate)
			self.move('forward', rate)
		elif right_avg < 0.3:
			print 'Too close from ideal state: adjusting...'
			self.move('left', rate)
			self.move('forward', rate)
		else:
			# Everything is ok.
			self.move('forward', rate)
		# As the turtlebot moves forward, the right value increases while < 1, then 
		# it goes to 0 thanks to our invariant.


	def go(self, left, centre, right, rate):
		# i'll refactor this later
		def avg(lst):
			return sum(lst)/float(len(lst))

		# there has to be a more elegant way to do this
		centre_average_distance = 0
		if centre:
			centre_average_distance = avg(centre) 
		right_average_distance = 0
		if right:
			right_average_distance = math.ceil(avg(right) * 100.0) / 100.0
		left_average_distance = 0
		if left:	
			left_average_distance = math.ceil(avg(left) * 100.0) / 100.0

		if not self.is_wall_found:
			self.approach_wall(left_average_distance, centre_average_distance, right_average_distance, rate)
		elif not self.initial_turn_done:
			self.init_turn(left_average_distance, centre_average_distance, right_average_distance, rate)
		else:
			self.follow_wall(left_average_distance, centre_average_distance, right_average_distance, rate)

	def listener(self):
		subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)
		rospy.spin()

if __name__=='__main__':
	FollowWall()
	# except:
	# 	rospy.loginfo('FollowWall Node terminated')
