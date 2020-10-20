#!/usr/bin/env python

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for velocity command.
from sensor_msgs.msg import LaserScan # message type for laser measurement.

import tf # library for transformations.

import numpy
import math

# Constants.
FREQUENCY = 10 # Hz.
VELOCITY = 0.2 # m/s
DURATION = 5 # s how long the message should be published.
MIN_DISTANCE = 1 # meter
KP = .2 # Play with these to see how robot acts, KP = better turn KD = better straight
KD = .8

class Test:
	    """Class example for a ROS node."""
	    def __init__(self, goal_distance):
		# 1st. initialization of node.
        rospy.init_node("goforward_node")

		# 2nd. setting up publishers/subscribers.
		self.subscriber = rospy.Subscriber("base_scan", LaserScan, self.laserscan_callback)
		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

		self.listener = tf.TransformListener()

		self.front_distance = 0
		self.right_distance = 0
		self.corner_distance = 0
		self.current_error = 0
		self.prev_error = 0
		self.goal = goal_distance


		# Sleep important for allowing time for registration to the ROS master.
		rospy.sleep(2)


	    def laserscan_callback(self, laserscan_msg):
		"""Callback function."""

		i = int(-laserscan_msg.angle_min/laserscan_msg.angle_increment) # Calculations to get in front and left of the robot 
		self.front_distance = laserscan_msg.ranges[i]

		z = 0
		vec_arr = []
		while z < i:
			 vec_arr.append(laserscan_msg.ranges[z])
			 z = z + 1
		vec_arr.append(laserscan_msg.ranges[i])
		self.right_distance = min(vec_arr)

		self.get_error()
	     

	    def get_error(self): # Gets error from least distance to wall and the error
			self.prev_error = self.current_error
			self.current_error = self.goal - self.right_distance
			print(self.current_error)


	    def calculate_rot_vel(self):
			if (self.current_error != 0 and self.prev_error != 0):
				delta_error = (self.current_error - self.prev_error)/(1.0/FREQUENCY)
				rot_vel = ((KP * self.current_error) + (KD * delta_error)) * 2 * math.pi
				return rot_vel
			else:
				rot_vel = 0
				return rot_vel


	    def main(self):
		"""main function."""
		
		# 3rd. actual code.
		# Rate at which to operate the while loop.
		rate = rospy.Rate(FREQUENCY)

		# Velocity message
		vel_msg = Twist()

		while not rospy.is_shutdown():
				if self.front_distance > 1: # If nothing in front, give forward velocity
					vel_msg.angular.z = self.calculate_rot_vel()
					vel_msg.linear.x = VELOCITY
				else:
					vel_msg.angular.z = 3  # If something in front, make turn right hard             
		
		    # Publish message.
				self.publisher.publish(vel_msg)

		    
		    # Check if done.
		    #if rospy.get_rostime() - start_time >= rospy.Duration(DURATION):
		    #    break
		    
		    # Sleep to keep the set frequency.
				rate.sleep()


if __name__ == "__main__":
	t = Test(.5)
    
	t.main()
