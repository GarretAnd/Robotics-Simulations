#!/usr/bin/env python

# import of relevant libraries.
import rospy # module for ROS APIs
import math
from geometry_msgs.msg import Twist # message type for velocity command.
from sensor_msgs.msg import LaserScan # message type for laser measurement.

# Constants.
FREQUENCY = 10 #Hz.
VELOCITY = 1 #m/s
DURATION = 5 #s how long the message should be published.
ROT_VEL = math.pi/6 # Rotational Velocity

class Test:
	"""Class example for a ROS node."""
	def __init__(self, radius):
		self.radius = radius


		# 1st. initialization of node. Node = thing that publishes stuff and sends commands
		rospy.init_node("goforward_node")

		# self.subscriber = rospy.Subscriber("base_scan", LaserScan, self.laserscan_callback)
		# Subscriber = recieves data from thing it is subscribed to (in this case the "base_Scan"), whenever it recieves data from the subscriber it calls the set up callback func which is the laserscan_cb

		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		# Publisher = sends the data it gets to other parts of robots, the thing specified in "" is what it controls. In this case it controls velocity.

		# Sleep important for allowing time for registration to the ROS master.
		rospy.sleep(2)

		# def laserscan_callback(self, laserscan_msg):
		#"""Callback function."""
		#print laserscan_msg.header.frame_id

	def half_circle(self, rate):
		vel_msg = Twist()

		arc = math.pi * self.radius
		vel_msg.angular.z = -ROT_VEL
		vel_msg.linear.x = (arc*ROT_VEL)/math.pi
		
		
		start_time = rospy.get_rostime()
			
		while not rospy.is_shutdown():
			# Check if done.
			if rospy.get_rostime() - start_time >= rospy.Duration(math.pi/ROT_VEL):
				break

			# Publish message.
			self.publisher.publish(vel_msg) # Sends msg to motor (which is the velocity)

			# Sleep to keep the set frequency.
			rate.sleep()



	def pointNorth(self, rate):
		# point north
		vel_msg = Twist()
		vel_msg.angular.z = ROT_VEL
		
		time_needed = (math.pi/2)/ROT_VEL
		

		start_time = rospy.get_rostime()
			
		while not rospy.is_shutdown():
			# Check if done.
			if rospy.get_rostime() - start_time >= rospy.Duration(time_needed):
				break

			# Publish message.
			self.publisher.publish(vel_msg) # Sends msg to motor (which is the velocity)

			# Sleep to keep the set frequency.
			rate.sleep()


	def pivot(self, rate):
		# Turn to make the circle
		vel_msg = Twist()
		vel_msg.angular.z = -ROT_VEL
		
		time_needed = (math.pi/2)/ROT_VEL

		start_time = rospy.get_rostime()
			
		while not rospy.is_shutdown():
			# Check if done.
			if rospy.get_rostime() - start_time >= rospy.Duration(time_needed):
				break

			# Publish message.
			self.publisher.publish(vel_msg) # Sends msg to motor (which is the velocity)

			# Sleep to keep the set frequency.
			rate.sleep()


	def straight(self, rate):
		# go forward
		vel_msg = Twist() # Type of message 

		vel_msg.linear.x = VELOCITY # Go forward with velocity of VELOCITY linear.x = forward motion angular.z = turn motion
		
		
		start_time = rospy.get_rostime()

		time_needed = self.radius
		
		while not rospy.is_shutdown():
			# Check if done.
			if rospy.get_rostime() - start_time >= rospy.Duration(time_needed):
				break

			# Publish message.
			self.publisher.publish(vel_msg) # Sends msg to motor (which is the velocity)

			# Sleep to keep the set frequency.
			rate.sleep()



	def main(self):

		"""main function."""
		# 3rd. actual code.

		# Rate at which to operate the while loop.
		rate = rospy.Rate(FREQUENCY)

		self.pointNorth(rate)
		self.straight(rate)
		self.pivot(rate)
		self.half_circle(rate)
		self.pivot(rate)
		self.straight(rate)



if __name__ == "__main__":
	t = Test(2)

	t.main()

