#!/usr/bin/env python

# import of relevant libraries.
import rospy # module for ROS APIs
import math
from geometry_msgs.msg import Twist # message type for velocity command.
from sensor_msgs.msg import LaserScan # message type for laser measurement.
from ast import literal_eval # lib to parse coordinates for me

# Constants.
FREQUENCY = 10 #Hz.
VELOCITY = 1 #m/s
DURATION = 5 #s how long the message should be published.
ROT_VEL = math.pi/6 # Rotational Velocity

class Test:
	"""Class example for a ROS node."""
	def __init__(self, string):
		
		self.polyline = string
		self.x = 0
		self.y = 0
		self.angle = 0


		# 1st. initialization of node. Node = thing that publishes stuff and sends commands
		rospy.init_node("goforward_node")

		# self.subscriber = rospy.Subscriber("base_scan", LaserScan, self.laserscan_callback)
		# Subscriber = recieves data from thing it is subscribed to (in this case the "base_Scan"), whenever it recieves data from the subscriber 
		# it calls the set up callback func which is the laserscan_cb

		self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		# Publisher = sends the data it gets to other parts of robots, the thing specified in "" is what it controls. In this case it controls velocity.

		# Sleep important for allowing time for registration to the ROS master.
		rospy.sleep(2)

		# def laserscan_callback(self, laserscan_msg):
		#"""Callback function."""
		#print laserscan_msg.header.frame_id


	def drive(self, rate):
		x = 0
		while x < len(self.polyline): # For all coordinates

			vel_msg = Twist()
			vel_msg.linear.x = VELOCITY
			
			delta_x = self.polyline[x][0] - self.x # Calculates Changes
			delta_y = self.polyline[x][1] - self.y
	
			distance = math.sqrt((delta_y)*(delta_y) + (delta_x)*(delta_x)) # Gets distance
			
			self.turnToDrive(rate, delta_x, delta_y) # Turns to needed position 
			
			time_needed = distance/VELOCITY # Gets time to travel
			start_time = rospy.get_rostime()
			
			while not rospy.is_shutdown():
				# Check if done.
				if rospy.get_rostime() - start_time >= rospy.Duration(time_needed):
					break

				# Publish message.
				self.publisher.publish(vel_msg) # Sends msg to motor (which is the velocity)

				# Sleep to keep the set frequency.
				rate.sleep()

			
			self.x = self.polyline[x][0]
			self.y = self.polyline[x][1]
			x = x + 1 # Updates y loop and internal values	
			

	def turnToDrive(self, rate, x, y):
		
		result = math.atan2(y, x)  # Gets angle needed to turn to
		
		turn_to = result - self.angle # Calculates where to turn to and updates
		
		self.angle = result
		
		vel_msg = Twist()
		vel_msg.angular.z = ROT_VEL # Makes Twist message

		time_needed = turn_to/ROT_VEL # calculates times
		if time_needed < 0:
			vel_msg.angular.z = -1 * ROT_VEL

		
		time_needed = abs(time_needed)
		start_time = rospy.get_rostime()
			
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

		self.drive(rate)


if __name__ == "__main__":
	# Got from: https://stackoverflow.com/questions/17050222/input-a-list-of-2d-coordinates-in-python-from-user
        try:
            string = [literal_eval(coord) for coord in raw_input("coords: ").split()]
        except ValueError:
            print "Please enter the coordinates in the format mentioned"
            exit()
	
	
	t = Test(string)

	t.main()

