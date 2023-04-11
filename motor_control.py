#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist

class Motor:
	def __init__(self):
                self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
	def set_direction(self, direction):
            #Maximum speed we want to go
            DESIRED_MAXIMUM_LINEAR_X = 0.1 #maximum linear x turtlebot can go is 0.22
            DESIRED_MAXIMUM_ANGULAR_Z = 1 #maximum angular z turtlebot can go is 2.84

            if direction == "stop":
                self.stop()
                return
            linear_x, angular_z = direction
            self.move_with_coor(linear_x, angular_z)

	def turn_left(self):
		self.move_with_coor(linear_x = 0.04, angular_z = 0.07)
	def turn_right(self):
		self.move_with_coor(linear_x = 0.04, angular_z=-0.07)
	def go_forward(self):
		self.move_with_coor(linear_x = 0.07, angular_z= 0)
	def go_backward(self):
		self.move_with_coor(linear_x = -0.1, angular_z=0)
	def stop(self):
		self.move_with_coor(linear_x = 0.0, angular_z = 0.0)
	def move_with_coor(self, linear_x, angular_z):
		move_cmd = Twist()
		move_cmd.linear.x = linear_x
		move_cmd.angular.z = angular_z
		self.pub_vel.publish(move_cmd)

