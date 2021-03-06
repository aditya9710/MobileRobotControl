#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():
	def __init__(self):
		# publisher for rotor speeds
		self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
		
		# subscribe to Odometry topic
		self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",
		Odometry, self.odom_callback)
		
		self.t0 = None
		self.t = None
		self.t_series = []
		self.x_series = []
		self.y_series = []
		self.z_series = []
		self.mutex_lock_on = False
		rospy.on_shutdown(self.save_data)

		# TODO: include initialization codes if needed

	def traj_evaluate(self):
		""" TODO: evaluating the corresponding trajectories designed in Part 1
			to return the desired positions, velocities and accelerations"""

		pass

	def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
		""" obtain the desired values by evaluating the corresponding
		trajectories"""
		self.traj_evaluate()

		""" TODO: implement the Sliding Mode Control laws designed in Part 2 to
			calculate the control inputs "u" """
		
		# REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
		
		""" TODO: convert the desired control inputs "u" to desired rotor 
		velocities "motor_vel" by using the "allocation matrix" """
		
		""" TODO: maintain the rotor velocities within the valid range of [0 to
		2618] """

		# publish the motor velocities to the associated ROS topic
		motor_speed = Actuators()
		motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0],
		motor_vel[2,0], motor_vel[3,0]]
		self.motor_speed_pub.publish(motor_speed)

	# odometry callback function (DO NOT MODIFY)
	def odom_callback(self, msg):
		if self.t0 == None:
			self.t0 = msg.header.stamp.to_sec()
		self.t = msg.header.stamp.to_sec() - self.t0
		
		# convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
		w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
		v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
		xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
		q = msg.pose.pose.orientation
		T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
		T[0:3, 3] = xyz[0:3, 0]
		R = T[0:3, 0:3]
		xyz_dot = np.dot(R, v_b)

		rpy_dot = np.dot(np.asarray([
		[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
		[0, np.cos(rpy[0]), -np.sin(rpy[0])],
		[0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
		]), w_b)
		rpy = np.expand_dims(rpy, axis=1)

		# store the actual trajectory to be visualized later
		if (self.mutex_lock_on is not True):
			self.t_series.append(self.t)
			self.x_series.append(xyz[0, 0])
			self.y_series.append(xyz[1, 0])
			self.z_series.append(xyz[2, 0])

		# call the controller with the current states
		self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

		# save the actual trajectory data
		def save_data(self):
			# TODO: update the path below with the correct path
			with open("/home/sfarzan/rbe502_project/src/project/scripts/log.pkl", "wb") as fp:
				self.mutex_lock_on = True
				pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)

if __name__ == '__main__':
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")