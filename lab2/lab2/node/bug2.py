#!/usr/bin/env python

import rospy
import math
import random
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Bug2():

	def __init__(self):	
		rospy.init_node("bug2",anonymous=True)
		self.movepub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
		self.pospub =rospy.Publisher("base_pose_ground_truth",Odometry,queue_size=10)
		start=[-8.0,-2.0]
		self.odom= Odometry()

		self.start = start
		self.goal = [4.5,9.0]
		self.current = [-8.0,-2.0]
		self.m_line_slope = (self.goal[1]-self.start[1])/(self.goal[0]-self.start[0])
		self.m_line_const= self.start[1]- self.m_line_slope*self.start[0]

		self.x_m=np.arange(-8.0,4.5,0.1)
		self.y_m=[self.m_line_slope*x+self.m_line_const for x in self.x_m]

		self.m_pts= [[self.x_m[n],self.y_m[n]] for n in range(len(self.x_m))]
		self.m_pts = np.asarray(self.m_pts)
		self.current_angle=0.0
		self.behavior="GOAL_SEEK"
		
		rospy.Subscriber('base_scan', LaserScan,self.laser_callback)	
		rospy.spin()


	def getLaserScanPoints(self,data):
		ranges = data.ranges
		maxval = data.range_max
		min_ang = data.angle_min
		max_ang = data.angle_max
		ang_inc = data.angle_increment
		rangesarr=[]
		x=0
		y=0
		for i in range(len(ranges)):
			angle_start = min_ang
			delta = i*ang_inc
			x=ranges[i]*np.cos(angle_start+delta)
			y=ranges[i]*np.sin(angle_start+delta)
			rangesarr.append([x,y])
		return rangesarr

	def mindist_approaching_obs(self,rawranges):
		is_obs_in_front = False
		is_obs_on_left = False
		obs_in_front=obs_in_left=0
		
		frontranges = rawranges[90:270]
		
		leftranges = rawranges[270:]
		return min(frontranges),min(leftranges)

	def calc_current_angle(self):
		current_angle = 2 * np.arcsin(self.current_angle)
		angle_from_goal = math.atan((self.goal[1] - self.current[1]) / (self.goal[0]- self.current[0])) - current_angle
		return angle_from_goal

	def is_approaching_goal_path(self,x,y):
		pt1_x=self.goal[0]
		pt1_y=self.goal[1]
		pt2_x=self.start[0]
		pt2_y=self.start[1]
		dist=np.abs((pt2_y-pt1_y)* x - (pt2_x - pt1_x)*y + pt2_x*pt1_y-pt2_y*pt1_x)/((pt2_y - pt1_y)**2+(pt2_x-pt1_x)**2)**0.5
		#print(x,y)
		#print(self.goal)
		#print(self.start)
		if dist<0.1:
			return True
		else:
			return False
		
	def is_goal_reached(self):
		dist = math.sqrt((self.current[0]-self.goal[0])**2+(self.current[1]-self.goal[1])**2)
		if (dist<0.1):
			return True
		else:
			return False
		
	def odom_callback(self,data):
		self.current[0]= data.pose.pose.position.x
		self.current[1]= data.pose.pose.position.y
		self.current_angle= data.pose.pose.orientation.z

	def laser_callback(self,data):
		#ranges = self.getLaserScanPoints(data)
		min_front,min_left=self.mindist_approaching_obs(data.ranges)
				
		if(not self.is_goal_reached()):
			rospy.Subscriber('base_pose_ground_truth',Odometry,self.odom_callback)
			self.twist =  Twist()
			angle_from_goal=self.calc_current_angle()
			print("behavior: ", self.behavior)	
			if self.behavior == "WALL_FOLLOW":
				self.twist.linear.x=0.5
				if(min_front<0.5):
					self.twist.angular.z=-0.3
					self.twist.linear.x=0
				elif(min_left<0.5):
					self.twist.angular.z=-0.3
					self.twist.linear.x=1
				elif(self.is_approaching_goal_path(self.current[0],self.current[1]) and min_front>0.7):
					#print("here")
					self.behavior="GOAL_SEEK"	
				elif(min_front>0.9 and min_left>0.9):
					self.twist.linear.x=0.3
					self.twist.angular.z=0.5					


					
			if self.behavior == "GOAL_SEEK":
				if(np.abs(angle_from_goal)>0.1 and self.is_approaching_goal_path(self.current[0],self.current[1])):
					self.twist.angular.z=angle_from_goal
					print("current angle", angle_from_goal)
				else:
					self.twist.linear.x=1
					if(min_front<0.5):
						self.behavior="WALL_FOLLOW"
						print("behavior changed to: ", self.behavior)				
			if(self.current==self.goal):
				self.twist.linear.x=0
				self.twist.linear.y=0
				self.twist.angular.z=0.0		
			self.movepub.publish(self.twist)

		


if __name__ == '__main__':
	try:
		bug=Bug2()
		
	except rospy.ROSInterruptException:
		pass
