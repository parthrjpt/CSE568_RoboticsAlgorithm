#!/usr/bin/env python

import rospy
import math
import random
import numpy as np
import heapq
import os

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion

'''
map=[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]
'''

import os,sys

file_path=sys.path[0]
file_path = file_path.replace('node','maps/map.txt')
f= open(file_path,"r")
fstr=f.read()
f.close()
fstr = fstr.replace('map = ','')
fstr=fstr.replace('[','')
fstr = fstr.replace(']','')
fstr = fstr.replace(',',' ')
fstr = fstr.replace(' ','')
fstr = fstr.replace('\n','')
map=np.array(list(fstr),dtype=np.int64)
print(map)
       
class AStar:
	def __init__(self,start,goal):
		rospy.init_node("astar",anonymous=True)
		self.grid=np.asarray(map).reshape(20,18)
		self.start=start
		self.goal=goal
		adjusted_start = self.map_to_grid(self.start)
		adjusted_goal = self.map_to_grid(self.goal)
		gridpath=self.astar(adjusted_start,adjusted_goal)
		self.rotate=True
		self.isGoal=False
		#print(gridpath)
		#print("path computed")
		if(len(gridpath)>0):
			path=[]
			for pt in gridpath:
				mpt = self.grid_to_map(pt)
				path.append(mpt)
		#print(path)
		self.path = path[:-1] + [self.goal]
		self.iter = 0
		self.rotate = True
		rospy.Rate(10)
		self.movepub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
		rospy.Subscriber("base_pose_ground_truth",Odometry,self.callback)
		rospy.spin()
	
	def ssd(self,p1,p2):
		return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
	
	def reconstruct_path(self,parent, current):
		path=[]
		while current in parent:
			path.append(current)
			current = parent[current]
			#print("recon node :",current)	
		return path[::-1]
	 
	def check_if_exists(self, node,grid):
		(gridx_len,gridy_len)=(grid.shape[0],grid.shape[1])
		#print("initial neighbor: ", node)
		if((node[0]>=0) and (node[0]<gridx_len) and (node[1]>=0) and (node[1]<gridy_len) and (grid[node[0]][node[1]]==0)):
			#print(True)
			return True
		else:
			#print(False)
			return False
		
	def map_to_grid(self,point):
		return(int(round(9.5-point[1])),int(round(8+point[0])))

	def grid_to_map(self,point):
		return(int(round(point[1]-8)),int(round(9.5-point[0])))

	def astar(self,start,goal):
		goal=goal
		start=start
		grid=self.grid
		parent,g,f = {}, {start:0},{start:self.ssd(start, goal)}
		openset= []
		heapq.heappush(openset,(f[start],start))
		actions=[(0,1),(0,-1),(1,0),(-1,0)]
		visited=[]
		while openset:
			#print("New cycle")
			current = heapq.heappop(openset)[1]
			#print("current: ",current)
			if current == goal:
				print("end Astar")
				return self.reconstruct_path(parent,current)				
				
			visited.append(current)
			for action in actions:
				neighbor= (current[0]+action[0],current[1]+action[1])
				if self.check_if_exists(neighbor,self.grid):
					temp_g= g[current]+self.ssd(current,neighbor)
					#print("exists")
					#print("neighbor, and g : ",neighbor,temp_g)
					if not(neighbor in visited and temp_g>=g.get(neighbor,0)):
						if  (neighbor not in [ele[1] for ele in openset]):
							#print("g updated")
							g[neighbor] = temp_g
							f[neighbor] = g[neighbor] + self.ssd(neighbor, goal)
							parent[neighbor] = current
							heapq.heappush(openset, (f[neighbor], neighbor))
							#print("updated heap :",openset)
		print("end Astar")					
		return False
				
	def callback(self,data):
		self.pos=data.pose.pose.position
		self.orientation = data.pose.pose.position	
		if (self.isGoal==False):
			path_node= self.path[self.iter]
			quaternion=(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
			(roll,pitch,yaw) = euler_from_quaternion(quaternion)
			current_to_goal=self.ssd(self.goal,(self.pos.x,self.pos.y))
			if(current_to_goal<0.7):
				self.isGoal=True
				print("Goal reached")
			elif(self.rotate==True):
				angle = math.atan2(path_node[1] - self.pos.y, path_node[0] - self.pos.x) - yaw
				if abs(angle) > 0.05:
					print("Changing Orientation")
					msg = Twist()
					msg.angular.z = 3*angle
					self.movepub.publish(msg)
				else:
					self.rotate = False
			else:
				dist = self.ssd((self.pos.x, self.pos.y), (path_node[0], path_node[1]))
				if(dist> 0.3):
					print("Proceeding towards path point")
					msg = Twist()
					msg.linear.x = 3*dist
					self.movepub.publish(msg)
				else:
					self.iter += 1
					self.rotate = True	
				


if __name__ == '__main__':
	try:
		start=(rospy.get_param('start_x'),rospy.get_param('start_y'))
		goal=(rospy.get_param('goal_x'),rospy.get_param('goal_y'))
		bug=AStar(start,goal)		
	except rospy.ROSInterruptException:
		pass
