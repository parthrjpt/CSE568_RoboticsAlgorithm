#!/usr/bin/env python

import rospy
import math
import random
import numpy as np

from sensor_msgs.msg import LaserScan


from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



def getLaserScanPoints(data):
	ranges = data.ranges
	maxval = data.range_max
	min_ang = data.angle_min
	max_ang = data.angle_max
	ang_inc = data.angle_increment
	rangesarr=[]
	x=0
	y=0
	print(min(ranges))
	for i in range(len(ranges)):
		angle_start = min_ang
		if(ranges[i] < maxval):
			delta = i*ang_inc
			x=ranges[i]*np.cos(angle_start+delta)
			y=ranges[i]*np.sin(angle_start+delta)
			rangesarr.append([x,y])
	return rangesarr
			
	

def Ransac(ranges):
	x_startArr=[]
	y_startArr=[]
	x_endArr=[]
	y_endArr=[]
	initial_len_ranges=len(ranges)
	potentialInliers = np.asarray(ranges)		
	while(len(potentialInliers)>0.5*initial_len_ranges):
		inliers_to_remove=[]
		x_start, y_start, x_end, y_end = 0, 0, 0, 0
		k=100
		d=10
		itera=0
		while itera<k:
			actualInliers=[]
			
			rand_pts_idx = np.random.choice(len(potentialInliers)-1,2,replace=False) 
			rand_pts =[potentialInliers[i] for i in rand_pts_idx]
			
			x0,y0= rand_pts[0]
			x1,y1= rand_pts[1]
			for pt in potentialInliers:
				x2,y2=pt
				dist_pts_line = np.abs((x1-x0)*(y0-y2)-(y1-y0)*(x0-x2)/math.sqrt((x1-x0)**2+(y1-y0)**2))
				if(dist_pts_line<0.2):
					actualInliers.append(pt)
					
			if(len(actualInliers)>d):
				d = len(actualInliers)
				x_start = x0
				y_start = y0
				x_end = x1
				y_end = y1
				inliers_to_remove=actualInliers

			itera += 1
		potentialInliers = potentialInliers.tolist()
		for i in range(len(inliers_to_remove)):
			potentialInliers.remove(inliers_to_remove[i].tolist())
		#potentialInliers = potentialInliers.remove([x_start,y_start])
		#potentialInliers = potentialInliers.remove([x_end,y_end])
		potentialInliers = np.asarray(potentialInliers)
		x_startArr.append(x_start)
		y_startArr.append(y_start)
		x_endArr.append(x_end)
		y_endArr.append(y_end)
	return x_startArr, y_startArr, x_endArr, y_endArr



def callback(data):
	markerpoint = Marker()

	ranges=getLaserScanPoints(data)
	markerpoint.header.frame_id = "/base_link"
	#markerpoint.header.stamp = rospy.Time.now()
	markerpoint.lifetime = rospy.Duration()
	markerpoint.type = Marker().LINE_STRIP
	#markerpoint.action = Marker().ADD
	#markerpoint.ns = "points"
	markerpoint.scale.x=0.01
	markerpoint.pose.orientation.w=1

	markerpoint.color.g = 1.0;
	markerpoint.color.a = 1.0;

	#print(ranges)
	if(len(ranges)>3):
		x_startArr,y_startArr,x_endArr,y_endArr = Ransac(ranges)
		
		op_arr_start= [[x_startArr[i],y_startArr[i]] for i in range(len(x_startArr)) ]
		op_arr_end= [[x_endArr[i],y_endArr[i]] for i in range(len(x_endArr)) ]
		for start,end in zip(op_arr_start,op_arr_end):
			p1=Point()
			p1.x=start[0]
			p1.y=start[1]
			p2=Point()
			p2.x=end[0]
			p2.y=end[1]
			markerpoint.points.append(p1)
			markerpoint.points.append(p2)
		#p1.x=x_start
		#p1.y=y_start
		#p2.x=x_end
		#p2.y=y_end
		#print(x_start,y_start,x_end,y_end)
		#rospy.loginfo(rospy.get_caller_id()+"x:  %s",p1)
		#markerpoint.points.append(p1)
		#markerpoint.points.append(p2)
		#print(markerpoint.points)
		pub.publish(markerpoint)

if __name__ == '__main__':
	try:
		rospy.init_node('ransac', anonymous=True)
		pub = rospy.Publisher("ransac_vis", Marker, queue_size=10)
		
		while not rospy.is_shutdown():
			
			rospy.Subscriber('base_scan', LaserScan,callback)	
			rospy.spin()

		
	except rospy.ROSInterruptException:
		pass
