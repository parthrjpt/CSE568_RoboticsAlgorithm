#!/usr/bin/env python

import rospy
import math
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
#from nav_msgs.msg import Odometry


# initializign Laserscan object and AckermanDriveStamped as seen in the random_walker.cpp files
#threshold was set to one after experimentinh with various values like 20,10,5 and 1.5. The evader would turn early on.
# Initially a rate was added but it meant that the subscriber would only turn if the callback publisher had published the command to turn. It was therefore removed


laser = LaserScan()
threshold = 1
driver=AckermannDriveStamped()

def callback(data):
	#rospy.loginfo(rospy.get_caller_id()+" %s",data.ranges) #Consumes memory, therefore just print the balues if debugging is required
	drivepub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10) 
	ranges=np.array(data.ranges)
	mid=(len(ranges)-1)/2 #Initially the the check for any value in ranges being <threshold was used however, due to the span of the ranges and angle being too huge.. just the range value in front was used.
	if(ranges[int(mid)]<threshold):
		rospy.loginfo("collision Approaching")
		#randomdir=-1
		#To include randomness in direction the below logic was tested 
		#if(np.random.random_sample()>0.5):
		#	randomdir=-1
		#else:
		#	randomdir=1
		driver.drive.speed=0.8 #if threshold is crossed, then reduce the speed and change angle
		#driver.drive.steering_angle= randomdir*math.pi/6
		driver.drive.steering_angle= -math.pi/4
  		driver.drive.steering_angle_velocity= math.pi/6
	else:
		driver.drive.speed=2.0 # else change speed back to 2 and re-initialize other values updated
		driver.drive.steering_angle= 0
		driver.drive.steering_angle_velocity= 0

	drivepub.publish(driver)



def controller(): 		
	rospy.init_node('evader', anonymous=True)	  
	#rate = rospy.Rate(1000) # 10hz
	laser.angle_min= -np.pi/180
	laser.angle_max= np.pi/180

	pub1 = rospy.Publisher('scan', LaserScan, queue_size=10)
  	pub1.publish(laser)	
	
	while not rospy.is_shutdown():
		rospy.Subscriber('scan', LaserScan,callback)
		#rate.sleep()
		rospy.spin()

if __name__ == '__main__':
    try:
        controller()  
    except rospy.ROSInterruptException:
        pass
