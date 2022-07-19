#!/usr/bin/env python3

import roslib
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math
import random

roslib.load_manifest('lab6')


def callback(msg):
    global laserRanges
    laserRanges = msg

    
# RANSAC algorithm gives two points for line
def ransac(coords,iter,threshold):

    maxInliers = 0
    for k in range(iter):

        p1 = random.choice(coords)
        p2 = random.choice(coords)
        x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]

        if p1 != p2:
            totalInliers = 0
            for p in coords:
                x, y = p[0], p[1]

                dist = abs(((y2-y1)*x) - ((x2-x1)*y) + ((x2*y1) - (y2*x1))) / math.sqrt(((y2-y1)**2) + ((x2-x1)**2))

                if dist < threshold:
                    totalInliers += 1

            if totalInliers > maxInliers:
                maxInliers = totalInliers
                bestTwoPts = []
                bestTwoPts = [p1,p2]

    return bestTwoPts



# perception node publishes marker line of obstacle
def perception():
    rospy.init_node("perception", anonymous = True)
    publisher1 = rospy.Publisher("/marker", Marker, queue_size = 10)
    publisher2 = rospy.Publisher("/laser", LaserScan, queue_size = 10)
    rospy.Subscriber("/base_scan", LaserScan, callback)
    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
     
        rate.sleep()
        # convert laser detected points into cartezian coordinates
        cart_coord = []
        theta = laserRanges.angle_min
        for r in laserRanges.ranges:
            if r<3:
                x = r*math.cos(theta)
                y = r* math.sin(theta)
                cart_coord.append((x,y))
            theta += laserRanges.angle_increment

        if len(cart_coord) > 0:
            markerPoints = ransac(cart_coord,8,0.2)
            X1, Y1, X2, Y2 = markerPoints[0][0], markerPoints[0][1], markerPoints[1][0], markerPoints[1][1]
        else:
            X1, Y1, X2, Y2 = 0, 0, 0, 0

        # marker message
        # defining marker obj.
        marker = Marker()
        
        # marker message arguments
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0)

        # line start and end pots
        marker.points = []
    
        # first point
        first_point = Point()
        first_point.x = X1
        first_point.y = Y1
        first_point.z = 0.0
        marker.points.append(first_point)
    
        # second point
        second_point = Point()
        second_point.x = X2
        second_point.y = Y2
        second_point.z = 0.0
        marker.points.append(second_point)

        # Set Marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # Set Marker color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
    
        # publish marker and laser
        publisher1.publish(marker) 
        publisher2.publish(laserRanges)
       
      

      
        
    

if __name__ == '__main__':
    try:
        perception()
    except rospy.ROSInterruptException:
        pass
