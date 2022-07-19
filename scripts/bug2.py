#!/usr/bin/env python3

import roslib
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from nav_msgs.msg import Odometry

roslib.load_manifest('lab6')



odom = Odometry()

# defining front, left and right of robot
def callback(msg):
    global Front, Left, Right, allRanges
    allRanges = msg
    Right = msg.ranges[:120]
    Front = msg.ranges[120:240]
    Left = msg.ranges[240:360]

# defining position and orientation of robot
def callback1(data):
    global x, y, theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    theta = data.pose.pose.orientation.z

# convert angles to ranges, ex: [0,pi/2] to ranges[180:360]
def anglesTOranges(minAngle,maxAngle):
    rangesBtnAngles = allRanges.ranges[int((((360/math.pi)*minAngle))+180):int(((360/math.pi)*maxAngle)+180)]
    return rangesBtnAngles

# determines whether obstacle in way or not
def obstPrsnt(angleDiff):
    phiMin = angleDiff - 0.5
    phiMax = angleDiff + 0.5

    if phiMin < -math.pi/2 or phiMax > math.pi/2:
        phiMin = -math.pi/2
        phiMax = math.pi/2
    rangesBtnAngles = allRanges.ranges[int((((360/math.pi)*phiMin))+180):int(((360/math.pi)*phiMax)+180)]
    minRange = min(rangesBtnAngles)

    return minRange < 1

# defining forward velocity value based on dist. from obstacle
def calFWDvel():
    if min(Front) < 0.5:
        return 0
    else:
        return min(1, min(Front)-0.5)

# rotational velocity while GOALSEEK
def calGSeekAngVel(angleDiff):
    if abs(angleDiff) < 0.03:
        return 0
    else:
        return 2*angleDiff

# rotational velocity while WALLFOLLOW : + CW
def calWFangVel():
    if max(min(Left),min(Front)) < 2:
        return -1
    else:
        return min(Left)

# check whether robot is on m-line or not
def onMline():
    dist = abs(((goaly-starty)*x) - ((goalx-startx)*y) + ((goalx*starty) - (goaly*startx))) / math.sqrt(((goaly-starty)**2) + ((goalx-startx)**2))
    return dist < 0.5

# bug2 controller
def bug():
    rospy.init_node('bug', anonymous = True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel = Twist()

    rospy.Subscriber("/base_scan", LaserScan, callback)
    rospy.Subscriber("/odom", Odometry, callback1)
    
    GOALSEEK = True
    WALLFOLLOW = False
    atGoal = False

    rate = rospy.Rate(10) 
    rospy.sleep(1)
    while not atGoal:
        # defining parameters
        global angleDiff, goalx, goaly, startx, starty
        # startx, starty = -8, -2
        startx = rospy.get_param('~startx')
        starty = rospy.get_param('~starty')
        goalx = rospy.get_param('~goalx')
        goaly = rospy.get_param('~goaly')
        robotAngle = math.asin(theta)
        distFrmGoal = math.sqrt((goalx - x) ** 2 + (goaly - y) ** 2)
        angleDiff = math.atan((goaly - y) / (goalx- x)) - 2*robotAngle

        if distFrmGoal < 0.5:
            print("at Goal!")
            linerVel = 0
            angularVel = 0
            atGoal = True
            break
        else:
            linerVel = calFWDvel()
            if GOALSEEK:
                angularVel = calGSeekAngVel(angleDiff)
                if obstPrsnt(angleDiff):
                    WALLFOLLOW = True
                    GOALSEEK = False

            if WALLFOLLOW:
                angularVel = calWFangVel()
                if onMline():
                    angularVel = -2
                    GOALSEEK = True
                    WALLFOLLOW = False
                    

            vel.linear.x = linerVel 
            vel.angular.z = angularVel
        pub.publish(vel)
        rate.sleep()

  
if __name__ == '__main__':

    try:
        bug()
    except rospy.ROSInterruptException:
        pass


