#!/usr/bin/env python3
from controller import Robot
from controller import InertialUnit
from controller import Gyro
from controller import Keyboard
from controller import Motor
from controller import Pen
from controller import GPS
from controller import Device
from controller import Supervisor


import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from rosgraph_msgs.msg import Clock

import math
import numpy as np
import time
import os 

#robotPoint
boundary = Supervisor()
robot_node = boundary.getFromDef("Submarine")


rospy.init_node('boundarys', anonymous=True)
sleep = rospy.Rate(5)
pub_boundary = rospy.Publisher("boundary", Quaternion, queue_size=1)
clock_Publisher = rospy.Publisher("sim_time", Clock, queue_size=1)


def plane(P,Q,R):
    a = np.subtract(Q,P)
    b = np.subtract(R,P)
    return np.cross(a,b)

def dist_to_plane(normVector,point,robotCoord):
    x1 = normVector[0]*robotCoord[0]+normVector[1]*robotCoord[1]+normVector[2]*robotCoord[2]
    x0 = normVector[0]*point[0]+normVector[1]*point[1]+normVector[2]*point[2]
    den = x1-x0
    nom = np.sqrt(normVector[0]*normVector[0] + normVector[1] * normVector[1]+ normVector[2]*normVector[2])
    return np.divide(den,nom)

timeStep = int(boundary.getBasicTimeStep())
boundary.step(timeStep)



gpsStarboardBow = boundary.getGPS('gpsStarboardBow')
gpsStarboardBow.enable(timeStep)

xpos, ypos , zpos = gpsStarboardBow.getValues()
starboardBowArr = np.array([xpos, ypos , zpos])

gpsStarboardStern = boundary.getGPS('gpsStarboardStern')
gpsStarboardStern.enable(timeStep)

xpos, ypos , zpos = gpsStarboardStern.getValues()
starboardSternArr = np.array([xpos, ypos , zpos])

gpsPortBow = boundary.getGPS('gpsPortBow')
gpsPortBow.enable(timeStep)

xpos, ypos , zpos = gpsPortBow.getValues()
portBowArr = np.array([xpos, ypos , zpos])

gpsPortStern = boundary.getGPS('gpsPortStern')
gpsPortStern.enable(timeStep)

xpos, ypos , zpos = gpsPortStern.getValues()
portSternArr = np.array([xpos, ypos , zpos])

gpsBottumBow = boundary.getGPS('gpsBottumBow')
gpsBottumBow.enable(timeStep)

xpos, ypos , zpos = gpsBottumBow.getValues()
bottumBowArr = np.array([xpos, ypos , zpos])

gpsBottumStern = boundary.getGPS('gpsBottumStern')
gpsBottumStern.enable(timeStep)

xpos, ypos , zpos = gpsBottumStern.getValues()
bottumSternArr = np.array([xpos, ypos , zpos])

gpsBottumMid = boundary.getGPS('gpsBottumMid')
gpsBottumMid.enable(timeStep)

xpos, ypos , zpos = gpsBottumMid.getValues()
bottumMidArr = np.array([xpos, ypos , zpos])


if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")




while boundary.step(timeStep) != -1 and not rospy.is_shutdown():

    robotpoint = trans_field.getSFVec3f()
    #print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))

    bowNorm = plane(bottumBowArr,portBowArr,starboardBowArr)
    topNorm = plane(portSternArr,portBowArr,starboardBowArr)
    sternNorm = plane(bottumSternArr,portSternArr,starboardSternArr)
    botNorm = plane(bottumSternArr,bottumMidArr,bottumBowArr)

    bowDist = -dist_to_plane(bowNorm,bottumBowArr,robotpoint)
    topDist = dist_to_plane(topNorm,portSternArr,robotpoint)
    sternDist = dist_to_plane(sternNorm,bottumSternArr,robotpoint)
    botDist = -dist_to_plane(botNorm,bottumSternArr,robotpoint)
    
    
    #print("bow dist: {} top dist: {} stern dist: {}".format(xpos, ypos , zpos))
    #print("bow dist: {} top dist: {} stern dist: {}".format(bowDist,topDist,sternDist))
    
    
    boolBow = False
    boolTop = False
    boolStern = False
    boolKeel = False
    if bowDist < 0:
        boolBow = True
    if topDist < 0:
        boolTop = True
    if sternDist < 0:
        boolStern = True
    if sternDist < 0:
        boolKeel = True

    #print(float(boolBow))
    boolBound = Quaternion()
    boolBound.x = float(boolBow)
    boolBound.y = float(boolTop)
    boolBound.z = float(boolStern)
    boolBound.w = float(boolKeel)
    pub_boundary.publish(boolBound)

    # pulish simulation clock
    msg = Clock()
    time = int(boundary.getTime())
    msg.clock.secs = int(time)
    # round prevents precision issues that can cause problems with ROS timers
    msg.clock.nsecs = int(boundary.getTime()*1000000)
    clock_Publisher.publish(msg)
    
