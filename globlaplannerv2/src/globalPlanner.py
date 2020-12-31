#!/usr/bin/env python3
"""Globla planner."""

import pickle as pic
import math
import time
import sys
import os
import random
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
import numpy as np

# edge = True
visitedBool = False
vistedPoint = Vector3( 0, 0, 0)
refListPath = "/home/woombat/Documents/my_test_project/refList/"
fileName = "visited_point.txt"
radii = 10
degrees = 33
rngTime =0.1
filename = refListPath + fileName
ravinL = False
ravinR = False
ravinB = False
ravinT = False

#fileObject = open(filename, 'w')
#global initialTime

pathIdx = 0
global position
global n
n = 1

# def vistedEgde(data):
#     global edge
#     edge = data.data
#     rospy.loginfo(rospy.get_caller_id() + "We have seen an edge %s", data.data)
#     rospy.spin()

def vistedCB(data):
    global visitedBool
    visitedBool = data.data
    rospy.loginfo(rospy.get_caller_id() + "We visited the point %s", data.data)
    

def vistedPointCB(data):
    global refListPath, t, spentSecs, fileObject, position
    #print("New point visted")
#    filename = refListPath + fileName

    #fileObject = open(filename, 'wb')

    x_coord = data.x
    y_coord = data.y
    z_coord = data.z
    t = data.w
    position = (x_coord, y_coord, z_coord)
    #rospy.loginfo("entered")
    #if not initialTime:
    #	initialTime = time
    msgData ="{},{},{},{}\n".format(t,x_coord,y_coord,z_coord)
    #print(msgData)
    #pic.dump(msgData, fileObject)
    #fileObject.write(msgData)
    #spentSecs = time - initilaTime
    #fileObject.close()

def resetCallback(msg):
	global reset
	reset = msg.data
	pass

def ravinLCallback(msg):
	global ravinL
	ravinL = msg.data
	pass

def ravinRCallback(msg):
	global ravinR
	ravinR = msg.data
	pass

def ravinTCallback(msg):
	global ravinT
	ravinT = msg.data
	pass

def ravinBCallback(msg):
	global ravinB
	ravinB = msg.data
	pass

def setupFile(n):
    #model file
    #scale factor x,y,z,bool
    #serial number structured
    global refListPath, fileName, fileObject

    fn = "/home/woombat/Documents/my_test_project/refList/visited_point{}.txt".format(n)
    fileObject = open(fn, 'w')

    filename = refListPath + fileName
    #fileObject = open(filename, 'wb')
    shipFile = "text\n"
    #pic.dump(shipFile, fileObject)
    fileObject.write(shipFile)
    scaleFactor = "{},{},{},{}\n".format(1.0,1.0,1.0,1)
    #pic.dump(scaleFactor, fileObject)
    fileObject.write(scaleFactor)
    serialNumber = str(n) + "\n"
    #pic.dump(serialNumber, fileObject)
    fileObject.write(serialNumber)
#    fileObject.close()
    print("New file made")
    return


def pol2cart(theta, r):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return(x, y)

#fixed angle
def rndWalk():
    global radii, degrees, position, ravinL, ravinR, ravinT, ravinB
    # turnTheta = 90*math.pi/180
    if ravinL or ravinR or ravinT or ravinB:
        # degs = []
        # if ravinL:
        #     degs.append(0)
        # if ravinR:
        #     degs.append(180)
        # degrees += random.randint(140,220)
        # turnTheta = degrees * math.pi/180
        # x,y = pol2cart(turnTheta,radii)
        # ravinL = False
        x = -position[0]
        y = -position[1]
    else:
        degrees = degrees + 33
        turnTheta = degrees * math.pi /180
        x,y = pol2cart(turnTheta,radii)
        # edge = False
    x += position[0]
    y += position[1]
    return x,y,position[2]

#rng Turn
def rndWalkRngTurn():
    global theta, rngTime, position, ravinL, ravinR, ravinT, ravinB, degrees
    radii = 2#10
    # degrees  = 0
    turnTheta = 0
    x,y = 0, 0
    now = time.time()
    elapsed = now - rngTime
    # if ravinL: #(edge == True and elapsed >  60) or ravinL == True:
    #     degrees = degrees + random.randint(140,220)
    #     turnTheta = degrees * math.pi /180
    #     x,y = pol2cart(turnTheta,radii)
    #     # edge = False
    #     rngTime = time.time()
    #     ravinL = False
    if ravinL or ravinR or ravinT or ravinB:
        x = -position[0]
        y = -position[1]
        rngTime = time.time()

    if elapsed > 60:
        degrees = degrees + random.randint(140,220)
        turnTheta = degrees * math.pi /180
        x,y = pol2cart(turnTheta,radii)
        rngTime = time.time()


    else:
        x,y = pol2cart(turnTheta,radii)
    x += position[0]
    y += position[1]
    return x,y,position[2]

def reseting():
	x,y,z = rndWalkRngTurn()
	return x,y,z

def txtToArray(txtName):
    global refListPath
    csv_file = refListPath + txtName
    arr = []
    with open(csv_file, 'r') as csv_file:
        for line in csv_file:
            list = line.split (",")
            inputArr = []

            x = float(list[0])
            y = float(list[1])
            inputArr = [x,y]
            arr.append(inputArr)
    return arr

def Pathwalk(arr):
	global visitedBool,pathIdx, position

	if visitedBool == False:
	    x = arr[pathIdx][0]
	    print("old coordinates")
	    y = arr[pathIdx][1]
	else:
	    pathIdx = pathIdx + 1
	    x = arr[pathIdx][0]
	    print("new coordinates")
	    y = arr[pathIdx][1]

	z = position[2]
	return x,y,z


def mainGlobalPlan():


    arrPlane = txtToArray("planePath.txt")
    arrCylinder = txtToArray("cylinderPath.txt")
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('globalPlanner', anonymous=True)
    sleep = rospy.Rate(50)
    pub_goal = rospy.Publisher("goal", Vector3, queue_size=1)
    rospy.Subscriber("reachPoint", Bool, vistedCB)
    # rospy.Subscriber("reachEdge", Bool, vistedEgde)
    rospy.Subscriber("path",Quaternion,vistedPointCB)
    rospy.Subscriber("reset",Bool,resetCallback)
    rospy.Subscriber("ravinL",Bool,ravinLCallback)
    rospy.Subscriber("ravinR",Bool,ravinRCallback)
    rospy.Subscriber("ravinT",Bool,ravinTCallback)
    rospy.Subscriber("ravinB",Bool,ravinBCallback)
    finish_pub = rospy.Publisher("finish",Bool,queue_size=2)
    reset_pub = rospy.Publisher("reset",Bool,queue_size=2)



    global t, fileObject, n, reset, position
    position = (0,0,0)
    #rospy.loginfo(type(initialTime))
    #rospy.loginfo(type(time))
    t = 0
    reset = False
    counter = 10
    #n = 1
    #setupFile(n)
    rospy.loginfo("runnings")
    while not rospy.is_shutdown():
        global visitedBool
        initialTime = 0

        if reset:
            x,y,z = reseting()
            reset_pub.publish(Bool(False))
            goalPoint = Vector3()
            goalPoint.x = float(x)
            goalPoint.y = float(y)
            goalPoint.z = float(0)
            pub_goal.publish(goalPoint)
            egde = False
            sleep.sleep()

        #time = 0
        #follow ref. list
        data = False
        pathwalk = True
        navigationFunc = True
        if t - counter >= 10:
	        #Generate path
            if navigationFunc == True:
	            ranWalk = False
	            rngWalk = False

	            if rngWalk == True and visitedBool == True:
	                x,y,z = rndWalkRngTurn()
	                goalPoint = Vector3()
	                goalPoint.x = float(x)
	                goalPoint.y = float(y)
	                goalPoint.z = float(z)
	                pub_goal.publish(goalPoint)
	                egde = False
	                #sleep.sleep()
	            if ranWalk == True and visitedBool == True:
	                x,y,z = rndWalk()
	                goalPoint = Vector3()
	                goalPoint.x = float(x)
	                goalPoint.y = float(y)
	                goalPoint.z = float(z)
	                pub_goal.publish(goalPoint)
	                egde = False
	                #sleep.sleep()
	            if 1:
	                x,y,z = Pathwalk(arrPlane)
	                goalPoint = Vector3()
	                goalPoint.x = x
	                goalPoint.y = y
	                goalPoint.z = z
	                pub_goal.publish(goalPoint)
	                sleep.sleep()


	            if initialTime == 0:
	                #rospy.loginfo("in")
	                initialTime = t
	            spentSecs = t - initialTime



	        #close down webots
            nomoves = False
            if nomoves == True:
	            print("Quiting")
	            rospy.spin()
	            os.kill(0,True)
            counter = t
    #		"""if spentSecs >= 3600*24:
    #			n += 1
    #			break"""
        #rospy.loginfo(time)
        if t  >= 60*60:
            # rospy.loginfo("ok")
            bool_pub = Bool()
            bool_pub.data = True
            finish_pub.publish(bool_pub)
            t = 0
            #bool_pub.data = False
            #finish_pub.publish(bool_pub)
            n += 1
            #fileObject.close()
            #setupFile(n)
            #break

        #counter += 1

        pass

    # spin() simply keeps python from exiting until this node is stopped
    #fileObject.close()
    rospy.spin()
    #fileObject.close()

if __name__ == '__main__':
    try:
        mainGlobalPlan()
    except rospy.ROSInterruptException:
        pass
