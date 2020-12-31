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

edge = True
visitedBool = False
vistedPoint = Vector3( 0, 0, 0)
refListPath = "/home/ros/Documents/my_test_project/refList/"
fileName = "visited_point.txt"
radii = 10
degrees = 33
rngTime =0.1
filename = refListPath + fileName
fileObject = open(filename, 'w')

def vistedEgde(data):
    global edge
    edge = data.data
    rospy.loginfo(rospy.get_caller_id() + "We have seen an edge %s", data.data)
    rospy.spin()

def vistedCB(data):
    global visitedBool
    visitedBool = data.data
    rospy.loginfo(rospy.get_caller_id() + "We visited the point %s", data.data)
    rospy.spin()

def vistedPointCB(data):
    global refListPath, fileName, initialTime, spentSecs
    print("New point visted")
#    filename = refListPath + fileName

    #fileObject = open(filename, 'wb')
    
    x_coord = data.x
    y_coord = data.y
    z_coord = data.z
    time = data.w
    if not initialTime:
    	initialTime = time
    msgData ="{},{},{},{}\n".format(time,x_coord,y_coord,z_coord)
    print(msgData)
    #pic.dump(msgData, fileObject)
    fileObject.write(msgData)
    spentSecs = time - initilaTime
    #fileObject.close()
    

def setupFile(n):
    #model file
    #scale factor x,y,z,bool
    #serial number structured
    global refListPath, fileName
    
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
    global edge, radii, degrees
    turnTheta
    if edge == False:
        x,y = pol2cart(turnTheta,radii)
    else:
        degrees = degrees + 33
        turnTheta = degrees * math.pi /180
        x,y = pol2cart(turnTheta,radii)
        edge = False
    return x,y

#rng Turn
def rndWalkRngTurn():
    global theta, edge, rngTime
    radii = 10
    degrees  = 0
    turnTheta = 0
    x,y = 0, 0
    now = time.time()
    elapsed = now - rngTime
    if edge == True and elapsed >  60:
        degrees = degrees + random.randint(1,40)
        turnTheta = degrees * math.pi /180
        x,y = pol2cart(turnTheta,radii)
        edge = False
        rngTime = time.time()
    else:
        x,y = pol2cart(turnTheta,radii)
    return x,y


def mainGlobalPlan():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('globalPlanner', anonymous=True)
    sleep = rospy.Rate(5)
    pub_goal = rospy.Publisher("goal", Vector3, queue_size=1)
    rospy.Subscriber("reachPoint", Bool, vistedCB)
    rospy.Subscriber("reachEdge", Bool, vistedEgde)
    rospy.Subscriber("path",Quaternion,vistedPointCB)
    
    global initialTime
    initialTime = 0
    
    n = 1
    setupFile(n)
    rospy.loginfo("runnings")
    while not rospy.is_shutdown():
        global visitedBool
        #follow ref. list
        data = False
        navigationFunc = True
        if data == True:
            csv_file = refListPath + "refList1.csv"
            with open(csv_file, 'wb') as csv_file:
                for line in csv_file:
                    line = line.strip('\n')
                    #print(line)
                    coord = line.split(' ') # take the pickled obj
                    x_coord = coord[0]
                    y_coord = coord[1]
                    z_coord = coord[2]
                    #print("x coord: {}   y coord: {}    z z_coord: {}").format(x_coord,y_coord,z_coord)
                    #to localPlanner returns points that has been missing
                    while visitedBool == False:
                        goalPoint = Vector3()
                        goalPoint.x = float(x_coord)
                        goalPoint.y = float(y_coord)
                        goalPoint.z = float(z_coord)
                        pub_goal.publish(goalPoint)
                        sleep.sleep()
                    visitedBool = False


        #Generate path
        if navigationFunc == True:
            ranWalk = False
            rngWalk = True
            if rngWalk == True:
                x,y = rndWalkRngTurn()
                goalPoint = Vector3()
                goalPoint.x = float(x)
                goalPoint.y = float(y)
                goalPoint.z = float(0)
                pub_goal.publish(goalPoint)
                egde = False
                sleep.sleep()
            if ranWalk == True:
                x,y = rndWalk()
                goalPoint = Vector3()
                goalPoint.x = float(x)
                goalPoint.y = float(y)
                goalPoint.z = float(0)
                pub_goal.publish(goalPoint)
                egde = False
                sleep.sleep()



        #close down webots
        nomoves = False
        if nomoves == True:
            print("Quiting")
            rospy.spin()
            os.kill(0,True)

		if spentSecs >= 3600*24:
			n += 1
			break
        pass

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    fileObject.close()

if __name__ == '__main__':
    try:
        mainGlobalPlan()
    except rospy.ROSInterruptException:
        pass
