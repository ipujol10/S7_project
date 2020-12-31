## [Import]
from controller import Supervisor
import math
import time
import rospy
import numpy as np

from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Bool, Float32

import griding
## ![Import]

##[Functions]

##[Callback]
def headingCallback(msg):
	global heading_msg
	heading_msg = msg
	pass

def finishCallback(msg):
	global finished
	finished = msg.data
	pass
##![Callback]

## [Drive - Side]
def drive_side(msg):
	angle = msg.y

	## [Control Side]
	side = angle/(10*math.pi/180) # if the angle is 10º, the side will be 1
									# if it's more, it will be bigger
									# if it's smaller, it will be smaller
	if abs(side) > 2: # if the side it's bigger than 2
		side = 2*side/abs(side) # saturate at a maximum of 2
	## ![Control Side]

	## [Control Drive]
	if side != 0: # is side is not 0
		drive = -(2/abs(side) - 1) # in case the side is 2 it will not have motion
									# if the side is smaller, drive will be grater
	else: # if side is 0
		drive = -4 # perfectly align and go fast
	if abs(drive) > 4:
		drive = -4 # saturation at -4
	## ![Control Drive]

	return drive, side
## ![Drive - Side]

## [Add noise]
def addNoise(real_point): # simulate the real error
	x,y,z = real_point # get the actual position

	xz_deviation = 1.02 # x and z standard deviation
	y_deviation = xz_deviation*.5 # y standard deviation (half x and z)

	## [Get new position]
	x += np.random.normal(loc=0, scale=xz_deviation)
	y += np.random.normal(loc=0, scale=y_deviation)
	z += np.random.normal(loc=0, scale=xz_deviation)
	## ![Get new position]

	return x,y,z
## ![Add noise]

## [Write the points and other data on a txt file]
def visitedPointCB(data, fileObject, tr, area, overlap, true_pos_list):
	## [Get the data from the point]
	x_coord = data.x
	y_coord = data.y
	z_coord = data.z
	time = data.w
	## ![Get the data from the point]

	s = "{},{},{},{}".format(x_coord,y_coord,z_coord,time) # add the robot position and time
	s += mtrx2str(tr) # add the rotation matrix

	s += "{},{},".format(area,overlap) # add the area and overlap values
	
	s += "{},{},{}".format(true_pos_list[0], true_pos_list[1], true_pos_list[2]) # add the true position

	msgData = s + "\n" # new line character
	fileObject.write(msgData) # write line
## ![Write the points and other data on a txt file]

## [Transform a 3x3 matrix into a string]
def mtrx2str(m):
	s = ","
	for row in m:
		for num in row:
			s += str(num) + ","
	return s
## ![Transform a 3x3 matrix into a string]

## [Start data file]
def setupFile(n):

	fn = "/home/ros/Documents/my_test_project/refList/visited_point{}.txt".format(n)
	fileObject = open(fn, 'w') # create or open the file to overwrite

	shipFile = "text" # placeholder
	f = open("/home/ros/underdasea/controllers/controller_1/fileNames.txt","r")
	read = f.read()
	read = read.split("\n")
	if n <= len(read): # if the name exists
		shipFile = read[n-1] # use it
	f.close()
	shipFile += "\n"
	fileObject.write(shipFile)
	scaleFactor = "{},{},{},{}".format(1.0,1.0,1.0,1)
	one = ",0"
	scaleFactor += 14*one
	scaleFactor += "\n"
	fileObject.write(scaleFactor)
	serialNumber = str(n) + "\n"
	fileObject.write(serialNumber)

	return fileObject
## ![Start data file]

## [Take the matrix and save it in a txt file]
def saveMatrix(grid, n):
	rows = len(grid)
	cols = len(grid[0])

	fn = "/home/ros/Documents/my_test_project/refList/visited_matrix{}.txt".format(n)
	matrixFile = open(fn,"w")
	matrixFile.write("{},{}\n".format(rows,cols))
	## [Flatten the matrix]
	for row in grid:
		for el in row:
			matrixFile.write("{},{}\n".format(el[0],el[1]))
	## ![Flatten the matrix]

	matrixFile.close()
## ![Take the matrix and save it in a txt file]

##![Functions]

##[Declare]
# create the Robot instance.
robot = Supervisor()
robot_node = robot.getSelf()
nFile = open("/home/ros/Documents/my_test_project/refList/number.txt","r")
n = int(nFile.readline()) # get the number test
nFile.close()

figs = ["plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane","plane"]
figureType = figs[n-1] # type of surface to test on

grid = griding.create(figureType) # initialize grid to test on

timeStep = 20 # set time step
tsp = timeStep/1000.0 # convert it to seconds

# Get and enable devices.
rospy.init_node('python_submarine_controller', anonymous=True) # node is called 'python_webots_controller'

##[Publishers]
imu_pub = rospy.Publisher('imuValues', Vector3, queue_size=1000)
speed_pub = rospy.Publisher("headingSpeed",Vector3,queue_size=10)
velocity_pub = rospy.Publisher('submarineVelocity', Float32, queue_size=10)
path_pub = rospy.Publisher("path", Quaternion, queue_size=1000)
true_pos_pub = rospy.Publisher('true_position', Quaternion, queue_size=1000)
reset_pub = rospy.Publisher("reset",Bool,queue_size=2)
row1_pub = rospy.Publisher("row1",Vector3,queue_size=2)
row2_pub = rospy.Publisher("row2",Vector3,queue_size=2)
row3_pub = rospy.Publisher("row3",Vector3,queue_size=2)
rows = [row1_pub, row2_pub, row3_pub]
RL_pub = rospy.Publisher("ravinL",Bool,queue_size=1)
RR_pub = rospy.Publisher("ravinR",Bool,queue_size=1)
RT_pub = rospy.Publisher("ravinT",Bool,queue_size=1)
RB_pub = rospy.Publisher("ravinB",Bool,queue_size=1)
##![Publishers]

##[Subscriber]
rospy.Subscriber("heading",Vector3,headingCallback)
rospy.Subscriber("finish",Bool,finishCallback)
##![Subscriber]

reset_pub.publish(Bool(True)) # tell to the planners to resset and start over

##[Sensors]
IMUsensor = robot.getInertialUnit('inertial unit')  # front central proximity sensor
IMUsensor.enable(timeStep)

GPSsensor = robot.getGPS('gps')
GPSsensor.enable(timeStep)

GYROsensor = robot.getGyro("gyro")
GYROsensor.enable(timeStep)

##[set robot initial values]
front_left_motor = robot.getMotor("front left thruster")
front_right_motor = robot.getMotor("front right thruster")
rear_left_motor = robot.getMotor("rear left thruster")
rear_right_motor = robot.getMotor("rear right thruster")
front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))
front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)

FL_wheel = robot.getMotor("front left wheel")
FR_wheel = robot.getMotor("front right wheel")
RL_wheel = robot.getMotor("rear left wheel")
RR_wheel = robot.getMotor("rear right wheel")
FL_wheel.setPosition(float('inf'))
FR_wheel.setPosition(float('inf'))
RL_wheel.setPosition(float('inf'))
RR_wheel.setPosition(float('inf'))
FL_wheel.setVelocity(0.0)
FR_wheel.setVelocity(0.0)
RL_wheel.setVelocity(0.0)
RR_wheel.setVelocity(0.0)
##![set robot initial values]

## [Initialize]
robot.step(timeStep) # enable robot step time
xpos, altitude , zpos = GPSsensor.getValues()

true_pos = Quaternion(xpos, altitude, zpos, time)
robot_pose = (xpos, altitude, zpos)
xpos_old = xpos
altitude_old = altitude
zpos_old = zpos

heading_msg = Vector3(0,0,0)
finished = False

x_bound = [-45, 53]
y_bound = [-12, 25]
area = 0
overlap = 0
pose = Quaternion()

thrust = 10000.0/2

iniTime = 0

fileObject = setupFile(n) # start data file

## ![Initialize]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
	## [Read the sensors]
	roll, pitch, heading = IMUsensor.getRollPitchYaw()
	orient = np.array(robot_node.getOrientation())
	orient = orient.reshape(3,3)
	tr = np.transpose(orient)

	xpos, altitude , zpos = GPSsensor.getValues()
	roll_vel, bleh, pitch_vel = GYROsensor.getValues()
	time = robot.getTime()
	## ![Read the sensors]
	
	## [Publish the rotation matrix rows]
	for i in range(len(rows)):
		line = tr[i]
		thing = Vector3(line[0], line[1], line[2])
		rows[i].publish(thing)
	## ![Publish the rotation matrix rows]

	## [Publish true robot position]
	true_pos_list = [xpos, altitude , zpos]
	true_pos = Quaternion(xpos, altitude , zpos, time)
	true_pos_pub.publish(true_pos)
	## ![Publish true robot position]

	## [Publish sensors position]
	robot_pose = addNoise((xpos, altitude , zpos))
	pose = Quaternion(robot_pose[0], robot_pose[1], robot_pose[2], time)
	path_pub.publish(pose)
	
	## [Update area & overlap]
	thisArea, thisOverlap, grid = griding.visit(robot_pose, grid, figureType) # Get the area and overlap of this tick
	area += thisArea
	overlap += thisOverlap
	if (time - iniTime) >= 5: # if it has been some time: write on the file
		visitedPointCB(pose, fileObject, tr, area, overlap, true_pos_list)
	## ![Update area & overlap]
	
	## [Send the boundary detections]
	if xpos >= x_bound[1]:
		RR_pub.publish(Bool(True))
	else:
		RR_pub.publish(Bool(False))

	if xpos <= x_bound[0]:
		RL_pub.publish(Bool(True))
	else:
		RL_pub.publish(Bool(False))

	if altitude >= y_bound[1]:
		RT_pub.publish(Bool(True))
	else:
		RT_pub.publish(Bool(False))

	if altitude <= y_bound[0]:
		RB_pub.publish(Bool(True))
	else:
		RB_pub.publish(Bool(False))
	## ![Send the boundary detections]
	## ![Publish sensors position]

	##[Derivate position for speed]
	xSpeed = (xpos-xpos_old)/tsp
	ySpeed = (altitude-altitude_old)/tsp
	zSpeed = (zpos-zpos_old)/tsp
	##[Derivate position for speed]

	##[Update position]
	xpos_old=xpos
	altitude_old=altitude
	zpos_old=zpos
	##[Update position]

	fl_wheel=0
	fr_wheel=0
	rl_wheel=0
	rr_wheel=0

	radcoeff = 1#180.0/math.pi
	scaling = 1

	imu_pub.publish(Vector3((roll)*radcoeff*scaling, (pitch)*radcoeff*scaling, (heading)*radcoeff*scaling))
	speed_pub.publish(Vector3(math.cos(heading)*xSpeed*-1+math.sin(heading)*zSpeed*-1,ySpeed,math.sin(heading)*xSpeed+math.cos(heading)*zSpeed))

	velocity_pub.publish(bleh)

	##[Control drive] side = 1 is counter-clockwise
	drive, side = drive_side(heading_msg)
	##![Control drive]
	k_d = 1
	k_s = 1

	fl_wheel=k_d*drive+k_s*side
	fr_wheel=k_d*drive-k_s*side
	rl_wheel=k_d*drive+k_s*side
	rr_wheel=k_d*drive-k_s*side

	front_left_motor.setVelocity(-thrust)#positive is up  #0.44467908653
	front_right_motor.setVelocity(thrust)#negative is up #0.44616503673
	rear_left_motor.setVelocity(thrust)#negative is up     #0.42589835641
	rear_right_motor.setVelocity(-thrust)#positive is up  #0.42744959936

	FL_wheel.setVelocity(fl_wheel)
	FR_wheel.setVelocity(fr_wheel)
	RL_wheel.setVelocity(rl_wheel)
	RR_wheel.setVelocity(rr_wheel)

	if finished:
		time = 0
		fileObject.close() # close the data file

		saveMatrix(grid,n) # save the matrix
		
		## [Get nex test number]
		n += 1 # add 1 to the test number
		nFile = open("/home/ros/Documents/my_test_project/refList/number.txt","w")
		nFile.write(str(n))
		nFile.close()
		## ![Get nex test number]
		
		robot.simulationQuit(1) # end siimulation
	pass