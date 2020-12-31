# -*- coding: utf-8 -*-
"""
Created on Mon Nov 30 12:24:26 2020

@author: inaki
"""

import math

## [Declare]

# ## [Boundaries]
x_bound = [-47, 54]
y_bound = [-14, 27]
# ## ![Boundaries]

## ![Declare]

def create(figType):
	visited = [] # Matrix that will be returned
	xRng = x_bound[1] - x_bound[0] # x range
	yRng = y_bound[1] - y_bound[0] # y range
	if figType == "plane": # if the implementation is on a plane
		xstps = int(xRng//.5 + (0 if (xRng%.5 == 0) else 1)) # calculate the number of cells on the x direction
		ystps = int(yRng//.5 + (0 if (yRng%.5 == 0) else 1)) # calculate the number of cells on the y direction
		for i in range(xstps): # for all cells in x
			temp = [] # create a temporal variable
			for j in range(ystps): # go through all y cells
				temp.append([0, 0]) # start cell with [visited, number of times overlapped]
			visited.append(temp) # after finishing a row add it to the final matrix
	
	## [In case it was going to be implemented a cylinder]
	elif figType == "cylinder":
		# r = 15
		angleIncrement = 1/30 # .5/15 = .5/r
		col = int(2*math.pi//angleIncrement + (0 if (2*math.pi%angleIncrement == 0) else 1))
		xstps = int(xRng//.5 + (0 if (xRng%.5 == 0) else 1))
		for i in range(xstps):
			temp = []
			for j in range(col):
				temp.append([0,0])
			visited.append(temp)
	## ![In case it was going to be implemented a cylinder]
	else:
		pass
	
	return visited

def visit(point, mat, figType):
	area = 0 # set the new area
	overlap = 0 # and overlapment to 0
	if point[0] >= x_bound[0] and point[0] <= x_bound[1] and point[1] >= y_bound[0] and point[1] <= y_bound[1]: # if the robot is inbounds
		if figType == "plane": # if the figure is a plane
			xscal = point[0] - x_bound[0] # convert the point relative to the boundaries
			yscal = point[1] - y_bound[0]
			pos = (int(xscal//.5), int(yscal//.5 )) # get the cell position
			
			if mat[pos[0]][pos[1]][0] == 1: # if the cell had been visited
				mat[pos[0]][pos[1]][1] += 1 # add an overlapment
				overlap = .5**2 # and set the overlap area to .5^2
			else: # if it's the first time visiting it
				mat[pos[0]][pos[1]][0] = 1 # set the cell as visited
				area = .5**2 # and ad area
		if figType == "cylinder":
			angleIncrement = 1/30 # .5/15 = .5/r
			xscal = point[0] - x_bound[0]
			angle = math.atan(point[2]/point[1])
			pos = (int(xscal//.5), int(angle//(1/30)))
			
			if mat[pos[0]][pos[1]][0] == 1:
				mat[pos[0]][pos[1]][1] += 1
				overlap = math.pi*angleIncrement*.5
			else:
				mat[pos[0]][pos[1]][0] = 1
				area = math.pi*angleIncrement*.5
		else:
			pass
		
	return area, overlap, mat