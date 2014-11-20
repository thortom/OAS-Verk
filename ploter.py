import numpy as np
import matplotlib.pyplot as plt
import os, sys

def plotBottomAndSonar(sonarRange, array, auv_posX, auv_posY, title = None):
	size = array.shape[0]

	x = 0
	depth = 1
	if (title != None):
		x = title.index('x_pos')
		depth = title.index('waterDepth')
	auv_posX = int(round(auv_posX))
	auv_posY = int(round(auv_posY))

	stepSize = array[size/2.0, x] - array[size/2.0 - 1, x]						# Take some random step in middle of data to determine the step size
	sonarRange = int(round((sonarRange*1.0)/stepSize))
	auv_posX = int(round((auv_posX*1.0)/stepSize))

	y1 = []
	y2 = []
	y3 = []
	y4 = []
	turnBottom = (array[size/2.0, depth] > -1)							        # Check random depth value if positive then turn bottom (make depth value negative).
	for i in range(size):
		y1.append(-(4.35779)/(49.8097)*array[i,x])
		y2.append(-(1.45424)/(49.9788)*array[i,x])
		y3.append((1.45424)/(49.9788)*array[i,x])
		y4.append((4.35779)/(49.8097)*array[i,x])

		# Mirror bottom for -depth and move sonar for auv_pos
		if turnBottom:
			array[i,depth] = -array[i,depth]
		y1[i] = auv_posY + y1[i]
		y2[i] = auv_posY + y2[i]
		y3[i] = auv_posY + y3[i]
		y4[i] = auv_posY + y4[i]

	# Fix range if needed
	if (auv_posX + sonarRange >= size):
		sonarRange = size - auv_posX
	
	plt.figure()
	plt.plot(array[:,x], array[:,depth])
	plt.plot(array[auv_posX:(auv_posX + sonarRange),x], y1[0:sonarRange])
	plt.plot(array[auv_posX:(auv_posX + sonarRange),x], y2[0:sonarRange])
	plt.plot(array[auv_posX:(auv_posX + sonarRange),x], y3[0:sonarRange])
	plt.plot(array[auv_posX:(auv_posX + sonarRange),x], y4[0:sonarRange])
	#plt.axes().set_aspect('equal')
	plt.show()
	return

def movieBottomAndSonar(sonarRange, array, title, resolution):
	x = title.index('x_pos')
	altitude = title.index('depth')
	size = array.shape[0]
	for i in range(size):
		if (i%int(round(size/resolution)) == 0):
			auv_posX = array[i,x]
			auv_posY = -array[i,altitude]
			plotBottomAndSonar(sonarRange, array, auv_posX, auv_posY, title)
		

def readAndPlot(array, title, count = None):
	size = array.shape[0]
	if (count == None):
		count = array.shape[1]

	x = title.index('x_pos')
	for i in range(count):
		if (i == 0):
			continue
		fig = plt.figure()
		plt.plot(array[1:size, x], array[1:size, i])
		plt.xlabel(title[x])
		plt.ylabel(title[i])
		plt.title(dataFile)
		plt.show()
	return

def readData(dataFile):
	array = np.loadtxt(dataFile, skiprows=1, delimiter=" ")
	strings = np.loadtxt(dataFile, dtype=str, delimiter=" ")
	title = []
	for i in range(strings.shape[1]):
		title.append(strings[0,i])
	return array, title

def readAndSubplot(array, title, count = None):
	size = array.shape[0]
	if (count == None):
		count = array.shape[1]

	x = title.index('x_pos')
	fig = plt.figure()
	for i in range(count):
		if (i == 0):
			continue
		plt.plot(array[1:size, x], array[1:size, i], label=title[i])
	plt.xlabel(title[x])
	plt.title(dataFile)
	plt.legend(loc='upper center')
	plt.show()
	return

if __name__ == "__main__":
	sonarRange = 20
	auv_posX = 0
	auv_posY = 0
	bottomFile = "../../../../debug/crew/Pilots/ObstacleAvoidancePilot/bottom.txt"
	dataFile = "example.txt"
	array, title = readData(dataFile)		# Loses the first point, assumes that the first point is the data name

	case = 0
	if len(sys.argv) == 2:
	    case = int(sys.argv[1])
	else:
		readAndSubplot(array, title)
		readAndPlot(array, title)
		plotBottomAndSonar(sonarRange, array, auv_posX, auv_posY, title)
		movieBottomAndSonar(sonarRange, array, title, 5.0)

	if case == 1:
		readAndSubplot(array, title)
	elif case == 2:
		readAndPlot(array, title)
	elif case == 3:
		plotBottomAndSonar(sonarRange, array, auv_posX, auv_posY, title)
	elif case == 4:
		movieBottomAndSonar(sonarRange, array, title, 10.0)



