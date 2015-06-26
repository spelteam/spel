#! /usr/bin/env python2.7

import glob
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import argparse
import numpy as np
import matplotlib.image as mpimg
from matplotlib.lines import Line2D
from pylab import figure, show
import math
import os
import re
import random

def usage():

	print("Author: Mykyta Fastovets / poselib project / 2015")
	print("This utility is an analysis tool for plotting error files generated by the poselib tuners.")
	print("Input should be a .err file.")
	print("Example usage: ./plotSimVsTemp.py ~/file.err ")

def dist(a,b):

	return math.sqrt((a[0]-b[0])**2+(a[1] - b[1])**2)

parser = argparse.ArgumentParser(description='1 non-optional argument')

parser.add_argument('ERRIN', action="store")
parseResult = parser.parse_args()

#DATADIR contains that folder that contains all other data
errFile = parseResult.ERRIN

myFile = open(errFile) 
data = [line.strip().split() for line in open(errFile)] #read the data from the int file
#result = data.split()
#re.findall('\{[\S\W]*\}', data)
#firstLine = data.pop(0) #pop the first line off the data stack

#numParams = int(firstLine.pop(0)) #the number of parameters in the file

frameIndex=-1
partIndex=-1
paramValue=-1

itemIndex=0
readMode = 'false'
result=[]

	
paramData=[]
frameData=[]
partData=[]

for dataItem in data:


	if dataItem[0]=='{':
		readMode='true'
		paramValue=data[itemIndex-1][0]

	elif dataItem[0]=='}':
		result.append([paramValue, paramData])
		paramData=[]

	elif dataItem[0]=='[':
		frameIndex=data[itemIndex-1][0]

	elif dataItem[0]==']':
		paramData.append([frameIndex, frameData])
		frameData=[]

	elif dataItem[0]=='(':
		partIndex=data[itemIndex-1][0]

	elif dataItem[0]==')':
		frameData.append([partIndex, partData])
		partData=[]

	elif len(dataItem)>1 and readMode=='true':
		#print dataItem
		#raw_input('test')
		partData.append(dataItem)
		# print 'ITEM '+ str(dataItem[0])
		# print dataItem
		# print 'PART '+str(partIndex)
		# print partData
		# print 'FRAME '+str(frameIndex)
		# print frameData
		# print 'PARAM '+str(paramValue)
		# print paramData
		# print 'RESULT '
		# print result
		#raw_input('test')
		
	itemIndex+=1

fig = plt.figure()


ax = fig.add_subplot(211)#, projection='2d')
ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
              alpha=0.5)
ax.xaxis.grid(True, linestyle='-', which='major', color='lightgrey',
              alpha=0.5)

dx = fig.add_subplot(212)#, projection='3d')
dx.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
              alpha=0.5)
dx.xaxis.grid(True, linestyle='-', which='major', color='lightgrey',
              alpha=0.5)

# #for paramVal in result:
# for frame in result[0][1]:
# 	#print frame
numParams = len(result)
for i in range(numParams):
	
	pcol = "#%06x" % random.randint(0,0xFFFFFF)
	numFrames = len(result[i][1])

	#print 'NUM FRAMES:'
	#print numFrames

	x = []
	y = []
	z = []
	z2 = []

	for j in range(numFrames):


		numParts = len(result[i][1][j][1])
		
		#print 'NUM PARTS'
		#print numParts
		avgMinIndex=0.0
		rmsError=0.0
		for k in range(numParts):
			
			numLabels=len(result[i][1][j][1][k][1])

			#print numLabels
			#raw_input('test')
			#print 'NUM LABELS'
			#print numLabels
			col = "#%06x" % random.randint(0,0xFFFFFF)
			minError=1000000000
			minIndex=-1.0
			topError=float(result[i][1][j][1][k][1][0][2])
			
			for l in range(numLabels):
				
				if float(result[i][1][j][1][k][1][l][2]) < minError:
					minError = float(result[i][1][j][1][k][1][l][2])
					minIndex = int(result[i][1][j][1][k][1][l][0])

			rmsError+=topError
			avgMinIndex+=minIndex
			
		avgMinIndex=float(avgMinIndex)/float(numParts)
		rmsError=math.sqrt(float(rmsError)/float(numParts))

		print avgMinIndex
		print rmsError

		#print int(result[i][1][j][1][k][1][l][0])
		x.append(float(result[i][0]))
		y.append(int(result[i][1][j][0]))
		z.append(avgMinIndex)
		z2.append(rmsError)

		#ax.scatter(float(result[i][0]), int(result[i][1][j][0]), avgMinIndex, marker='o', s=15, color=pcol, alpha=1.0) #draw min ranks
		ax.scatter(int(result[i][1][j][0]), avgMinIndex, marker='o', s=15, color=pcol, alpha=1.0) #draw min ranks
		dx.scatter(int(result[i][1][j][0]), rmsError, marker='o', s=15, color=pcol, alpha=1.0) #draw min ranks
	#ax.plot(x, y, z, color=pcol, alpha=1.0) #draw min ranks
	ax.plot(y, z, color=pcol, alpha=1.0, label=str(result[i][0])) #draw min ranks
	dx.plot(y, z2, color=pcol, alpha=1.0, label=str(result[i][0])) #draw min ranks


dx.set_xlabel('Frame Number', fontsize=35)
dx.set_ylabel('RMS Error (pixels)', fontsize=35)
#dx.set_zlabel('Detector Score', fontsize=35)

ax.set_xlabel('Frame Number', fontsize=35)
ax.set_ylabel('Avg. Min Index', fontsize=35)
#ax.set_zlabel('Avg Min Index', fontsize=35)

handles, labels = ax.get_legend_handles_labels()
ax.legend(handles, labels)
ax.grid()
ax.legend()



plt.show()