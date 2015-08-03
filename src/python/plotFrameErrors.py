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

def usage():

	print("Author: Mykyta Fastovets / poselib project / 2015")
	print("This utility is an analysis tool for plotting error files generated by the poselib tuners.")
	print("Input should be a .err file.")
	print("Example usage: ./plotSimVsTemp.py ~/file.err ")

def dist(a,b):

	return math.sqrt((a[0]-b[0])**2+(a[1] - b[1])**2)

parser = argparse.ArgumentParser(description='1 non-optional argument')

parser.add_argument('ERRIN', action="store")
parser.add_argument('FRAMENUM', action="store")
parseResult = parser.parse_args()

#DATADIR contains that folder that contains all other data
errFile = parseResult.ERRIN

data = [line.strip().split() for line in open(errFile)] #read the data from the int file
firstLine = data.pop(0) #pop the first line off the data stack

numParams = int(firstLine.pop(0)) #the number of parameters in the file

pfRMS=[]
for dataItem in data: #data now contains everything but the first line
	frameID = int(dataItem[0])

	params = [float(x) for x in dataItem[1:numParams+1]]

	#rest of the items are bodypart errors
	partErrors=[float(x) for x in dataItem[numParams+1:len(dataItem)]] 

	if frameID==int(parseResult.FRAMENUM): #if we're looking at the right frame
		rmsErr=0 #compute RMS error for this frame
		for error in partErrors: #for each number
			rmsErr+=math.sqrt(float(error))
		rmsErr = math.sqrt(rmsErr/float(len(partErrors)))
		
		pfRMS.append([params, rmsErr])

fig = plt.figure()

ax = fig.add_subplot(211)
ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
              alpha=0.5)
ax.xaxis.grid(True, linestyle='-', which='major', color='lightgrey',
              alpha=0.5)


x=[]
y=[]
index=0
labels=[]
for e in pfRMS:
	#print e
	x.append(index)
	y.append(e[1])
	labels.append(str(e[0]))
	index+=1
#ax.bar(x,y)
ax.scatter(x, y, marker='o', s=40, color='green', alpha=0.3)
ax.plot(x, y, color='green', lw=6, alpha=0.3)

#plt.xticks( x + 0.5,  labels)

ax.set_ylabel('RMS Error (pix)', fontsize=30)

plt.show()