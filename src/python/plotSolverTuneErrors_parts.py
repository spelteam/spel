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

def parseErrFile(errFile):

	params=[]
	times=[]

	polygons=[]
	errors=[]
	evals=[]
	frameIDs=[]

	data=""
	with open (errFile, "r") as myfile:
		data=myfile.read().replace('\n', '\t')

	pattern = "\{(.*?)\}"
	paramPattern = "\}(.*?)\{"
	firstParamPattern = "\t(.*?)\{"
	match = re.findall(pattern,data)
	paramMatch = re.findall(paramPattern,data)
	firstParamMatch = re.findall(firstParamPattern,data)[0].split('\t')[-2]
	#print len(match)
	firstParamMatch = firstParamMatch.split()
	params.append(firstParamMatch[0])
	times.append(firstParamMatch[1])

	for p in paramMatch:
		p=p.split()
		params.append(p[0])
		times.append(p[1])

	for m in match:
		frames=m.strip().split('\t') #split by frames

		paramErrors=[] #all errors at this param level
		paramPolygons=[] #all polygons at this param level
		paramEvals=[] #all eval scores at this param level
		paramFrames=[]

		#print frames[0]

		for frame in frames:

			frame = frame.split()
			#print frame
			#frameID evalScore limb1RMS (limb1Poly) limb2RMS (limb2Poly) limb3RMS (limb3Poly) limb4RMS (limb4Poly)... limbKRMS (limbKPoly)

			frameID = frame[0]
			frameEval = frame[1]

			frameErrors = []
			framePolygons = []
			i=2
			while i<len(frame):
				frameErrors.append(frame[i])
				i+=1
				polygon = []
				for k in range(4):
					polygon.append((float(frame[i]), float(frame[i+1])))
					i+=2
				framePolygons.append(polygon)

			#print frameErrors
			paramFrames.append(frameID)
			paramEvals.append(frameEval)
			paramErrors.append(frameErrors)
			paramPolygons.append(framePolygons)
		
		polygons.append(paramPolygons)
		errors.append(paramErrors)
		evals.append(paramEvals)
		frameIDs.append(paramFrames)

	return (params, times, frameIDs, errors, evals, polygons)

parser = argparse.ArgumentParser(description='2 non-optional arguments')

parser.add_argument('ERRIN', action="store")
#parser.add_argument('PART', action="store")
parseResult = parser.parse_args()

data = parseErrFile(parseResult.ERRIN)

#data(params, times, frameIDs, errors, evals, polygons)

params = data[0]
times = data[1]
frameIDs = data[2]
errors = data[3]
evals = data[4]
polygons = data[5]

seqName=parseResult.ERRIN.split('_')[-1].split('.')[0]
solverName=parseResult.ERRIN.split('_')[-2]
paramName=parseResult.ERRIN.split('_')[0].split('/')[-1]

print 'Per-part: '+paramName+', '+solverName+', '+seqName

pcol = [(0.94,0.64,1.0),(0.0,0.46,0.86),(0.6,0.25,0.0),(0.3,0,0.36),(0.1,0.1,0.1),(0,0.36,0.19),(0.17,0.8,0.28),(1,0.8,0.6),
(0.5,0.5,0.5),(0.58,1,0.71),(0.56,0.47,0), (0.6, 0.8, 1.0), (0.25, 0.6, 0.0), (0.86, 0.0, 0.46), (1.0, 1.0, 0.64), (0, 0.5, 1.0), (1.0, 0.5, 0.0)]

fig = plt.figure(figsize=(20, 20), dpi=300) #figsize=(30, 30), dpi=600

  # <BodyPart parentJointId="0" id="0" expectedDistance="0" relativeLength="0.2953" name="Root" childJointId="1" lwRatio="1.9"/>
  # <BodyPart parentJointId="0" id="1" expectedDistance="0" relativeLength="0.0647" name="Left Hip" childJointId="2" lwRatio="3.2"/>
  # <BodyPart parentJointId="0" id="2" expectedDistance="0" relativeLength="0.0647" name="Right Hip" childJointId="3" lwRatio="3.2"/>
  # <BodyPart parentJointId="1" id="3" expectedDistance="0" relativeLength="0.04663" name="Neck" childJointId="4" lwRatio="1.454"/>
  # <BodyPart parentJointId="1" id="4" expectedDistance="0" relativeLength="0.0881" name="Left Clavicle" childJointId="5" lwRatio="3.4"/>
  # <BodyPart parentJointId="1" id="5" expectedDistance="0" relativeLength="0.0881" name="Right Clavicle" childJointId="6" lwRatio="3.4"/>
  # <BodyPart parentJointId="2" id="6" expectedDistance="0" relativeLength="0.2694" name="Left Femur" childJointId="7" lwRatio="3.059"/>
  # <BodyPart parentJointId="3" id="7" expectedDistance="0" relativeLength="0.2694" name="Right Femur" childJointId="8" lwRatio="3.059"/>
  # <BodyPart parentJointId="4" id="8" expectedDistance="0" relativeLength="0.1088" name="Head" childJointId="9" lwRatio="1.125"/>
  # <BodyPart parentJointId="5" id="9" expectedDistance="0" relativeLength="0.1554" name="Left Humerus" childJointId="10" lwRatio="3.0"/>
  # <BodyPart parentJointId="6" id="10" expectedDistance="0" relativeLength="0.1554" name="Right Humerus" childJointId="11" lwRatio="3.0"/>
  # <BodyPart parentJointId="7" id="11" expectedDistance="0" relativeLength="0.2694" name="Left Tibia" childJointId="12" lwRatio="4.727"/>
  # <BodyPart parentJointId="8" id="12" expectedDistance="0" relativeLength="0.2694" name="Right Tibia" childJointId="13" lwRatio="4.727"/>
  # <BodyPart parentJointId="10" id="13" expectedDistance="0" relativeLength="0.1554" name="Left Radius" childJointId="14" lwRatio="3.75"/>
  # <BodyPart parentJointId="11" id="14" expectedDistance="0" relativeLength="0.1554" name="Right Radius" childJointId="15" lwRatio="3.75"/>
  # <BodyPart parentJointId="12" id="15" expectedDistance="0" relativeLength="0.1139" name="Left Metatarsal" childJointId="16" lwRatio="3.5"/>
  # <BodyPart parentJointId="13" id="16" expectedDistance="0" relativeLength="0.1139" name="Right Metatarsal" childJointId="17" lwRatio="3.5"/>

partNames=['Spine', 'Left Hip', 'Right Hip', 'Neck', 'Left Clavicle', 'Right Clavicle', 
'Left Femur', 'Right Femur', 'Head', 'Left Humerus', 'Right Humerus', "Left Tibia", 
'Right Tibia', 'Left Radius', 'Right Radius', 'Left Metatarsal', 'Right Metatarsal']

#plt.title(paramName+' PCP per part for \n'+seqName)
subplotCount=1
for partToPlot in range(17):

	ax = fig.add_subplot(6,3,subplotCount)#, projection='2d')
	ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
	              alpha=0.5)
	ax.xaxis.grid(True, linestyle='-', which='major', color='lightgrey',
	              alpha=0.5)

	for p in range(len(params)): #for each element in params
		

		#there will be a line for each param value
		pFrames=frameIDs[p] #list of frames
		numFrames = len(pFrames)

		partErrorRatios=[]

		#if numFrames>0:
		#print numFrames
		for e in range(numFrames):
			partErrs = errors[p][e]
			numParts = len(partErrs)
			for pe in range(numParts):
				PCL = dist(polygons[p][e][pe][0], polygons[p][e][pe][1])
					#print PCL
				PCE = math.sqrt(float(partErrs[pe]))
				partErrorRatios.append((pe, float(PCE)/float(PCL)))
		#now produce the pcp curve for this param setting
		paramPCP=[]
		for i in range(100):
			#compute the pcp score
			pcpScore=0
			totalParts=0
			for scoreItem in partErrorRatios:
				if scoreItem[0]==partToPlot:
					totalParts+=1
					if scoreItem[1] < 0.01*float((i+1)):
						pcpScore+=1
			paramPCP.append(float(pcpScore)/float(totalParts)*100.0)

		ax.plot(range(1,101), paramPCP, color=pcol[p], alpha=1.0, label=str(params[p]), linewidth=4.0)

	# ax.set_xticklabels([])
	# ax.set_yticklabels([])

	if subplotCount==17: #last part
		plt.rc('legend',**{'fontsize':25})
		handles1, labels1 = ax.get_legend_handles_labels()
		ax.legend(handles1, labels1)
		ax.grid()
		ax.legend(title=paramName, loc='upper left', bbox_to_anchor=(1.1, 1.05),
		          ncol=3, fancybox=True, shadow=True)
		ax.get_legend().get_title().set_fontsize('30')


	ax.set_xlabel('tau %')
	ax.set_ylabel('PCP Score')
	ax.set_title(partNames[partToPlot])

	plt.tick_params(axis='both', which='major')
	plt.tick_params(axis='both', which='minor')

	subplotCount+=1

plt.tight_layout(h_pad=1)
plotSave = parseResult.ERRIN.split('.')[0]+'_part_pcp.png'
fig.savefig(plotSave, bbox_inches='tight')
