#! /usr/bin/env python2.7
from spacepy import pycdf
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
from itertools import tee, izip
from scipy.misc import imread	

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)

def usage():

	print("Author: Mykyta Fastovets / poselib project / 2015")
	print("This utility is a tool for reading and convering 2D joint location GT data from H3.6M dataset into simple text files.")
	print("Example usage: ./h36m.py ~/file.cdf file.txt")

parser = argparse.ArgumentParser(description='1 non-optional argument')

parser.add_argument('IN', action="store")
#parser.add_argument('OUT', action="store")
parseResult = parser.parse_args()

projectName = parseResult.IN.strip().split('/')[-1]
projectName = projectName[:-4]
print projectName

outFile = projectName+"_GT.xml"

fo = open(outFile, 'a')

cdf = pycdf.CDF(parseResult.IN) #read the CDF file

data = cdf['Pose'][0]
#So these are[0,1,2,3,4,5,6,7, 8, 9, 10,11,12,13,14,15,16]
bodyJoints = [0,1,2,3,6,7,8,12,13,14,15,17,18,19,25,26,27]
#bodyJoints = [0,1,2,3]
bodyParts = [[0,7], [0,1], [1,2], [2,3], [0,4], [4,5], [5,6],  [7,8], [8,9], [9,10], [7,11], [11,12], [12,13], [7,14], [14,15], [15,16]]
bodyJointNames = ["Root", "Right Hip", "Right Knee", "Right Ankle", "Left Hip", "Left Knee", "Left Ankle", "Mid Back", 
"Neck Bottom", "Neck Top", "Head Top", "Left Shoulder", "Left Elbow", "Left Wrist", "Right Shoulder", "Right Elbow", "Right Wrist"]
bodyPartNames = ["Root", "Right Hip", "Right Femur", "Right Tibia", "Left Hip", "Left Femur", "Left Tibia",  "Upper Back", #
"Neck", "Head", "Left Clavicle", "Left Humerus", "Left Radius", "Right Clavicle", "Right Humerus", "Right Radius"]

partLwRatio = [1.9, 3.2, 3.059, 4.727, 3.2, 3.059, 4.727, 1.9, 1.454, 1.125, 3.4, 3.0, 3.75, 3.4, 3.0, 3.75]
partRelativeLength = [1.48, 0.0647, 0.2694, 0.2694, 0.0647, 0.2694, 0.2694, 1.48, 0.04663, 0.1088, 0.0881, 0.1554, 0.1554, 0.0881, 0.1554, 0.1554]

#First write the XML header with limb structure
fo.write('<?xml version="1.0"?>\n')
fo.write('<Project name="'+projectName+'" imgFolderPath="../img/'+projectName+'/" maskFolderPath="../mask/'+projectName+'/" camFolderPath="" allowScaling="true"  simMatPath="" exportPath="">\n')
fo.write(' <BodyJoints>\n')

for i in range(17):
	fo.write('  <BodyJoint id="'+str(i)+'" name="'+bodyJointNames[i]+'"/>\n')

fo.write(' </BodyJoints>\n')

fo.write(' <BodyParts>\n')

for i in range(16):
	fo.write('  <BodyPart id="'+str(i)+'" name="'+str(bodyPartNames[i])+'" parentJointId="'+str(bodyParts[i][0])+'" childJointId="'+str(bodyParts[i][1])+
		'" expectedDistance="0" lwRatio="'+str(partLwRatio[i])+'" relativeLength="'+str(partRelativeLength[i])+'"/>\n')

fo.write(' </BodyParts>\n')

fo.write(' <Frames>\n')

frameCounter=1
for frame in data:

	#fig = plt.figure()
	#ax = fig.add_subplot(111)
	
	rawFrameData=[]

	cnt=0
	for x,y in pairwise(frame):
		if cnt%2==0:
			rawFrameData.append([x,y])
		cnt+=1

	#print rawFrameData
	#image = imread('/home/mfastovets/phd/H3.6M/img0001.png')
	#plt.imshow(image, zorder=0);
	



	# for part in bodyParts:
	# 	p0=frameData[part[0]]
	# 	p1=frameData[part[1]]
	# 	ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color='blue', linestyle='-', linewidth=2)

	# ax.scatter([item[0] for item in frameData],[item[1] for item in frameData], color = 'green', marker = 'o', zorder=1)
	# plt.show()

	fo.write('  <Frame id="'+str(frameCounter)+'" imgPath="'+str(frameCounter).zfill(4)+'.png" maskPath="'+str(frameCounter).zfill(4)+
		'.pgm" camPath="" isKeyframe="true" gpX="" gpY="">\n')

	fo.write('   <BodyJoints>\n')
	frameData=[]
	for p in bodyJoints:
		frameData.append(rawFrameData[p])

	for i in range(17):
		fo.write('    <BodyJoint x="'+str(int(frameData[i][0]))+'" y="'+str(int(frameData[i][1]))+'" depthSign="false" id="'+str(i)+'"/>\n')

	fo.write('   </BodyJoints>\n')

	fo.write('   <BodyParts>\n')

	for i in range(16):
		fo.write('    <BodyPart id="'+str(i)+'" isOccluded="false"/>\n')

	fo.write('   </BodyParts>\n')
	fo.write('  </Frame>\n')
	frameCounter+=1
	#break

fo.write(' </Frames>\n')
fo.write('</Project>')