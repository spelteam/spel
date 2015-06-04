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

parser = argparse.ArgumentParser(description='2 non-optional argument')

parser.add_argument('IN', action="store")
parser.add_argument('OUT', action="store")
parseResult = parser.parse_args()

outFile = parseResult.OUT

cdf = pycdf.CDF(parseResult.IN) #read the CDF file

data = cdf['Pose'][0]

#bodyParts = [1, 2, 3, 4, 7, 8, 9, 13, 14, 15, 16, 18, 19, 20, 26, 27, 28]
bodyParts = [0,1,2,3,6,7,8,12,13,14,15,17,18,19,25,26,27]

for frame in data:

	fig = plt.figure()
	ax = fig.add_subplot(111)
	
	rawFrameData=[]
	
	cnt=0
	for x,y in pairwise(frame):
		if cnt%2==0:
			rawFrameData.append([x,y])
		cnt+=1

	#print rawFrameData
	image = imread('/home/mfastovets/phd/H3.6M/img0001.png')
	plt.imshow(image, zorder=0);
	

	frameData=[]
	for p in bodyParts:
		frameData.append(rawFrameData[p])


	ax.scatter([item[0] for item in frameData],[item[1] for item in frameData], color = 'green', marker = 'o', zorder=1)
	plt.show()
	break