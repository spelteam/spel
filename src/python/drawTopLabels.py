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
from os import listdir
from os.path import isfile, join
import re
import random
from matplotlib.patches import Polygon
from PIL import Image

def usage():

	print("Author: Mykyta Fastovets / poselib project / 2015")
	print("This utility is an analysis tool for plotting error files generated by the poselib tuners.")
	print("Input should be a .err file.")
	print("Example usage: ./plotSimVsTemp.py ~/file.err ")

def dist(a,b):

	return math.sqrt((a[0]-b[0])**2+(a[1] - b[1])**2)

def SaveFigureAsImage(fileName,fig=None,**kwargs):
    ''' Save a Matplotlib figure as an image without borders or frames.
       Args:
            fileName (str): String that ends in .png etc.
 
            fig (Matplotlib figure instance): figure you want to save as the image
        Keyword Args:
            orig_size (tuple): width, height of the original image used to maintain 
            aspect ratio.
    '''
    fig_size = fig.get_size_inches()
    w,h = fig_size[0], fig_size[1]
    fig.patch.set_alpha(0)
    if kwargs.has_key('orig_size'): # Aspect ratio scaling if required
        w,h = kwargs['orig_size']
        w2,h2 = fig_size[0],fig_size[1]
        fig.set_size_inches([(w2/w)*w,(w2/w)*h])
        fig.set_dpi((w2/w)*fig.get_dpi())
    a=fig.gca()
    a.set_frame_on(False)
    a.set_xticks([]); a.set_yticks([])
    plt.axis('off')
    plt.xlim(0,w); plt.ylim(h,0)
    fig.savefig(fileName, transparent=True, bbox_inches='tight', \
                        pad_inches=0)



parser = argparse.ArgumentParser(description='1 non-optional argument')

parser.add_argument('ERRIN', action="store")
parser.add_argument('PARAM_INDEX', action="store")
parser.add_argument('IMAGE_DIR', action="store")
parseResult = parser.parse_args()

#DATADIR contains that folder that contains all other data
errFile = parseResult.ERRIN

plotTitle = errFile.split('_')[-1].split('.')[0]

print 'Reading '+errFile

myFile = open(errFile) 
data = [line.strip().split() for line in open(errFile)] #read the data from the int file

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

#Create plottable objects, with in x,y format and define the graphs that will be produced

numParams = len(result)

#Detector quality graphs, to measure correspondence of score to RMS error, there will be a line for each param
numTopLables=100 #define number of labels from the top scorers to take
percentTopLabels=10 #define the percentage of labels to analyse more deeply from the top

topErrorsP = [] #percent errors
topErrorsA = [] #absolute errors

#Param evaluation graphs, to measure how each parameter setting performs
percentileParamErrors=[]

#print result
nParts = 0


tme=[] #top scoring #1 ranked label ERROR, for each param, we want this to be low
tms=[] #top scoring #1 ranked label SCORE, for each param, we want this to be low

tmi=[] #lowest error label index within percentTopLabels, for each param, we want this to be low
tmig=[] #lowest error label index GLOBALLY, for each param, we want this to be low
tmigp=[]

nal=[] #number of acceptable labels within percentTopLabels, we want this to be high, accept thresh defined in detectorTuner

Polygons=[]
for i in range(numParams):

	numFrames = len(result[i][1])

	paramPolygons=[]

	for j in range(numFrames):

		framePolygons=[]

		numParts = len(result[i][1][j][1])

		for k in range(numParts):

			partID = result[i][1][j][1][k][0]

			errVal = float(result[i][1][j][1][k][1][0][2]) #finalScore (support)
			scoreVal = float(result[i][1][j][1][k][1][0][1]) #partError[e] (err to GT)
			acceptVal = float(result[i][1][j][1][k][1][0][3]) #isAccepted (is this close enough?)

			polygon=[(float(result[i][1][j][1][k][1][0][7]), float(result[i][1][j][1][k][1][0][8])),
			(float(result[i][1][j][1][k][1][0][9]), float(result[i][1][j][1][k][1][0][10])),
			(float(result[i][1][j][1][k][1][0][11]), float(result[i][1][j][1][k][1][0][12])),
			(float(result[i][1][j][1][k][1][0][13]), float(result[i][1][j][1][k][1][0][14]))]

			framePolygons.append([errVal, polygon])
		paramPolygons.append(framePolygons)
	Polygons.append(paramPolygons)

paramIndex=parseResult.PARAM_INDEX
imageDir=parseResult.IMAGE_DIR


onlyfiles = [ f for f in listdir(imageDir) if isfile(join(imageDir,f)) ]

#pad files if needed
for i in range(len(onlyfiles)):
	filename = onlyfiles[i]
	num, end = filename.split('.')
	if end=='png':
		num = num.zfill(4)
	 	new_filename = num + ".png"
	 	os.rename(os.path.join(imageDir+'/', filename), os.path.join(imageDir+'/', new_filename)) #rename

onlyfiles = sorted([ f for f in listdir(imageDir) if isfile(join(imageDir,f)) ]) #re-read


if not os.path.exists(imageDir+'_det/'):
	os.mkdir(imageDir+'_det/')

frameCounter=0

#print Polygons[int(paramIndex)]
for frame in Polygons[int(paramIndex)]:
	
	image = Image.open(imageDir+'/'+onlyfiles[frameCounter]) #read image
	
	(width, height) = image.size

	#print width, height
	
	figSize=(width, height)
	fig = plt.figure()

	fig = plt.gcf()
	DPI = fig.get_dpi()
	fig.set_size_inches(width/float(DPI),height/float(DPI))

	fig = plt.imshow(image)

	for part in frame:
		 partPoly = np.asarray(part[1])
		 polyArtist = Polygon(partPoly, alpha=0.5, facecolor='green')
		 plt.gca().add_artist(polyArtist)
	
	# a=plt.gca()
	# a.set_frame_on(False)
	# a.set_xticks([]); a.set_yticks([])
	# plt.axis('off')
	# plt.xlim(0,width); plt.ylim(height,0)
	# plt.axis('off')

	#fig.axes.get_xaxis().set_visible(False)
	#fig.axes.get_yaxis().set_visible(False)
	
	#plt.savefig(imageDir+'_det/'+'/det_'+onlyfiles[frameCounter], bbox_inches='tight', pad_inches = 0, figsize=figSize, dpi=100)
	SaveFigureAsImage(imageDir+'_det/'+'/det_'+onlyfiles[frameCounter],plt.gcf(), orig_size=(width,height) )

	plt.clf()
	frameCounter+=1 #augment frame counter
