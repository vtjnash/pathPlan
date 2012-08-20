#!/usr/bin/env python
import os
import random
import math

environment_name = raw_input("Environment name> ")
boxX = float(raw_input("Box X> "))
boxY = float(raw_input("Box Y> "))
boxL = float(raw_input("Box L> "))
boxW = float(raw_input("Box W> "))
botSize = float(raw_input("Bot Size R> "))
goalSize = float(raw_input("Goal Size R> "))
nobs = int(raw_input("Number of Obstacles> "))
muobs = float(raw_input("Mean size of Obstacles> "))
sigmaobs = float(raw_input("Std dev. of Obstacles> "))

try:
	os.mkdir(environment_name)
except:
	pass
os.chdir(environment_name)
with file('log.txt','w') as f:
	f.write('%s\n%f\n%f\n%f\n%f\n%f\n%f\n%d\n%f\n%f' %(environment_name,
	boxX, boxY, boxL, boxW, botSize, goalSize, nobs, muobs, sigmaobs))

X1 = boxX - boxL/2
X2 = boxX + boxL/2
Y1 = boxY - boxW/2
Y2 = boxY + boxW/2
with file("box.txt",'w') as f:
	f.write("%f, %f, %f, %f\n" %(boxX, boxY, boxW, boxL))

botX = random.uniform(X1+botSize, X2-botSize)
botY = random.uniform(Y1+botSize, Y2-botSize)
with file("init.txt",'w') as f:
	f.write("%f, %f, %f\n" %(botX, botY, botSize))

goalX = random.uniform(X1+goalSize, X2-goalSize)
goalY = random.uniform(Y1+goalSize, Y2-goalSize)
with file("goal.txt",'w') as f:
	f.write("%f, %f, %f\n" %(goalX, goalY, goalSize))

with file("obstacles.txt",'w') as f:
	for i in xrange(nobs):
		while True:
			obSize = abs(random.gauss(muobs, sigmaobs))
			x = random.uniform(X1+obSize, X2-obSize)
			y = random.uniform(Y1+obSize, Y2-obSize)
			if (math.hypot(x-goalX,y-goalY) > goalSize + obSize) and \
			   (math.hypot(x-botX,y-botY) > botSize + obSize):
				f.write("%f, %f, %f\n" %(x,y,obSize))
				break


