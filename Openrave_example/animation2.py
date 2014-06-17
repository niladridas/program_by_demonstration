from openravepy import *
import time
import csv

f =open('myLog.csv', 'rb')
reader = csv.reader(f)

xc = list(reader)
datavalues = []
for i in xrange(len(xc)):
	datavalues.append(map(float,xc[i][1:]))


env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('data/lab1.env.xml') # load a simple scene
robot = env.GetRobots()[0] # get the first robot

robot.SetDOFValues(datavalues[0],[0,1,2,3,4,5,6])

for i in xrange(len(datavalues)):
	 	robot.SetDOFValues(datavalues[i],[0,1,2,3,4,5,6])
     		time.sleep(0.02)

