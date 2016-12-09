#!/usr/bin/env python

from openravepy import *
import numpy, time


env = Environment()
env.Load('re.robot.xml')
env.SetViewer('qtcoin')
viewer = env.GetViewer()
viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
robot = env.GetRobots()[0]


#manipprob = interfaces.BaseManipulation(robot) 
#traj=manipprob.MoveManipulator(goal=[-0.75,1.24,],outputtrajobj=True,execute=False)
#spec=traj.GetConfigurationSpecification()
#for i in range(5):
#    starttime = time.time()
#    while time.time()-starttime < traj.GetDuration():
#        curtime = time.time()-starttime
#        with env: # have to lock environment since accessing robot
#            trajdata=traj.Sample(curtime)
#            values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
#            robot.SetDOFValues(values)
#        time.sleep(0.01)



def reset():
	env.Reset()
	env.Load('re.robot.xml')
 
import IPython
IPython.embed()

