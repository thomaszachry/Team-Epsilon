#!/usr/bin/env python

from openravepy import *
import numpy, time

if __name__ == "__main__":
    env = Environment()
    env.Load('env.xml')
    env.SetViewer('qtcoin')
    viewer = env.GetViewer()
    viewer.SetBkgndColor([.8, .85, .9])  # RGB tuple
    robot = env.GetRobots()[0]



import IPython
IPython.embed()

