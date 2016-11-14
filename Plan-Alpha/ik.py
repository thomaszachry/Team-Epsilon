#!/usr/bin/python

#RUN FILE BY PROVIDING VALUES FOR X Y: python ik.py x y
import math
import sys

l1 = 12.5 #Length of link 1
l2 = 13.5 #length of link 2



#IK for just the 2 links
def invkin2(x,y):
    """Returns the angles of the first two links
    in the robotic arm as a list.
    returns -> (th1, th2)
    input:
    x - The x coordinate of the effector
    y - The y coordinate of the effector
    angleMode - tells the function to give the angle in
                degrees/radians. Default is degrees
    output:
    th1 - angle of the first link w.r.t ground
    th2 - angle of the second link w.r.t the first link"""


	
    #stuff for calculating th2
    r_2 = x**2 + y**2
    l_sq = l1**2 + l2**2
    term2 = (r_2 - l_sq)/(2*l1*l2)
    term1 = ((1 - term2**2)**0.5)*-1
    #calculate th2
    th2 = math.atan2(term1, term2)
    #optional line.
    th2 = -1*th2

    #Stuff for calculating th2
    k1 = l1 + l2*math.cos(th2)
    k2 = l2*math.sin(th2)
    r  = (k1**2 + k2**2)**0.5
    gamma = math.atan2(k2,k1)
    #calculate th1
    th1 = math.atan2(y,x) - gamma

    return math.degrees(th1), math.degrees(th2)


if __name__ == "__main__":
    print invkin2(float(sys.argv[1]),float(sys.argv[2])) #x,y

