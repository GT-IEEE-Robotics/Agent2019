# -*- coding: utf-8 -*-
from numpy import arccos
import math


def rotation(tup,theta):
    X = math.cos(theta)*tup[0]-math.sin(theta)*tup[1]
    Y = math.sin(theta)*tup[0]+math.cos(theta)*tup[1]
    return(X,Y)
    

def findangle(vectorlist,imu):
    vectorA = vectorlist[0]
    vectorB = vectorlist[1]
    magA = math.sqrt(vectorA[0]**2+vectorA[1]**2)
    magB = math.sqrt(vectorB[0]**2+vectorB[1]**2)
    dot = vectorA[0]*vectorB[0] + vectorA[1]*vectorB[1]
    Cos = dot/(magA*magB)
    theta = arccos(Cos)
    return "theta is rad:{} or deg:{}".format(theta,theta*(180/math.pi))

def findcoordinates(Displacement,Vprime,IMU):
    #change v' to normal coordinates
    V= rotation(Vprime,IMU)
    #print(V)
    #now add displacement and V together
    D = Displacement 
    U1 = D[0] + V[0]
    U2 = D[1] + V[1]
    U = (U1,U2)
    return U


imu = 3*math.pi/4
displacement = (6,6)
vprime = (-1,1)
print(findcoordinates(displacement,vprime,imu))