#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import ximureceiver, quaternion
import serial
import numpy as np
import math
import sys


def quatern2rotMat(q):
    R = np.zeros((3,3))
    R[0][0] = 2*math.pow(q[0],2) - 1 + 2*math.pow(q[1],2)
    R[0][1] = 2*(q[1]*q[2] + q[0]*q[3])
    R[0][2] = 2*(q[1]*q[3] - q[0]*q[2])
    R[1][0] = 2*(q[1]*q[2] - q[0]*q[3])
    R[1][1] = 2*math.pow(q[0],2) - 1 + 2*math.pow(q[2],2)
    R[1][2] = 2*(q[2]*q[3] + q[0]*q[1])
    R[2][0] = 2*(q[1]*q[3] + q[0]*q[2])
    R[2][1] = 2*(q[2]*q[3] - q[0]*q[1])
    R[2][2] = 2*math.pow(q[0],2) - 1 + 2*math.pow(q[3],2)
    return R

def talker(str,filename):
    ximu = ximureceiver.XimuReceiver()
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    s=serial.Serial(str, 115200)
    hello_str = "roll:0,pitch:0,yaw:0"
    linVel = np.zeros((3))
    linPos = np.zeros((3))
    acc = np.zeros((3))
    mag = np.zeros((3))
    gyr = np.zeros((3))
    if filename != "ok":
        fr = open(filename,"w")
    while not rospy.is_shutdown():
        #while True:
        ximu.processNewChar(ord(s.read())%256)

        if ximu.isInertialAndMagGetReady():
            InertialAndMag = ximu.getInertialAndMag()
            acc[0] = InertialAndMag.accX
            acc[1] = InertialAndMag.accY
            acc[2] = InertialAndMag.accZ

            
            mag[0] = InertialAndMag.magX
            mag[1] = InertialAndMag.magY
            mag[2] = InertialAndMag.magZ

            gyr[0] = InertialAndMag.gyrX
            gyr[1] = InertialAndMag.gyrY
            gyr[2] = InertialAndMag.gyrZ


        if ximu.isQuaternionGetReady():
            Quaternions = ximu.getQuaternion()
            Quaternion = quaternion.Quaternion(Quaternions.w, Quaternions.x, Quaternions.y, Quaternions.z)
            Quater = np.array([Quaternions.w, Quaternions.x, Quaternions.y, Quaternions.z])
            EulerAngles = Quaternion.getEulerAngles()
                #print "roll = ", EulerAngles.roll
                #print "pitch = ", EulerAngles.pitch
                #print "yaw = ", EulerAngles.yaw
            hello_str = "roll:%s,pitch:%s,yaw:%s" %(EulerAngles.roll,EulerAngles.pitch,EulerAngles.yaw)
            write_str = "%s,%s,%s\r\n" %(EulerAngles.roll,EulerAngles.pitch,EulerAngles.yaw)
            ##
            R = quatern2rotMat(Quater)
            tcAcc = np.dot(R, acc)

            linAcc = tcAcc - np.array([0,0,0.919])
            print "a:", linAcc[0]," b:", linAcc[1]," c:", linAcc[2]

            linAcc = linAcc * 9.81
            linVel = linVel + linAcc/256.0
            tmp = math.pow(linAcc[0],2) + math.pow(linAcc[1],2) +math.pow(linAcc[2],2)
            print tmp
            if tmp<2:
                linVel = np.zeros((3))
                linAcc = np.zeros((3))

            linPos = linPos + linVel/256.0

            hello_str = "%s,%s,%s" %(linPos[0],linPos[1],linPos[2])
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            if filename != "ok":
                fr.write(write_str)
        #rate.sleep()

if __name__ == '__main__':
    try:

        print "Open port:%s" %(sys.argv[1])
        if len(sys.argv)==2:
            talker(sys.argv[1],"ok")
        if len(sys.argv)==3:
            talker(sys.argv[1],sys.argv[2])
    except rospy.ROSInterruptException:
        pass
