#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
from geometry_msgs.msg import Twist

class CmdTwist(object):
    _cmdMsg=''
    _duration=0
    _startTime=0 # in second
    _stopTime=0 # in second


    def __init__(self,cmdMsg):
        self._cmdMsg=cmdMsg
        self._startTime=time.time()
    
    def setStopTime(self,stopTime):
        self._stopTime=stopTime
        self._duration=stopTime-self._startTime

    def duration(self):
       return self._duration

    def getTwistCmd(self):
        return self._cmdMsg

    def reverse(self):
        r_cmd=Twist()
        r_cmd.linear.x=self._cmdMsg.linear.x * -1
        r_cmd.linear.y=self._cmdMsg.linear.y * -1
        r_cmd.linear.z=self._cmdMsg.linear.z * -1
        r_cmd.angular.x=self._cmdMsg.angular.x * -1
        r_cmd.angular.y=self._cmdMsg.angular.y * -1
        r_cmd.angular.z=self._cmdMsg.angular.z * -1
        return r_cmd


    