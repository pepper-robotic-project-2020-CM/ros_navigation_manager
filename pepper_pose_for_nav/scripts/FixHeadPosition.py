#!/usr/bin/env python  

import qi
import argparse
import sys
import time
import rospy
from pepper_pose_for_nav.srv import FixHeadAtPosition

class HeadFix():


    def __init__(self,session):
        """
        This example uses the setExternalCollisionProtectionEnabled method.
        """
        # Get the service ALMotion.
        self._motion_service  = session.service("ALMotion")
        self._memory_service = session.service("ALMemory")
        self._autolife_service = session.service("ALAutonomousLife")
        self._autolife_service.setState('disabled')
        self._posture_service = session.service("ALRobotPosture")
        self._posture_service.goToPosture("Stand",0.3)


        self._fractionMaxSpeed = 0.2
        self._motion_service.setStiffnesses("HEAD", 1.0)
        self._motion_service.setStiffnesses("TORSO", 1.0)
        self._error=0.1
        #pitch_value=0.3
        self._pitch_value=0.0
        self._isEnd=False
        #declare ros service 
        self.setHeadPositionSrv = rospy.Service('fix_head_pose_srv', FixHeadAtPosition, self.setHeadPositionSrvCallback)


    def fixHead(self):
        while not rospy.is_shutdown():
            headYawPos = self._memory_service.getData("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
            headPitchPos = self._memory_service.getData("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
            print("headYawPos:"+str(headYawPos)+",headPitchPos:"+str(headPitchPos))
            #if abs(headPitchPos)>error or abs(headYawPos)>error:
            if headPitchPos>(self._pitch_value+self._error) or headPitchPos<(self._pitch_value-self._error)  or abs(headYawPos)>self._error:
                self._motion_service.setAngles("HeadYaw", 0.0, self._fractionMaxSpeed) 
                self._motion_service.setAngles("HeadPitch", self._pitch_value, self._fractionMaxSpeed) ## fix head on the horizon 0.0, fix head looking for obstacle 0.3
                print("update head")

            time.sleep(0.1)

    def setHeadPositionSrvCallback(self,req):
        self._pitch_value=req.pitch_value
        return True
  

if __name__ == "__main__":
    rospy.init_node('pepper_head_pose_fix')
    ip=rospy.get_param('~ip',"127.0.0.1")
    port=rospy.get_param('~port',9559)
   
    session = qi.Session()
    try:
        session.connect("tcp://" + ip + ":" + str(port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    headFix=HeadFix(session)
    headFix.fixHead()
    # spin
    rospy.spin()



