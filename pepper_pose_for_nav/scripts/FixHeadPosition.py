#!/usr/bin/env python  

import qi
import argparse
import sys
import time
import rospy


def HeadFix(session):
    """
    This example uses the setExternalCollisionProtectionEnabled method.
    """
    # Get the service ALMotion.
    motion_service  = session.service("ALMotion")
    memory_service = session.service("ALMemory")
    autolife_service = session.service("ALAutonomousLife")
    autolife_service.setState('disabled')
    posture_service = session.service("ALRobotPosture")
    posture_service.goToPosture("Stand",0.3)


    fractionMaxSpeed = 0.5
    motion_service.setStiffnesses("HEAD", 1.0)
    motion_service.setStiffnesses("TORSO", 1.0)
    error=0.1
    #pitch_value=0.3
    pitch_value=0.0
    isEnd=False

    while not rospy.is_shutdown():
            headYawPos = memory_service.getData("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
            headPitchPos = memory_service.getData("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
            print("headYawPos:"+str(headYawPos)+",headPitchPos:"+str(headPitchPos))
            #if abs(headPitchPos)>error or abs(headYawPos)>error:
            if headPitchPos>(pitch_value+error) or headPitchPos<(pitch_value-error)  or abs(headYawPos)>error:
                motion_service.setAngles("HeadYaw", 0.0, fractionMaxSpeed) 
                motion_service.setAngles("HeadPitch", pitch_value, fractionMaxSpeed) ## fix head on the horizon 0.0, fix head looking for obstacle 0.3
                print("update head")

            time.sleep(0.1)
  

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
    HeadFix(session)

