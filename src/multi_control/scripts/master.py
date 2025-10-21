#!/usr/bin/python3
# coding=utf8
# Date:2021/09/07
# Author:Aiden
import os
import time
import math
import rospy
import roslibpy
import device_scan
import sqlite3 as sql
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

device_client = []
servo_topic = []
def init_servo():
    global device_client
    global servo_topic

    device_client = []
    servo_topic = []

    device_lists = device_scan.scan_device_all('ArmPi_FPV')

    for device in device_lists:
        cpu_serial, ip = device[1], device[2]
        print('device', cpu_serial, ip)
        device_client.append(roslibpy.Ros(host=str(ip), port=9090))
    
    for client in device_client:
        client.run()
        servo1_topic = roslibpy.Topic(client, '/r_joint_controller/command_duration', 'hiwonder_servo_msgs/CommandDuration')
        servo2_topic = roslibpy.Topic(client, '/joint5_controller/command_duration', 'hiwonder_servo_msgs/CommandDuration')
        servo3_topic = roslibpy.Topic(client, '/joint4_controller/command_duration', 'hiwonder_servo_msgs/CommandDuration')
        servo4_topic = roslibpy.Topic(client, '/joint3_controller/command_duration', 'hiwonder_servo_msgs/CommandDuration')
        servo5_topic = roslibpy.Topic(client, '/joint2_controller/command_duration', 'hiwonder_servo_msgs/CommandDuration')
        servo6_topic = roslibpy.Topic(client, '/joint1_controller/command_duration', 'hiwonder_servo_msgs/CommandDuration')
        servo_topic.extend(([[client, servo1_topic, servo2_topic, servo3_topic, servo4_topic, servo5_topic, servo6_topic]]))
    #print(servo_topic)

def runAction(actNum):
    actNum = "/home/ubuntu/software/armpi_fpv_control/ActionGroups/" + actNum + ".d6a"
    if os.path.exists(actNum) is True:
        ag = sql.connect(actNum)
        cu = ag.cursor()
        cu.execute("select * from ActionGroup")
        while True:
            act = cu.fetchone()
            if act is not None:
                for servo in servo_topic:
                    for i in range(0, len(act) - 2, 1):
                        if i == 3:
                            servo[i + 1].publish(roslibpy.Message({'data': math.radians(-240 * (act[2 + i] - 500)/1000.0), 'duration': act[1] / 1000.0}))
                        else:
                            servo[i + 1].publish(roslibpy.Message({'data': math.radians(240 * (act[2 + i] - 500)/1000.0), 'duration': act[1] / 1000.0}))
                time.sleep(float(act[1]) / 1000.0)
            else:
                break

        cu.close()
        ag.close()

try:
    rospy.init_node('arm_group_control')
    init_servo()
    rospy.loginfo('connect success')
    runAction('wave')
except BaseException as e:
    print(e)
    rospy.loginfo('connect failed')
