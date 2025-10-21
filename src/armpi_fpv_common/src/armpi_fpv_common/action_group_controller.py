#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import rospy
import sqlite3 as sql
from ros_robot_controller.msg import SetBusServoState, BusServoState

class ActionGroupController:
    runningAction = False
    stopRunning = False

    def __init__(self, action_path=os.path.split(os.path.realpath(__file__))[0]):
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/ros_robot_controller/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.sleep(1)
        self.action_path = action_path
        self.servo_state_pub = rospy.Publisher('ros_robot_controller/bus_servo/set_state', SetBusServoState, queue_size=10)
        time.sleep(0.1)

    def stop_servo(self):
        data = []
        msg = SetBusServoState()
        for i in range(24):
            servo_state = BusServoState()
            servo_state.present_id = [1, i + 1]
            servo_state.stop = [1, 1]
            data.append(servo_state)
        msg.state = data
        self.servo_state_pub.publish(msg)

    def stop_action_group(self):
        self.stopRunning = True

    def runAction(self, actNum):
        '''
        运行动作组，无法发送stop停止信号
        :param actNum: 动作组名字 ， 字符串类型
        :param times:  运行次数
        :return:
        '''
        if actNum is None:
            return
        actNum = os.path.join(self.action_path, actNum + ".d6a")
        self.stopRunning = False
        if os.path.exists(actNum) is True:
            if self.runningAction is False:
                self.runningAction = True
                ag = sql.connect(actNum)
                cu = ag.cursor()
                cu.execute("select * from ActionGroup")
                while True:
                    act = cu.fetchone()
                    if self.stopRunning is True:
                        self.stopRunning = False                   
                        break
                    if act is not None:
                        for i in range(0, len(act)-2, 1):
                            data = SetBusServoState()
                            servo_state = BusServoState()
                            servo_state.present_id = [1, i + 1]
                            servo_state.position = [1, act[2 + i]]
                            data.state = [servo_state]
                            data.duration = act[1]/1000.0
                            self.servo_state_pub.publish(data)
                        time.sleep(float(act[1])/1000.0)
                    else:   # 运行完才退出
                        break
                self.runningAction = False
                
                cu.close()
                ag.close()
        else:
            self.runningAction = False
            print("未能找到动作组文件")
