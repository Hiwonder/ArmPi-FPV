#!/usr/bin/python3
# coding=utf8
# Date:2021/04/20
# Author:Aiden
# 第13章 ArmPi FPV创意玩法课程\2.智慧仓储课程\第4课 智能出库
import cv2
import math
import rospy
import threading
import numpy as np
from threading import Timer

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool, SetBoolResponse

from warehouse.msg import Grasp
from warehouse.srv import SetOutTarget
from ros_robot_controller.msg import BuzzerState 
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from armpi_fpv_common import common
from hiwonder_servo_controllers import bus_servo_control
from armpi_fpv_kinematics.kinematics_control import set_pose_target

# 出仓
# 如未声明，使用的长度，距离单位均为m

# 初始化
target_data = ()
running = False
registered = False
lock = threading.RLock()

# 初始位置
def initMove():
    with lock:
        bus_servo_control.set_servos(joints_pub, 1.5, ((1, 200), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    rospy.sleep(2)

# 变量重置
def reset():
    global target_data
    
    target_data = ()   

# app初始化调用
def init():
    rospy.loginfo("out Init")
    initMove()
    reset()

def pick(grasps):
    position = grasps.grasp_pos.position
    rotation = grasps.grasp_pos.rotation
    approach = grasps.grasp_approach
    retreat = grasps.grasp_retreat
    
    # 计算是否能够到达目标位置，如果不能够到达，返回False
    target1 = set_pose_target((position.y + approach.y, position.x + approach.x, position.z + approach.z), rotation.r, [-90, 90], 0)
    target2 = set_pose_target((position.y, position.x, position.z), rotation.r, [-90, 90], 0)
    target3 = set_pose_target((position.y, position.x, position.z + grasps.up), rotation.r, [-90, 90], 0)
    target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)

    if not running:
        return False 
    if target1[1] and target2[1] and target3[1] and target4[1]:
        # 第一步：云台转到朝向目标方向，夹持器打开
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 0.8, ((1, grasps.pre_grasp_posture), (2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data[0])))
        rospy.sleep(0.8)
        if not running:
            return False
        
        # 第二步：移到接近点
        target1 = set_pose_target((position.y + approach.y, position.x + approach.x, position.z + approach.z), rotation.r, [-90, 90], 0)
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 0.5, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))      
        rospy.sleep(0.5)
        if not running:
            return False
        
        # 第三步：移到目标点
        target2 = set_pose_target((position.y, position.x, position.z), rotation.r, [-90, 90], 0)
        servo_data = target2[1]
        bus_servo_control.set_servos(joints_pub, 0.5, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))
        rospy.sleep(1)
        if not running:
            target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1, ((1, 200), (3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))       
            rospy.sleep(1)            
            return False
        
        # 第四步：夹取
        bus_servo_control.set_servos(joints_pub, 0.5, ((1, grasps.grasp_posture), ))               
        rospy.sleep(1)
        if not running:
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, grasps.pre_grasp_posture), ))               
            rospy.sleep(0.5)
            target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1, ((1, 200), (3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))       
            rospy.sleep(1)             
            return False
        
        # 第五步：抬升
        target3 = set_pose_target((position.y, position.x, position.z + grasps.up), rotation.r, [-90, 90], 0)
        servo_data = target3[1]
        if servo_data != target2[1]:
            bus_servo_control.set_servos(joints_pub, 0.4, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))
            rospy.sleep(0.5)
        if not running:
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, grasps.pre_grasp_posture), ))               
            rospy.sleep(0.5)
            target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1, ((1, 200), (3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))      
            rospy.sleep(1)              
            return False
        
        # 第六步：移到撤离点
        target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)
        servo_data = target4[1]
        if servo_data != target3[1]:            
            bus_servo_control.set_servos(joints_pub, 0.5, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))        
            rospy.sleep(0.5)
            if not running:
                bus_servo_control.set_servos(joints_pub, 0.5, ((1, grasps.pre_grasp_posture), ))               
                rospy.sleep(0.5)                 
                return False
            
        # 第七步：移到稳定点
        target1 = set_pose_target((position.y + approach.y, position.x + approach.x, position.z + approach.z), rotation.r, [-90, 90], 0)
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 0.5, ((2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data[0])))
        rospy.sleep(0.5)
        if not running:
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, grasps.pre_grasp_posture), ))               
            rospy.sleep(0.5)             
            return False
        
        return target2[2]
    else:
        rospy.loginfo('pick failed')
        return False

def place(places):
    position = places.grasp_pos.position
    rotation = places.grasp_pos.rotation
    approach = places.grasp_approach
    retreat = places.grasp_retreat
    
    # 计算是否能够到达目标位置，如果不能够到达，返回False
    target1 = set_pose_target((position.y + approach.y, position.x + approach.x, position.z + approach.z), rotation.r, [-90, 90], 0)
    target2 = set_pose_target((position.y, position.x, position.z), rotation.r, [-90, 90], 0)
    target3 = set_pose_target((position.y, position.x, position.z + places.up), rotation.r, [-90, 90], 0)
    target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)

    if not running:
        return False
    if target1[1] and target2[1] and target3[1] and target4[1]:
        # 第一步：云台转到朝向目标方向
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 0.8, ((1, places.pre_grasp_posture), (2, servo_data[0]), (3, 80), (4, 825), (5, 625), (6, servo_data[0])))
        rospy.sleep(0.8)
        if not running:
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, places.grasp_posture), ))               
            rospy.sleep(0.5)            
            return False
        
        # 第二步：移到接近点
        target1 = set_pose_target((position.y + approach.y, position.x + approach.x, position.z + approach.z), rotation.r, [-90, 90], 0)
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 1, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))      
        rospy.sleep(1)
        if not running:
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, places.grasp_posture), ))               
            rospy.sleep(0.5)            
            return False
        
        # 第三步：移到目标点
        target2 = set_pose_target((position.y, position.x, position.z), rotation.r, [-90, 90], 0)
        servo_data = target2[1]
        if servo_data != target1[1]:        
            bus_servo_control.set_servos(joints_pub, 1, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))
            rospy.sleep(1.5)
        if not running:
            bus_servo_control.set_servos(joints_pub, 0.5, ((1, places.grasp_posture), ))               
            rospy.sleep(0.5)
            target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1, ((1, 200), (3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))    
            rospy.sleep(1)              
            return False
        
        # 第四步：放置
        bus_servo_control.set_servos(joints_pub, 0.8, ((1, places.grasp_posture), ))         
        rospy.sleep(1)
        if not running:
            target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1, ((1, 200), (3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))       
            rospy.sleep(1)              
            return False
        
        # 第五步：抬升
        target3 = set_pose_target((position.y, position.x, position.z + places.up), rotation.r, [-90, 90], 0)
        servo_data = target3[1]
        if servo_data != target2[1]:
            bus_servo_control.set_servos(joints_pub, 0.8, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))
            rospy.sleep(0.8)
        if not running:
            target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1, ((1, 200), (3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))       
            rospy.sleep(1)              
            return False
        
        # 第六步：移到撤离点
        target4 = set_pose_target((position.y + retreat.y, position.x + retreat.x, position.z + retreat.z), rotation.r, [-90, 90], 0)
        servo_data = target4[1]
        if servo_data != target3[1]:
            bus_servo_control.set_servos(joints_pub, 0.5, ((3, servo_data[3]), (4, servo_data[2]), (5, servo_data[1]), (6, servo_data[0])))
            rospy.sleep(0.5)
            if not running:
                return False
            
        # 第七步：移到稳定点
        target1 = set_pose_target((position.y + approach.y, position.x + approach.x, position.z + approach.z), rotation.r, [-90, 90], 0)
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 1, ((2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data[0])))
        rospy.sleep(1)
        if not running:
            return False
        
        return True
    else:
        rospy.loginfo('place failed')
        return False

#############################################
# 货架每层位置x, y, z(m)
shelf_position = {'R1':[-0.277,  0, 0.02],
                  'R2':[-0.277,  0, 0.12],
                  'R3':[-0.277,  0, 0.21],
                  'L1':[0.277, 0, 0.02],
                  'L2':[0.277, 0, 0.12],
                  'L3':[0.277, 0, 0.21]}

# 每层货架对应放置点位置x, y, z(m)
place_position = {'R1':[-0.06, 0.15, 0.01],
                  'R2':[0.06,  0.15, 0.01],
                  'R3':[0,     0.15, 0.01],
                  'L1':[-0.06, 0.22, 0.015],
                  'L2':[0.06,  0.22, 0.015],
                  'L3':[0,     0.22, 0.015]}
###############################################
# 每层放置时的俯仰角
roll_dict = {'R1': 50,
             'R2': 60,
             'R3': 90,
             'L1': 50,
             'L2': 60,
             'L3': 90}

def move():
    while True:
        if running:            
            if len(target_data) != 0:
                i = target_data[0] 
                if running:
                    if shelf_position[i][0] > 0:
                        approach_x = -0.07
                    else:
                        approach_x = 0.07
                                        
                    grasps = Grasp()
                    # 夹取的位置
                    grasps.grasp_pos.position.x = shelf_position[i][0]
                    grasps.grasp_pos.position.y = shelf_position[i][1]
                    grasps.grasp_pos.position.z = shelf_position[i][2]
                    
                    # 夹取时的俯仰角
                    grasps.grasp_pos.rotation.r = roll_dict[i]
                    
                    # 夹取后抬升的距离
                    grasps.up = 0
                    
                    # 夹取时靠近的方向和距离
                    grasps.grasp_approach.x = approach_x
                    
                    # 夹取后后撤的方向和距离
                    grasps.grasp_retreat.x = approach_x
                    grasps.grasp_retreat.z = 0.02
                    
                    # 夹取前后夹持器的开合
                    grasps.grasp_posture = 570
                    grasps.pre_grasp_posture = 200
                    msg = BuzzerState()
                    msg.freq = 1900
                    msg.on_time = 0.1
                    msg.off_time = 0.9
                    msg.repeat = 1
                    buzzer_pub.publish(msg)
                    result = pick(grasps)                              
                    if result:                    
                        if place_position[i][0] < 0:
                            yaw = int(120 - (90 + math.degrees(math.atan2(place_position[i][0], place_position[i][1]))))
                        else:
                            yaw = int(120 + (90 - math.degrees(math.atan2(place_position[i][0], place_position[i][1]))))
                        places = Grasp()                    
                        places.grasp_pos.position.x = place_position[i][0]
                        places.grasp_pos.position.y = place_position[i][1]
                        places.grasp_pos.position.z = place_position[i][2]
                        places.grasp_pos.rotation.r = -160
                        
                        places.up = 0.045
                        places.grasp_approach.z = places.up
                        places.grasp_retreat.z = places.up
                        places.grasp_posture = 200
                        places.pre_grasp_posture = 570
                        
                        place(places)
                        try:
                            target_data.remove(i)
                        except BaseException as e:
                            print(e)
                    initMove()
            else:
                rospy.sleep(0.01)
        else:
            rospy.sleep(0.01)
            
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 将ros发布的图像转化成opencv能够处理的格式，并且将处理后的图像发布出去
def image_callback(ros_image):
    global lock
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    ros_image = common.cv2_image2ros(image)
    image_pub.publish(ros_image)
            
def enter_func(msg):
    global lock
    global image_sub
    global running
    global registered
    
    rospy.loginfo("enter out")
    with lock:
        init()
        if not registered:
            registered = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
        
    return [True, 'enter']

heartbeat_timer = None
def exit_func(msg):
    global lock
    global image_sub
    global running
    global registered
    
    rospy.loginfo("exit out")
    with lock:
        running = False
        try:
            if registered:
                registered = False
                if heartbeat_timer is not None:
                    heartbeat_timer.cancel()
                image_sub.unregister()
        except:
            pass
        
    return [True, 'exit']

def start_running():
    global lock
    global running
    
    rospy.loginfo("start running out")
    with lock:
        running = True

def stop_running():
    global lock
    global running
    
    rospy.loginfo("stop running out")
    with lock:
        running = False
        reset()

def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()
        
    return [True, 'set_running']

def set_target(msg):
    global lock
    global target_data
    
    rospy.loginfo('%s', msg.position)
    with lock:
        target_data = msg.position

    return [True, 'set_target']

def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/out/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('out', log_level=rospy.INFO)
    
    # 舵机发布
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    
    # 图像发布
    image_pub = rospy.Publisher('/out/image_result', Image, queue_size=1)  # register result image publisher
    
    # app通信服务
    enter_srv = rospy.Service('/out/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/out/exit', Trigger, exit_func)
    running_srv = rospy.Service('/out/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/out/set_target', SetOutTarget, set_target)
    heartbeat_srv = rospy.Service('/out/heartbeat', SetBool, heartbeat_srv_cb)
    
    buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
    
    debug = 0
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        
        msg = SetOutTarget()
        msg.position = ['R1', 'R2', 'R3', 'L1', 'L2', 'L3']
        
        set_target(msg)

        start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
