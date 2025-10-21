#!/usr/bin/python3
# coding=utf8
# Date:2021/05/04
# Author:Aiden
# 第13章 ArmPi FPV创意玩法课程\1.AI视觉创意玩法\第1课 物品追踪
import cv2
import time
import rospy
import queue
import signal
import numpy as np
from threading import RLock, Timer

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse, Trigger

from object_tracking.srv import SetTarget
from ros_robot_controller.msg import RGBState, RGBsState
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from armpi_fpv_common import pid, misc, common
from hiwonder_servo_controllers import bus_servo_control
from armpi_fpv_kinematics.kinematics_control import set_pose_target, go_pose_target

# 颜色跟踪
lock = RLock()

size = (320, 240)
target_color = ''
running = False
start = False
registered = False

x_dis = 500
y_dis = 0.167
Z_DIS = 0.2
z_dis = Z_DIS
x_pid = pid.PID(P=0.1, I=0, D=0.005)  # pid初始化
y_pid = pid.PID(P=0.000008, I=0, D=0)
z_pid = pid.PID(P=0.00005, I=0, D=0)

image_queue = queue.Queue(maxsize=2)
def image_callback(ros_image):
    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    if image_queue.full():
        # 如果队列已满，丢弃最旧的图像
        image_queue.get()
    # 将图像放入队列
    image_queue.put(bgr_image)

# 初始位置
def initMove(delay=True):
    with lock:
        bus_servo_control.set_servos(joints_pub, 0.5, [[1, 200],])
        go_pose_target(joints_pub, [y_dis, 0, Z_DIS], 0, [-10, 10], 0, 1.5)
    if delay:
        rospy.sleep(2)

def set_rgb(r, g, b):
    rgb1 = RGBState()
    rgb1.id = 1
    rgb1.r = r
    rgb1.g = g
    rgb1.b = b
    rgb2 = RGBState()
    rgb2.id = 2
    rgb2.r = r
    rgb2.g = g
    rgb2.b = b
    msg = RGBsState()
    msg.data = [rgb1, rgb2]
    rgb_pub.publish(msg)

# 变量重置
def reset():
    global x_dis, y_dis, z_dis
    global target_color
    
    with lock:
        x_dis = 500
        y_dis = 0.167
        z_dis = Z_DIS
        x_pid.clear()
        y_pid.clear()
        z_pid.clear()
        set_rgb(0, 0, 0)
        target_color = ''

color_range = None
# app初始化调用
def init():
    global color_range 
    
    rospy.loginfo("object tracking Init")
    color_range = rospy.get_param('/lab_config_manager/color_range_list', {})  # get lab range from ros param server
    reset()
    initMove()

def run():
    global x_dis, y_dis, z_dis

    if running:
        try:
            image = image_queue.get(block=True, timeout=1)
        except queue.Empty:
            return
        if start:
            img_h, img_w = image.shape[:2]

            cv2.line(image, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
            cv2.line(image, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)

            frame_resize = cv2.resize(image, size, interpolation=cv2.INTER_NEAREST)
            frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

            area_max = 0
            area_max_contour = 0

            if target_color in color_range:
                target_color_range = color_range[target_color]
                frame_mask = cv2.inRange(frame_lab, tuple(target_color_range['min']), tuple(target_color_range['max']))  # 对原图像和掩模进行位运算
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
                area_max_contour, area_max = common.get_area_max_contour(contours)  # 找出最大轮廓

            if area_max > 500:  # 有找到最大面积
                (center_x, center_y), radius = cv2.minEnclosingCircle(area_max_contour)  # 获取最小外接圆
                center_x = int(misc.map(center_x, 0, size[0], 0, img_w))
                center_y = int(misc.map(center_y, 0, size[1], 0, img_h))
                radius = int(misc.map(radius, 0, size[0], 0, img_w))
                cv2.circle(image, (int(center_x), int(center_y)), int(radius), common.range_rgb[target_color], 2)

                x_pid.SetPoint = img_w / 2.0  # 设定
                x_pid.update(center_x)  # 当前
                x_dis += x_pid.output  # 输出

                x_dis = 200 if x_dis < 200 else x_dis
                x_dis = 800 if x_dis > 800 else x_dis
                # print(area_max)
                y_pid.SetPoint = 2500  # 设定
                if abs(area_max - 2500) < 200:
                    area_max = 2500
                y_pid.update(area_max)  # 当前
                dy = y_pid.output
                y_dis += dy  # 输出
                y_dis = 0.12 if y_dis < 0.12 else y_dis
                y_dis = 0.19 if y_dis > 0.19 else y_dis

                z_pid.SetPoint = img_h / 2.0
                z_pid.update(center_y)
                dy = z_pid.output
                z_dis += dy

                z_dis = 0.23 if z_dis > 0.23 else z_dis
                z_dis = 0.15 if z_dis < 0.15 else z_dis

                res = set_pose_target((y_dis, 0, round(z_dis, 4)), 0, [-10, 10], 0)
                if res[1]:
                    bus_servo_control.set_servos(joints_pub, 0.02, [[3, res[1][3]], [4, res[1][2]], [5, res[1][1]], [6, x_dis]])
                    time.sleep(0.02)
        ros_image = common.cv2_image2ros(image)
        image_pub.publish(ros_image)
    else:
        time.sleep(0.01)

def enter_func(msg):
    global lock
    global image_sub
    global running
    global registered
    
    rospy.loginfo("enter object tracking")
    init()
    with lock:
        if not registered:
            registered = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
            running = True
    return [True, 'enter']

heartbeat_timer = None
def exit_func(msg):
    global lock
    global image_sub
    global running, start
    global registered
    
    rospy.loginfo("exit object tracking")
    with lock:
        running = False
        start = False
        reset()
        try:
            if registered:
                registered = False
                if heartbeat_timer is not None:
                    heartbeat_timer.cancel()
                image_sub.unregister()
        except BaseException as e:
            rospy.loginfo('%s', e)
        
    return [True, 'exit']

def start_running():
    global lock
    global start
    
    rospy.loginfo("start running object tracking")
    with lock:
        start = True

def stop_running():
    global lock
    global start
    
    rospy.loginfo("stop running object tracking")
    with lock:
        start = False
        reset()
        initMove(delay=False)

def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()
        
    return [True, 'set_running']

def set_target(msg):
    global lock
    global target_color
    
    rospy.loginfo("%s", msg)
    with lock:
        target_color = msg.data
        set_rgb(common.range_rgb[target_color][2], common.range_rgb[target_color][1], common.range_rgb[target_color][0])
        rospy.sleep(0.1)
        
    return [True, 'set_target']

def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/object_tracking/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp

def shutdown(signal, frame):
    exit_func(None)
    rospy.signal_shutdown('shutdown')

if __name__ == '__main__':
    rospy.init_node('object_tracking', log_level=rospy.INFO)
    
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    
    image_pub = rospy.Publisher('/object_tracking/image_result', Image, queue_size=1)  # register result image publisher

    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    
    enter_srv = rospy.Service('/object_tracking/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/object_tracking/exit', Trigger, exit_func)
    running_srv = rospy.Service('/object_tracking/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/object_tracking/set_target', SetTarget, set_target)
    heartbeat_srv = rospy.Service('/object_tracking/heartbeat', SetBool, heartbeat_srv_cb)

    signal.signal(signal.SIGINT, shutdown) 

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        
        msg = SetTarget()
        msg.data = 'blue'
        
        set_target(msg)
        start_running()
    
    while not rospy.is_shutdown():
        run()
