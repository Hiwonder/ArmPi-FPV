#!/usr/bin/python3
# coding=utf8
# Date:2021/02/27
# Author:Aiden
# 第13章 ArmPi FPV创意玩法课程\1.AI视觉创意玩法\第2课 人脸识别
import cv2
import math
import rospy
import queue
import signal
import threading
import numpy as np
from typing import Union, Tuple
from threading import RLock, Timer
from sensor_msgs.msg import Image
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from std_srvs.srv import Trigger, SetBool, SetBoolResponse

import mediapipe as mp
from armpi_fpv_common import common
from hiwonder_servo_controllers import bus_servo_control
from armpi_fpv_kinematics.kinematics_control import go_pose_target

running = False
registered = False
lock = RLock()

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.8) #阈值

def _normalized_to_pixel_coordinates(
    normalized_x: float, normalized_y: float, image_width: int,
    image_height: int) -> Union[None, Tuple[int, int]]:
  """Converts normalized value pair to pixel coordinates."""

  # Checks if the float value is between 0 and 1.
  def is_valid_normalized_value(value: float) -> bool:
    return (value > 0 or math.isclose(0, value)) and (value < 1 or
                                                      math.isclose(1, value))

  if not (is_valid_normalized_value(normalized_x) and
          is_valid_normalized_value(normalized_y)):
    # TODO: Draw coordinates even if it's outside of the image bounds.
    return None
  x_px = min(math.floor(normalized_x * image_width), image_width - 1)
  y_px = min(math.floor(normalized_y * image_height), image_height - 1)
  return x_px, y_px

# 初始位置
def initMove(delay=True):
    with lock:
        go_pose_target(joints_pub, (0.17, 0, 0.3), 65, [-90, 90], 0, 1)
    if delay:
        rospy.sleep(2)

d_pulse = 5
have_move = False
servo6_pulse = 500
start_greet = False
action_finish = True
# 变量重置
def reset():
    global d_pulse
    global have_move       
    global start_greet
    global servo6_pulse
    global action_finish 

    with lock:
        d_pulse = 5
        have_move = False
        servo6_pulse = 500
        start_greet = False

# app初始化调用
def init():
    print("face detect Init")
    initMove()
    reset()

def move():
    global have_move
    global start_greet
    global action_finish 
    global d_pulse, servo6_pulse 
        
    while True:
        if running:
            if start_greet:
                start_greet = False                
                action_finish = False

                bus_servo_control.set_servos(joints_pub, 0.3, ((2, 300),))
                rospy.sleep(0.3)

                bus_servo_control.set_servos(joints_pub, 0.6, ((2, 700),))
                rospy.sleep(0.6)
                
                bus_servo_control.set_servos(joints_pub, 0.6, ((2, 300),))
                rospy.sleep(0.6)
                
                bus_servo_control.set_servos(joints_pub, 0.3, ((2, 500),))
                rospy.sleep(0.3)
                
                bus_servo_control.set_servos(joints_pub, 0.4, ((1, 200),))
                rospy.sleep(0.4)

                bus_servo_control.set_servos(joints_pub, 0.4, ((1, 500),))
                rospy.sleep(0.4)
                
                bus_servo_control.set_servos(joints_pub, 0.4, ((1, 200),))
                rospy.sleep(0.4)
                
                bus_servo_control.set_servos(joints_pub, 0.4, ((1, 500),))
                rospy.sleep(1)
                
                action_finish = True
                
                have_move = True
            else:
                if have_move:
                    have_move = False
                    bus_servo_control.set_servos(joints_pub, 0.2, ((1, 500), (2, 500)))
                    rospy.sleep(0.2)
                if servo6_pulse > 875 or servo6_pulse < 125:
                    d_pulse = -d_pulse
                bus_servo_control.set_servos(joints_pub, 0.05, ((6, servo6_pulse),))           
                servo6_pulse += d_pulse       
               
                rospy.sleep(0.05)                
        else:
            rospy.sleep(0.01)
            
# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

def run():
    global start_greet

    try:
        image = image_queue.get(block=True, timeout=1)
    except queue.Empty:
        return

    if running:
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = face_detection.process(image)
        image.flags.writeable = True
        h, w = image.shape[:2]
        if results.detections:
            for detection in results.detections:
                if detection.label_id == [0]:
                    mp_drawing.draw_detection(image, detection)
                    data = detection.location_data.relative_bounding_box
                    point = _normalized_to_pixel_coordinates(data.xmin, data.ymin, w, h)
                    
                    if point is not None:
                        x1, y1 = point
                        width, height = _normalized_to_pixel_coordinates(data.width, data.height, w, h)
                        x, y = (x1 + width / 2, y1 + height / 2)
                        cv2.circle(image, (int(x), int(y)), 5, (255, 255, 0), -1)
                        if action_finish and abs(x - w/2) < 100:
                            start_greet = True       

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    ros_image = common.cv2_image2ros(image)
    image_pub.publish(ros_image)

image_queue = queue.Queue(maxsize=2)
def image_callback(ros_image):
    rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    if image_queue.full():
        # 如果队列已满，丢弃最旧的图像
        image_queue.get()
    # 将图像放入队列
    image_queue.put(bgr_image)

def enter_func(msg):
    global lock
    global image_sub
    global running
    global registered

    rospy.loginfo("enter face detect")
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
    
    rospy.loginfo("exit face detect")
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

    rospy.loginfo("start running face detect")
    with lock:
        running = True

def stop_running():
    global lock
    global running

    rospy.loginfo("stop running face detect")
    with lock:
        running = False
        reset()
        initMove(delay=False)

def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()
    
    return [True, 'set_running']

def heartbeat_srv_cb(msg):
    global heartbeat_timer
    
    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/face_detect/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp

def shutdown(signal, frame):
    exit_func(None)
    rospy.signal_shutdown('shutdown')

if __name__ == '__main__':
    rospy.init_node('face_detect', log_level=rospy.INFO)
    
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

    image_pub = rospy.Publisher('/face_detect/image_result', Image, queue_size=1)  # register result image publisher

    enter_srv = rospy.Service('/face_detect/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/face_detect/exit', Trigger, exit_func)
    running_srv = rospy.Service('/face_detect/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/face_detect/heartbeat', SetBool, heartbeat_srv_cb)
    
    signal.signal(signal.SIGINT, shutdown) 
    debug = 0
    if debug:
        enter_func(1)
        start_running()
    
    while not rospy.is_shutdown():
        run()
