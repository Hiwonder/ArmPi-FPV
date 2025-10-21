#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/10/22
import cv2
import math
import yaml
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

range_rgb = {
    'red': (0, 50, 255),
    'blue': (255, 50, 0),
    'green': (50, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def loginfo(msg):
    rospy.loginfo('\033[1;32m%s\033[0m' % msg)

def cv2_image2ros(image, frame_id=''):
    ros_image = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = frame_id
    ros_image.height = image.shape[:2][0]
    ros_image.width = image.shape[:2][1]
    ros_image.encoding = 'bgr8'
    ros_image.data = image.tostring()
    ros_image.header = header
    ros_image.step = ros_image.width * 3

    return ros_image


def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()

    data = yaml.load(file_data, Loader=yaml.FullLoader)

    return data


def save_yaml_data(data, yaml_file):
    f = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, f)

    f.close()

def get_area_max_contour(contours, threshold=50):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况(get the contour whose area is the largest. Filter out those whose area is too small)
    :param contours: 轮廓列表(contour list)
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤(area threshold. Contour whose area is less than this value will be filtered out)
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None(if the maximum contour area is greater than this threshold, return the
    largest contour, otherwise return None)
    """
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > threshold:  # 过滤干扰
                area_max_contour = c

    return area_max_contour,  contour_area_max  # 返回最大的轮廓