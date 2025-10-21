#关掉所有ros后台
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
或者
~/.stop_ros.sh

#第8章 ArmPi FPV逆运动学基础及实战应用课程
#第3课 单次控制多个舵机
python3 /home/ubuntu/course/control_course/control_by_servo.py

#第4课 机械臂上下左右移动
python3 /home/ubuntu/course/control_course/control_by_kinematics.py

#第5课 机械臂色块位置识别
python3 /home/ubuntu/course/vision_course/get_color_position.py

#第6课 色块角度识别
python3 /home/ubuntu/course/vision_course/get_color_rotation_angle.py

#第7课 机械臂色块追踪
python3 /home/ubuntu/course/vision_course/track_by_color.py

#第8课 机械爪位置自适应调整
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /object_pallezting/enter "{}"
rosservice call /object_pallezting/set_running "data: true"
rosservice call /object_pallezting/set_running "data: false"
rosservice call /object_pallezting/exit "{}"

#第9课 机械臂色块分拣
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /object_sorting/enter "{}"
rosservice call /object_sorting/set_running "data: true"
rosservice call /object_pallezting/set_running "data: false"
rosservice call /object_sorting/set_target "color:
- 'red'
tag:
- 'tag1'"
rosservice call /object_sorting/exit "{}"

#第10课 机械臂色块码垛
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /object_pallezting/enter "{}"
rosservice call /object_pallezting/set_running "data: true"
rosservice call /object_pallezting/set_running "data: false"
rosservice call /object_pallezting/exit "{}"

#第11章 单目相机课程
#第1课 单目相机的基本使用
python3 /home/ubuntu/course/vision_course/camera_display.py

#第2课 相机标定
roslaunch armpi_fpv_bringup camera_calib.launch

#第12章 ROS+OpenCV课程\1.AI视觉识别
#第2课 颜色识别实验
python3 /home/ubuntu/course/vision_course/color_recognition.py

#第3课 AprilTag识别
python3 /home/ubuntu/course/vision_course/apriltag_recognition.py

#第4课 AprilTagy远近识别实验
python3 /home/ubuntu/course/vision_course/get_apriltag_area.py

#第5课 人脸检测实验
python3 /home/ubuntu/course/vision_course/face_recognition.py

#第6课 手势识别实验
python3 /home/ubuntu/course/vision_course/gesture_recognition.py

#第7课 形状识别实验
python3 /home/ubuntu/course/vision_course/shape_recognition.py

#第8课 条形码识别实验
python3 /home/ubuntu/course/vision_course/barcode_recognition.py

#第12章 ROS+OpenCV课程\2.AI视觉追踪
#第1课 色块定位实验
python3 /home/ubuntu/course/vision_course/track_by_color.py

#第2课 色块追踪实验
python3 /home/ubuntu/course/vision_course/get_color_position.py

#第3课 人脸定位实验
python3 /home/ubuntu/course/vision_course/get_face_position.py

#第4课 人脸追踪实验
python3 /home/ubuntu/course/vision_course/track_by_face.py

#第5课 AprilTag定位
python3 /home/ubuntu/course/vision_course/get_apriltag_position.py

#第6课 AprilTag追踪
python3 /home/ubuntu/course/vision_course/track_by_apriltag.py

#第13章 ArmPi FPV创意玩法课程\1.AI视觉创意玩法
#第1课 物品追踪
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /object_tracking/enter "{}"
rosservice call /object_tracking/set_running "data: true"
rosservice call /object_tracking/set_running "data: false"
rosservice call /object_tracking/set_target "data: 'red'"
rosservice call /object_tracking/exit "{}"

#第2课 人脸识别
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /face_detect/enter "{}"
rosservice call /face_detect/set_running "data: true"
rosservice call /face_detect/set_running "data: false"
rosservice call /face_detect/exit "{}"

#第3课 你放我抓
python3 /home/ubuntu/course/vision_course/grasp_by_vision.py

#第4课 物品分拣
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /object_sorting/enter "{}"
rosservice call /object_sorting/set_running "data: true"
rosservice call /object_pallezting/set_running "data: false"
rosservice call /object_sorting/set_target "color:
- 'red'
tag:
- 'tag1'"
rosservice call /object_sorting/exit "{}"

#第5课 智能码垛
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /object_pallezting/enter "{}"
rosservice call /object_pallezting/set_running "data: true"
rosservice call /object_pallezting/set_running "data: false"
rosservice call /object_pallezting/exit "{}"

#第6课 手势识别堆积木实验
python3 /home/ubuntu/course/vision_course/control_by_gesture.py

#第13章 ArmPi FPV创意玩法课程\2.智慧仓储课程
#第3课 智能入库
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /in/enter "{}"
rosservice call /in/set_running "data: true"
rosservice call /in/set_running "data: false"
rosservice call /in/set_target "goods:
- 'red'
- 'tag1'
position:
- 'R3'
- 'L3'"
rosservice call /in/exit "{}"

#第4课 智能出库
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /out/enter "{}"
rosservice call /out/set_running "data: true"
rosservice call /out/set_running "data: false"
rosservice call /out/set_target "position:
- 'R1'
- 'R2'
- 'R3'"
rosservice call /out/exit "{}"

#第5课 智能转仓
roslaunch armpi_fpv_bringup bringup.launch
rosservice call /exchange/enter "{}"
rosservice call /exchange/set_running "data: true"
rosservice call /exchange/set_running "data: false"
rosservice call /exchange/set_target "position:
- 'R1'
- 'L1'"
rosservice call /exchange/exit "{}"

#第13章 ArmPi FPV创意玩法课程\3.语音交互玩法课程
#1.树莓派语音识别实验
python3 /home/ubuntu/armpi_fpv/src/asr_control/scripts/asr_demo.py

#2.树莓派语音合成实验
python3 /home/ubuntu/armpi_fpv/src/asr_control/scripts/tts_demo.py

#3.树莓派语音交互实验
python3 /home/ubuntu/armpi_fpv/src/asr_control/scripts/interaction.py

#第4课 语音播报垃圾分类
python3 /home/ubuntu/course/vision_course/garbage_classification.py

#第5课 语音控制色块分拣
roslaunch armpi_fpv_bringup bringup.launch
python3 /home/ubuntu/armpi_fpv/src/asr_control/scripts/asr_control_sorting.py

#第14章 群发控制课程
#第2课 多台机械臂控制
roslaunch armpi_fpv_bringup bringup.launch
rosrun multi_control master.py

#第15章 拓展课程之传感器开发与应用\1.拓展课程-传感器基础开发课程
#第1课 光敏传感器实验
python3 /home/ubuntu/course/sensor_course/sensor_examples/light_sensor.py

#颜色传感器校准
python3 /home/ubuntu/course/sensor_course/sensor_examples/color_sensor_calibration.py

#第2课 颜色传感器控制
python3 /home/ubuntu/course/sensor_course/sensor_examples/color_sensor.py

#第3课 红外避障传感器检测
python3 /home/ubuntu/course/sensor_course/sensor_examples/infrared_obstacle_avoidance.py

#第4课 触摸传感器检测
python3 /home/ubuntu/course/sensor_course/sensor_examples/touch_sensor.py

#第5课 发光超声波测距
python3 /home/ubuntu/course/sensor_course/sensor_examples/ultrasonic_sensor.py

#第6课 发光超声波RGB控制
python3 /home/ubuntu/course/sensor_course/sensor_examples/ultrasonic_rgb.py

#第7课 点阵模块显示
python3 /home/ubuntu/course/sensor_course/sensor_examples/dot_matrix_module.py

#第8课 数码管显示
python3 /home/ubuntu/course/sensor_course/sensor_examples/digital_tube.py

#第9课 风扇模块控制
python3 /home/ubuntu/course/sensor_course/sensor_examples/fan_module.py

#第15章 拓展课程之传感器开发与应用\2.拓展课程-传感器应用开发课程
#第1课 机械臂人脸检测+风扇控制
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/fan_control_by_face_detect.py

#第2课 机械臂超声波扫描显示
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/ultrasonic_measuring_display.py

#第3课 机械臂超声波控制抓取
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/grasp_by_ultrasonic_sensor.py

#第4课 机械臂超声波+AI识别控制抓取
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/grasp_by_ultrasonic_and_vision.py

#第5课 机械臂颜色传感器识别
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/color_sorting_by_color_sensor.py

#第6课 机械臂超声波补光
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/fill_light_by_ultrasonic.py

#第7课 机械臂颜色+AI识别
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/color_sorting_by_sensor_and_vision.py

#第8课 机械臂点阵显示
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/dot_matrix_display.py

#第9课 机械臂形状AI识别并显示
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/shape_recognition_by_vision.py

#第10课 机械臂感光控制
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/action_control_by_light_sensor.py

#第11课 机械臂触摸控制
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/action_control_by_touch_sensor.py

#第12课 机械臂红外检测
python3 /home/ubuntu/course/sensor_course/sensor_with_arm/action_control_by_infrared_sensor.py

#按键控制机械臂
python3 /home/ubuntu/course/control_course/control_by_button.py

#板载按键
python3 /home/ubuntu/course/sensor_course/sensor_examples/key.py

#板载led
python3 /home/ubuntu/course/sensor_course/sensor_examples/led.py

#动作上位机
python3 /home/ubuntu/software/armpi_fpv_control/main.py

#阈值上位机
python3 /home/ubuntu/software/lab_config/main.py

#舵机测试上位机
python3 /home/ubuntu/software/servo_tool.py

#自启脚本
/home/pi/armpi_fpv/start_node.sh

#自启方式
systemd
路径：/etc/systemd/system/start_node.service