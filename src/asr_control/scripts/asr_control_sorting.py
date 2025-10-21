#!/usr/bin/python3
# coding=utf8
# Date:2021/04/16
# Author:Aiden
# 第13章 ArmPi FPV创意玩法课程\3.语音交互玩法课程\第5课 语音控制色块分拣
import sys
import rospy
import signal
import roslibpy
from armpi_fpv_common import tts, asr

# 语音控制分捡
# 运行指令：python asr_control_sorting.py

command = {'2':'red',
           '3':'green',
           '4':'blue',
           '5':'tag1',
           '6':'tag2',
           '7':'tag3',
           '8':''}

rospy.init_node('asr_control_sorting')

running = True
def shutdown(signum, frame):
    global running
    
    running = False
    service = roslibpy.Service(client, '/object_sorting/exit', 'std_srvs/Trigger')
    request = roslibpy.ServiceRequest()
    result = service.call(request)
    rospy.loginfo('shutdown')
    rospy.signal_shutdown('shutdown')

signal.signal(signal.SIGINT, shutdown)

# 后台已经运行与app端通信的rosbridge，我们只需要仿照app端写客户端即可
client = roslibpy.Ros(host='127.0.0.1', port=9090)
client.run()

if client.is_connected:
    rospy.loginfo('connect success')
    try:
        my_asr = asr.ASR()
        my_tts = tts.TTS()

        debug = True
        if debug:
            my_asr.eraseWords()
            my_asr.setMode(2)
            my_asr.addWords(1, 'kai shi')
            my_asr.addWords(2, 'fen jian hong se')
            my_asr.addWords(3, 'fen jian lv se')
            my_asr.addWords(4, 'fen jian lan se')
            my_asr.addWords(5, 'fen jian biao qian yi')
            my_asr.addWords(6, 'fen jian biao qian er')
            my_asr.addWords(7, 'fen jian biao qian san')
            my_asr.addWords(8, 'ting zhi fen jian')
    
        data = my_asr.getResult()
        my_tts.TTSModuleSpeak('[h0][v10]', '准备就绪')
        rospy.sleep(2)
        print('''
口令: 开始
指令2：分捡红色
指令3：分捡绿色
指令4：分捡蓝色
指令5：分捡标签一
指令6：分捡标签二
指令7：分捡标签三
指令8：停止分捡
''')
    except BaseException as e:
        rospy.loginfo('sensor init failed: %s', e)
        sys.exit(0)
else:
    rospy.loginfo('connect failed')
    sys.exit(0)

cmd = {'2':'分捡红色',
       '3':'分捡绿色',
       '4':'分捡蓝色',
       '5':'分捡标签一',
       '6':'分捡标签二',
       '7':'分捡标签三',
       '8':'停止分捡'}

service = roslibpy.Service(client, '/object_sorting/enter', 'std_srvs/Trigger')
request = roslibpy.ServiceRequest()
result = service.call(request)

while running:
    try:
        data = my_asr.getResult()
        if data:
            rospy.loginfo('指令{}:{}\n'.format(data, cmd[str(data)]))
            my_tts.TTSModuleSpeak('', '收到, ' + cmd[str(data)])
            rospy.sleep(1)

            color = ''
            tag = ''
            if str(data) in command:
                object_data = command[str(data)]
                if 'tag' in object_data:
                    tag = object_data
                else:
                    color = object_data

            if color == '' and tag == '':
                service = roslibpy.Service(client, '/object_sorting/set_running', 'std_srvs/SetBool')
                request = roslibpy.ServiceRequest({'data':False})
            else:
                service = roslibpy.Service(client, '/object_sorting/set_target', 'object_sorting/SetTarget')
                request = roslibpy.ServiceRequest({'color':[color], 'tag':[tag]})
                result = service.call(request)
                
                service = roslibpy.Service(client, '/object_sorting/set_running', 'std_srvs/SetBool')
                request = roslibpy.ServiceRequest({'data':True})

            result = service.call(request)
        else:
            rospy.sleep(0.01)
    except BaseException as e:
        rospy.loginfo('error: %s', e)
