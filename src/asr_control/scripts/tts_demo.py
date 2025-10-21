#!/usr/bin/python3
# coding=utf8
# 第13章 ArmPi FPV创意玩法课程\3.语音交互玩法课程\第3课 语音模块测试\2.树莓派语音合成实验
import time
from armpi_fpv_common import tts

v = tts.TTS()
#[h0]设置单词发音方式，0为自动判断单词发音方式，1为字母发音方式，2为单词发音方式
#[v10]设置音量，音量范围为0-10,10为最大音量。
#注意括号里的字符长度不能超过32,如果超过了请分多次来说
v.TTSModuleSpeak("[h0][v10]","你好,我是语音合成模块")
time.sleep(3) # 必要延时，等待播放完成
v.TTSModuleSpeak("[v8]", "你好，我是语音合成模块")
time.sleep(3)
v.TTSModuleSpeak("[v3]", "你好，我是语音合成模块")
