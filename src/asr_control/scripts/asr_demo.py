#!/usr/bin/python3
# coding=utf8
# 第13章 ArmPi FPV创意玩法课程\3.语音交互玩法课程\第3课 语音模块测试\1.树莓派语音识别实验
import time
from armpi_fpv_common.asr import ASR

asr = ASR()

#添加的词条和识别模式是可以掉电保存的，第一次设置完成后，可以将1改为0
if 1:
    asr.eraseWords()
    asr.setMode(1)
    asr.addWords(1, 'kai shi')
    asr.addWords(2, 'fen jian hong se')
    asr.addWords(3, 'fen jian lv se')
    asr.addWords(4, 'fen jian lan se')
    asr.addWords(5, 'fen jian biao qian yi')
    asr.addWords(6, 'fen jian biao qian er')
    asr.addWords(7, 'fen jian biao qian san')
    asr.addWords(8, 'ting zhi fen jian')           
while 1:
    data = asr.getResult()
    if data:
        print("result:", data)
    time.sleep(0.01)
