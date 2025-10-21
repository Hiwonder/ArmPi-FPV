#!/usr/bin/env python3
# coding=utf8
import time
import smbus2
# 幻尔科技语音合成模块使用例程#

class TTS:
    TTS_MODULE_I2C_ADDR =  0x40 #传感器的IIC地址
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)

    def TTSModuleSpeak(self, sign, words):
        head = [0xFD,0x00,0x00,0x01,0x00]             #文本播放命令
        wordslist = words.encode("gb2312")            #将文本编码格式设为GB2312
        signdata = sign.encode("gb2312")    
        length = len(signdata) + len(wordslist) + 2
        head[1] = length >> 8
        head[2] = length
        head.extend(list(signdata))
        head.extend(list(wordslist))      
        write = smbus2.i2c_msg.write(self.TTS_MODULE_I2C_ADDR, head)  
        self.bus.i2c_rdwr(write)
        time.sleep(0.05)
        
if __name__ == '__main__':
    v = TTS()
    #[h0]设置单词发音方式，0为自动判断单词发音方式，1为字母发音方式，2为单词发音方式
    #[v10]设置音量，音量范围为0-10,10为最大音量。
    #注意括号里的字符长度不能超过32,如果超过了请分多次来说
    v.TTSModuleSpeak("[h0][v10]","你好,我是语音合成模块")
    time.sleep(3) # 必要延时，等待播放完成
    v.TTSModuleSpeak("[v8]", "你好，我是语音合成模块")
    time.sleep(3)
    v.TTSModuleSpeak("[v3]", "你好，我是语音合成模块")
