import pyrealsense2 as rs
# import pyrealsense2.pyrealsense2 as rs
import numpy as np
from serial.tools import list_ports
from pydobot import Dobot

import os
import time

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)
(x, y, z, r, j1, j2, j3, j4) = device.pose()
print("#################dobot pose")
print("x: {}, y: {}, z: {}, r: {}, j1: {}, j2: {}, j3: {}, j4: {}".format(x, y, z, r, j1, j2, j3, j4))
# move to calibration position

# device.gohome()
# time.sleep(30)
device.move_to(x=200, y=0, z=0, r=-30, wait=True)

device.grip(enable=False)


while True:
    var = int(input("请输入grip指令：【1】打开 【0】关闭\n"))

    if var == 0:
        device.grip(enable=True)
    elif var == 1:
        device.grip(enable=False)
    else:
        print("输入有误")


