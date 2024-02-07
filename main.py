from machine import Pin, PWM, ADC
from utime import sleep
import re
import json
from umqtt.simple import MQTTClient
import time
import os
from machine import Timer, Pin, PWM, ADC, UART
import utime
import neopixel
import random
import machine
from utime import sleep

# http server
from micropyserver import MicroPyServer
import utils
from motorPCA9685 import MotorDriver

# # ----------------- fixed parameters -----------------
# ## 初始化UART
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart2 = UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))
inb1 = Pin(10, Pin.OUT)
inb2 = Pin(9, Pin.OUT)
pwmb = PWM(Pin(11))

m = MotorDriver()

# ## arm
ina1 = Pin(8, Pin.OUT)
ina2 = Pin(7, Pin.OUT)
pwma = PWM(Pin(6))
pwma.freq(1000)


is_finished = False
target_index = 0

# ## grab claw
maxGrabLevel = 70
minGrabLevel = 30

# # ----------------- functions -----------------


def parse_data(data):
    values_list = []
    lines = data.split("\n")

    for line in lines:
        if line.strip():  # Check if the line is not just whitespace
            values = {}
            data_items = line.split(",")
            for item in data_items:
                key, value = item.strip().split(":")
                try:
                    values[key] = float(value) if "." in value else int(value)
                except ValueError:
                    values[key] = value
            values_list.append(values)

    return values_list


def RotateBCCW(duty):
    inb1.value(0)
    inb2.value(1)
    duty_16 = int((duty * 65536) / 100)
    pwmb.duty_u16(duty_16)


def RotateBCW(duty):
    inb1.value(1)
    inb2.value(0)
    duty_16 = int((duty * 65536) / 100)
    pwmb.duty_u16(duty_16)


def RotateACW(duty):
    ina1.value(1)
    ina2.value(0)
    duty_16 = int((duty * 65536) / 100)
    print(duty_16)
    pwma.duty_u16(duty_16)


def RotateACCW(duty):
    ina1.value(0)
    ina2.value(1)
    duty_16 = int((duty * 65536) / 100)
    pwma.duty_u16(duty_16)


def StopMotor():
    ina1.value(0)
    ina2.value(0)
    pwma.duty_u16(0)
    inb1.value(0)
    inb2.value(0)
    pwmb.duty_u16(0)


def openClaw():
    global maxGrabLevel
    RotateBCCW(maxGrabLevel)
    maxGrabLevel -= 2
    sleep(0.5)
    StopMotor()
    sleep(0.2)


def closeClaw():
    global maxGrabLevel
    RotateBCW(maxGrabLevel)
    maxGrabLevel -= 2
    sleep(0.5)
    StopMotor()
    sleep(0.2)


def moveForwardSpd(speedpct1):
    m.MotorRunInstant("MA", "forward", speedpct1)
    m.MotorRunInstant("MB", "forward", speedpct1)
    m.MotorRunInstant("MC", "forward", speedpct1)
    m.MotorRunInstant("MD", "forward", speedpct1)


def moveBackwardSpd(speedpct1):
    m.MotorRunInstant("MA", "backward", speedpct1)
    m.MotorRunInstant("MB", "backward", speedpct1)
    m.MotorRunInstant("MC", "backward", speedpct1)
    m.MotorRunInstant("MD", "backward", speedpct1)


def rotateLeftSpd(speedpct1):
    m.MotorRunInstant("MA", "backward", speedpct1)
    m.MotorRunInstant("MB", "forward", speedpct1)
    m.MotorRunInstant("MC", "backward", speedpct1)
    m.MotorRunInstant("MD", "forward", speedpct1)


def rotateRightSpd(speedpct1):
    m.MotorRunInstant("MA", "forward", speedpct1)
    m.MotorRunInstant("MB", "backward", speedpct1)
    m.MotorRunInstant("MC", "forward", speedpct1)
    m.MotorRunInstant("MD", "backward", speedpct1)


def stopMove():
    m.MotorStop("MA")
    m.MotorStop("MB")
    m.MotorStop("MC")
    m.MotorStop("MD")


def parallelLeft(speedpct):
    m.MotorRunInstant("MA", "forward", speedpct)
    m.MotorRunInstant("MB", "backward", speedpct)
    m.MotorRunInstant("MC", "backward", speedpct)
    m.MotorRunInstant("MD", "forward", speedpct)


def parallelRight(speedpct):
    m.MotorRunInstant("MA", "backward", speedpct)
    m.MotorRunInstant("MB", "forward", speedpct)
    m.MotorRunInstant("MC", "forward", speedpct)
    m.MotorRunInstant("MD", "backward", speedpct)


def armUp(duty_cycle):
    # arm up
    RotateACW(duty_cycle)
    sleep(0.1)
    StopMotor()
    sleep(0.1)


def armDown(duty_cycle):
    # arm down
    RotateACCW(duty_cycle)
    sleep(0.05)
    StopMotor()
    sleep(0.2)


def keepForward(sp):
    moveForwardSpd(sp)
    moveForwardSpd(sp - 5)
    moveForwardSpd(sp - 10)
    stopMove()


def keepTurnLeft(sp):
    rotateLeftSpd(sp)
    rotateLeftSpd(sp - 5)
    rotateLeftSpd(sp - 8)
    rotateLeftSpd(sp - 10)
    stopMove()


def keepTurnRight(sp):
    rotateRightSpd(sp)
    rotateRightSpd(sp - 5)
    rotateRightSpd(sp - 8)
    rotateRightSpd(sp - 10)
    stopMove()


def keepBackward(sp):
    moveBackwardSpd(sp)
    moveBackwardSpd(sp - 5)
    moveBackwardSpd(sp - 10)
    stopMove()

# # ----------------- adjustable parameters -----------------

target_id_list = [21, 32, 21, 11]

target_action_list = ["locate", "grab", "locate", "finish"]
"""
action status: 

- locate: locate the object by the big tag.
- grab: grab the object
- finish: finish the task
"""

grab_color = "red"

test_mode = False

big_tag_id_list = [
    21,
]
small_tag_id_list = [32, 11]
small_tag_zoomfactor = 15 / 4
big_tag_zoomfactor = 78 / 6.6

object_width = 6 # cm
color_obj_zoomfactor = 19.5 * 2 * 27 / 6 * object_width

k210_cam_offset = 95 - 160 / 2  # 相机安装在机械臂上的偏移量
claw_range = (90 - 75) / 2 
k210_center = 160 / 2 + k210_cam_offset
rotate_in_front_of_obj = 2 # cm 在物体前方允许旋转的距离

def get_zf(id):
    if id in big_tag_id_list:
        return big_tag_zoomfactor
    else:
        return small_tag_zoomfactor


claw_open_len = 14  # cm
claw_close_len = 15

claw_grab_len = 11
claw_arm_up_len = 20 # 大于这个高度，需要抬起机械臂
grab_mode = False


# # ----------------- the life cycle of a job -----------------


# ## test
sp = 50
while test_mode:
    # moveForwardSpd(sp)

    # rotateLeftSpd(sp)
    # rotateRightSpd(sp)
    # moveBackwardSpd(sp)
    # armUp(30)
    # armDown(10)
    # openClaw()
    # closeClaw()
    # parallelLeft(sp)
    # stopMove()
    pass

if test_mode:
    for _ in range(3):
        closeClaw()
    for _ in range(3):
        openClaw()

# ### test end
# ### -----------------
# ## main
for _ in range(10):
    armDown(10)
    
for _ in range(5):
    openClaw()
    # closeClaw()
    # pass



# 判断当前应该前进还是后退，还是转弯，还是抓取
def get_action(cx, cy, w, h):
    global is_finished
    global grab_mode
    obj_dis = color_obj_zoomfactor / w
    # 如果cx大于k210_center + claw_range，说明物体在右边，右转

    # 如果obj_dis > rotate_in_front_of_obj + claw_open_len，说明物体在前方，调整角度
    if obj_dis > rotate_in_front_of_obj + claw_open_len:
        if cx > k210_center + claw_range:
            print("right")
            keepTurnRight(30)
            
        # 如果cx小于k210_center - claw_range，说明物体在左边，左转
        elif cx < k210_center - claw_range:
            print("left")
            keepTurnLeft(30)
        else:
            print("forward")
            keepForward(30)
    else:
        if obj_dis < claw_grab_len:
            print("grab")
            for _ in range(6):
                closeClaw()
                is_finished = True
        else:
            grab_mode = True
            if(h > claw_arm_up_len):
                armUp(25)
                sleep(1)
                print("arm up, and h is: ", h)
            else:
                print("forward to grab")
                # moveForwardSpd(30)
                keepForward(30)

# 核心逻辑
while is_finished is False and not test_mode:
    if uart2.any():
        # print(f"current target id is: {target_id_list[target_index]}")

        

        try:
            uart2_data = uart2.read().decode("utf-8")
            data = json.loads(uart2_data)
        except Exception as e:
            print("urat2 data error")
            data = {}
        tag_id = data.get("TagId", "N/A")
        obj_status = data.get("ObjectStatus", "N/A")

        obj_status = data.get("ObjectStatus", "N/A")
        obj_w = int(data.get("ObjectWidth", "N/A")) if obj_status == "get" else 0
        obj_h = int(data.get("ObjectHeight", "N/A")) if obj_status == "get" else 0
        obj_x = int(data.get("ObjectX", "N/A")) if obj_status == "get" else 0
        obj_y = int(data.get("ObjectY", "N/A")) if obj_status == "get" else 0
        obj_cx = int(data.get("ObjectCX", "N/A")) if obj_status == "get" else 0
        obj_cy = int(data.get("ObjectCY", "N/A")) if obj_status == "get" else 0

        if tag_id == target_id_list[target_index]:
            zoomfactor = get_zf(tag_id)
            print(f"received tag id:{tag_id}")
            tag_z = int(-zoomfactor * float(data.get("TagTz", "N/A")))
            tag_x = int(zoomfactor * float(data.get("TagTx", "N/A")))
            tag_y = int(-zoomfactor * float(data.get("TagTy", "N/A")))
            print(f"tag_z: {tag_z}, tag_x: {tag_x}, tag_y: {tag_y}")
        if obj_status == "get":
            # print(
            #     f"obj_status: {obj_status}, obj_w: {obj_w}, obj_h: {obj_h}, obj_x: {obj_x}, obj_y: {obj_y}, obj_cx: {obj_cx}, obj_cy: {obj_cy}"
            # )
            obj_dis = color_obj_zoomfactor / obj_w
            print(f"obj_dis: {obj_dis}")
            get_action(obj_cx, obj_cy, obj_w, obj_h)
        if obj_status == "none" and grab_mode:
            for _ in range(4):
                closeClaw()
            for _ in range(2):
                armUp(30)
            is_finished = True
