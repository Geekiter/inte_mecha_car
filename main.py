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


# ## grab claw
maxGrabLevel = 70
minGrabLevel = 30
grabLastState = 2
grabState = 0  # 0 unknown, 1, close, 2, open
currentGrabLevel = 70

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
    global grabLastState
    global grabState
    global currentGrabLevel
    global maxGrabLevel
    global minGrabLevel

    if grabLastState != 1:
        currentGrabLevel = maxGrabLevel

    else:
        currentGrabLevel = max(currentGrabLevel - 10, minGrabLevel)

    print(f"close Claw at {currentGrabLevel}")
    RotateBCCW(currentGrabLevel)
    sleep(0.5)
    StopMotor()
    sleep(0.2)
    grabLastState = 1


def closeClaw():
    global grabLastState
    global grabState
    global currentGrabLevel
    global maxGrabLevel
    global minGrabLevel

    if grabLastState != 2:
        currentGrabLevel = maxGrabLevel

    else:
        currentGrabLevel = max(currentGrabLevel - 10, minGrabLevel)

    print(f"open Claw at {currentGrabLevel}")

    RotateBCW(currentGrabLevel)
    sleep(0.5)
    StopMotor()
    sleep(0.2)
    grabLastState = 2


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
target_index = 0
target_id_list = ["", 11, "", 11, ""]

target_action_list = [
    "grab-by-color",
    "put-down",
    "grab-by-color",
    "put-down",
    "finished",
]
"""
action status: 

- locate: locate the object by the big tag.
- grab: grab the object by tag
- finish: finish the task
- grab-by-color: grab the object of the specific color
- put-down: put down the object
"""

grab_color = "red"

test_mode = False

big_tag_id_list = [
    21,
]
small_tag_id_list = [32, 11]
small_tag_width = 5.1
small_tag_zoomfactor = 19.5 * 2 / 12.5 / 5.1 * small_tag_width
big_tag_zoomfactor = 78 / 6.6

object_width = 6  # cm
color_obj_zoomfactor = 19.5 * 2 * 27 / 6 * object_width
object_height = 8  # cm
color_obj_zoomfactor_h = 19.5 * 2 * 36 / 8 * object_width

k210_cam_offset = 85 - 160 / 2  # 相机安装在机械臂上的偏移量
claw_range = (90 - 75) / 2
k210_qqvga = (120, 160)
k210_center = k210_qqvga[1] / 2 + k210_cam_offset  # QQVGA分辨率：120*160
k210_y_center = k210_qqvga[0] / 2
arm_range = 20  # pixel 上下浮动范围
rotate_in_front_of_obj = 1  # cm 在物体前方允许旋转的距离


def get_zf(id):
    if id in big_tag_id_list:
        return big_tag_zoomfactor
    else:
        return small_tag_zoomfactor


claw_open_len = 14  # cm
claw_close_len = 15

claw_grab_len = 11
claw_arm_up_len = 20  # 大于这个高度，需要抬起机械臂
grab_mode = False
put_down_obj = False
arm_up_len = 2
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

for _ in range(8):
    openClaw()
    # closeClaw()
    # pass


# 判断当前应该前进还是后退，还是转弯，还是抓取
def get_action(cx, cy, w, h):
    global is_finished
    global grab_mode
    obj_dis_w = color_obj_zoomfactor / w
    obj_dis = color_obj_zoomfactor_h / h
    print(f"obj_dis: {obj_dis}")
    # 如果cx大于k210_center + claw_range，说明物体在右边，右转

    # 如果obj_dis > rotate_in_front_of_obj + claw_open_len，说明物体在前方，调整角度
    if (obj_dis > rotate_in_front_of_obj + claw_open_len and not grab_mode) or (
        obj_dis_w > rotate_in_front_of_obj + claw_open_len and not grab_mode
    ):
        if cy < k210_y_center - 1.5 * arm_range:
            print("view is low, arm up")
            armUp(25)
            sleep(0.3)
        elif cy > k210_y_center + 1.5 * arm_range:
            print("view is high, arm down")
            armDown(10)
            sleep(0.3)
        elif cx > k210_center + claw_range:
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
        grab_mode = True
        if h > claw_arm_up_len:
            armUp(25)
            sleep(1)
            print("arm up, and h is: ", h)
        else:
            print("forward to grab")
            # moveForwardSpd(30)
            for _ in range(2):
                keepForward(25)
                sleep(0.3)
        # if obj_dis < claw_grab_len:
        #     print("grab")
        #     for _ in range(6):
        #         closeClaw()
        #         # is_finished = True
        # else:
        #     grab_mode = True
        #     if h > claw_arm_up_len:
        #         armUp(25)
        #         sleep(1)
        #         print("arm up, and h is: ", h)
        #     else:
        #         print("forward to grab")
        #         # moveForwardSpd(30)
        #         for _ in range(2):
        #             keepForward(25)


def get_tag_action(tag_x, tag_y, tag_z):
    global put_down_obj
    global target_index
    if tag_z > claw_close_len + arm_up_len:
        if tag_y < k210_y_center - arm_range:
            print("view high is low, arm up")
            armUp(25)
            sleep(0.3)
        elif tag_y > k210_y_center + arm_range:
            print("view high is high, arm down")
            armDown(10)
            sleep(0.3)
        elif tag_x > k210_center + claw_range:
            print("right")
            keepTurnRight(30)
        elif tag_x < k210_center - claw_range:
            print("left")
            keepTurnLeft(30)
        else:
            print("forward")
            keepForward(30)
    else:
        for _ in range(3):
            armUp(30)
            sleep(0.3)

        # for _ in range(8):
        #     keepForward(30)

        for _ in range(4):
            openClaw()

        for _ in range(10):
            keepBackward(30)

        for _ in range(10):
            armDown(10)
            sleep(0.3)

        target_index += 1


# 核心逻辑
while is_finished is False and not test_mode:
    if uart2.any():
        # print(f"current target id is: {target_id_list[target_index]}")
        try:
            uart2_data = uart2.read().decode("utf-8")
            print(f"uart2_data: {uart2_data}")
            # 查找}第一个出现的位置，然后截取字符串
            json_end_index = uart2_data.find("}")
            if json_end_index == -1:
                continue
            else:
                uart2_data = uart2_data[: uart2_data.find("}") + 1]
            data = json.loads(uart2_data)
        except Exception as e:
            print("urat2 data error", e)
            data = {}
        if target_index < len(target_action_list):
            print(f"current target action is: {target_action_list[target_index]}")
        else:
            print("target action list is empty")
            break

        if target_action_list[target_index] == "put-down":
            tag_id = data.get("TagId", "N/A")
            print(f"tag_id: {tag_id}")
            if tag_id == target_id_list[target_index]:
                zoomfactor = get_zf(tag_id)
                print(f"received tag id:{tag_id}")
                print(
                    f"x: {data.get('TagTx', '999')}, y: {data.get('TagTy', '999')}, z: {data.get('TagTz', '999')}"
                )
                tag_z = int(-zoomfactor * float(data.get("TagTz", "999")))
                tag_x = int(zoomfactor * float(data.get("TagTx", "999")))
                tag_y = int(-zoomfactor * float(data.get("TagTy", "999")))
                tag_cx = int(data.get("TagCx", "999"))
                tag_cy = int(data.get("TagCy", "999"))
                print(f"tag_z: {tag_z}, tag_cx: {tag_cx}, tag_cy: {tag_cy}")
                get_tag_action(tag_cx, tag_cy, tag_z)
            else:
                keepTurnRight(30)
                sleep(0.3)
        elif target_action_list[target_index] == "grab-by-color":
            obj_status = data.get("ObjectStatus", "N/A")

            if obj_status == "get":

                def get_obj_data(key):
                    global data
                    try:
                        return int(data.get(key, 0) if obj_status == "get" else 0)
                    except Exception as e:
                        print("get obj data err:", e)
                        print(data.get(key, 0))
                        return 0

                obj_w = get_obj_data("ObjectWidth")
                obj_h = get_obj_data("ObjectHeight")
                obj_x = get_obj_data("ObjectX")
                obj_y = get_obj_data("ObjectY")
                obj_cx = get_obj_data("ObjectCX")
                obj_cy = get_obj_data("ObjectCY")
                # print(
                #     f"obj_status: {obj_status}, obj_w: {obj_w}, obj_h: {obj_h}, obj_x: {obj_x}, obj_y: {obj_y}, obj_cx: {obj_cx}, obj_cy: {obj_cy}"
                # )

                get_action(obj_cx, obj_cy, obj_w, obj_h)
            if obj_status == "none":
                if grab_mode:
                    for _ in range(5):
                        closeClaw()
                    for _ in range(5):
                        armUp(30)
                    for _ in range(8):
                        keepBackward(30)
                    # for _ in range(15):
                    #     armDown(10)
                    target_index += 1
                    grab_mode = False
                else:
                    keepTurnRight(30)
                    sleep(0.3)

        elif target_action_list[target_index] == "finished":
            is_finished = True
            break
