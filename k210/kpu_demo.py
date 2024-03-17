import sensor, image, time, math, lcd
from machine import UART
from fpioa_manager import fm
import json
import gc
import KPU as kpu
from Maix import GPIO
import os
import gc
import Maix

print(os.listdir("/"))

lcd.init()
sensor.reset(freq=20000000)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 如果分辨率大得多，内存就不够用了……
sensor.skip_frames(time=2000)
sensor.set_vflip(1)
clock = time.clock()

# 映射串口引脚
fm.register(6, fm.fpioa.UART1_RX, force=True)
fm.register(7, fm.fpioa.UART1_TX, force=True)
fm.register(16, fm.fpioa.GPIO1)
KEY = GPIO(GPIO.GPIO1, GPIO.IN)
# 初始化串口
uart = UART(UART.UART1, 115200, read_buf_len=4096)
uart.write("Hello pico! K210 restart")


f_x = (2.8 / 3.984) * 160  # find_apriltags 如果没有设置，则默认为这个
f_y = (2.8 / 2.952) * 120  # find_apriltags 如果没有设置，则默认为这个
c_x = 160 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.w * 0.5)
c_y = 120 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.h * 0.5)

# kpu
anchors = [2.41, 2.62, 1.06, 1.12, 1.94, 2.0, 1.41, 1.53, 0.59, 0.75]
model_addr = "/sd/duck_in128.kmodel"
# model_addr = "/sd/m.kmodel"
labels = ["red_box", "duck"]

# img_mode = "find_apriltags"
img_mode = "kpu"
task = None
task = kpu.load(model_addr)
kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)

def degrees(radians):
    return (180 * radians) / math.pi


while True:
    print("stack mem", gc.mem_free() / 1024) # stack mem
    print("heap mem", Maix.utils.heap_free() / 1024) # heap mem
    uart_send_data = {}
    clock.tick()
    uart_json = {}
    img = None
    try:
        read_data = uart.read(256)
        if read_data:
            read_str = read_data.decode("utf-8")
            # 查找{和}第一个出现的位置，然后截取字符串
            if "{" in read_str and "}" in read_str:
                read_str = read_str[read_str.index("{") : read_str.index("}") + 1]
                uart_json = json.loads(read_str)

        if "img_mode" in uart_json:
            img_mode = uart_json["img_mode"]
            if img_mode not in ["kpu", "find_apriltags"]:
                img_mode = "kpu"
            gc.collect()
            img = None
            print("uart img_mode:", img_mode)

    except Exception as e:
        print("uart read error:", e)
        uart_json = {}
    print("current mode: ", img_mode)
    if KEY.value() == 0:
        gc.collect()
        if img_mode == "kpu":
            img_mode = "find_apriltags"
        elif img_mode == "find_apriltags":
            img_mode = "kpu"
        time.sleep(1)
    if img_mode == "find_apriltags":
        # if task is not None:
        #     kpu.deinit(task)
        #     gc.collect()
        img = sensor.snapshot()

        apriltags = img.find_apriltags(
            fx=f_x, fy=f_y, cx=c_x, cy=c_y
        )  # defaults to TAG36H11
        uart_send_data["TagStatus"] = "none"

        for tag in apriltags:
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))

            uart_send_data["TagStatus"] = "get"
            uart_send_data["TagId"] = tag.id()
            uart_send_data["TagTx"] = tag.x_translation()
            uart_send_data["TagTy"] = tag.y_translation()
            uart_send_data["TagTz"] = tag.z_translation()
            uart_send_data["TagCx"] = tag.cx()
            uart_send_data["TagCy"] = tag.cy()
    elif img_mode == "kpu":
        # if task is None:
        #     task = kpu.load(model_addr)
        #     kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)
        img = sensor.snapshot()
        img_size = 128
        img2 = img.resize(img_size, img_size)
        img2.pix_to_ai()
        objects = kpu.run_yolo2(task, img2)
        del img2
        if objects:
            for obj in objects:
                min_w = 999
                min_pos = None
                if labels[obj.classid()] == "duck":
                    pos = obj.rect()
                    pos = (
                        int(pos[0] * img.width() / img_size),
                        int(pos[1] * img.height() / img_size),
                        int(pos[2] * img.width() / img_size),
                        int(pos[3] * img.height() / img_size),
                    )
                    if pos[2] < min_w:
                        min_w = pos[2]
                        min_pos = pos

                    uart_send_data["DuckStatus"] = "get"
                uart_send_data["DuckX"] = min_pos[0]
                uart_send_data["DuckY"] = min_pos[1]
                uart_send_data["DuckWidth"] = min_pos[2]
                uart_send_data["DuckHeight"] = min_pos[3]

                img.draw_rectangle(min_pos, color=(0, 255, 255))
    lcd.display(img)

    uart_send_data_json = json.dumps(uart_send_data)
    print("uart send data",uart_send_data_json)
    uart.write(uart_send_data_json)  # 数据回传


    # print(clock.fps())
