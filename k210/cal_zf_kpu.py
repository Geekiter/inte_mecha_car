import sensor, image, time, math, lcd
from machine import UART
from fpioa_manager import fm
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
uart.write("Hello pico! K210 start")
# kpu
anchors = [2.41, 2.62, 1.06, 1.12, 1.94, 2.0, 1.41, 1.53, 0.59, 0.75]
model_addr = "/sd/duck_in128.kmodel"
# model_addr = "/sd/m.kmodel"
labels = ["red_box", "duck"]
task = None
task = kpu.load(model_addr)
kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)

real_dis = 14.2  # cm

while True:
    # print("stack mem", gc.mem_free() / 1024) # stack mem
    # print("heap mem", Maix.utils.heap_free() / 1024) # heap mem

    img = sensor.snapshot()
    img_size = 128
    img2 = img.resize(img_size, img_size)
    img2.pix_to_ai()
    objects = kpu.run_yolo2(task, img2)
    del img2

    if objects:
        for obj in objects:
            if labels[obj.classid()] == "duck":
                pos = obj.rect()
                # 将224*224的坐标转换为原始图像的坐标
                pos = (
                    int(pos[0] * img.width() / img_size),
                    int(pos[1] * img.height() / img_size),
                    int(pos[2] * img.width() / img_size),
                    int(pos[3] * img.height() / img_size),
                )


                img.draw_rectangle(pos, color=(0, 255, 255))

                zf = real_dis * pos[2]
                img.draw_string(0, 0, "zf:%.2f" % zf, color=(255, 0, 0), scale=2)
                print("zf:%.2f" % zf)
    lcd.display(img)