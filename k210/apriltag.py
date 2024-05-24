import sensor, image, time, math, lcd
from machine import UART, Timer
from fpioa_manager import fm
import json

lcd.init()
sensor.reset(freq=20000000)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 如果分辨率大得多，内存就不够用了……
sensor.skip_frames(time=2000)
#sensor.set_windowing((240, 240))
# sensor.set_auto_gain(False)  # 必须关闭此功能，以防止图像冲洗…
# sensor.set_auto_whitebal(False)  # 必须关闭此功能，以防止图像冲洗…
sensor.set_vflip(1)
clock = time.clock()

# 映射串口引脚
fm.register(6, fm.fpioa.UART1_RX, force=True)
fm.register(7, fm.fpioa.UART1_TX, force=True)

# 初始化串口
uart = UART(UART.UART1, 115200, read_buf_len=4096)
uart.write("Hello pico! K210 restart")


f_x = (2.8 / 3.984) * 160  # find_apriltags 如果没有设置，则默认为这个
f_y = (2.8 / 2.952) * 120  # find_apriltags 如果没有设置，则默认为这个
c_x = 160 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.w * 0.5)
c_y = 120 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.h * 0.5)


def degrees(radians):
    return (180 * radians) / math.pi


while True:
    uart_send_data = {}
    clock.tick()
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

    uart_send_data_json = json.dumps(uart_send_data)
    print(uart_send_data_json)
    uart.write(uart_send_data_json)  # 数据回传

    lcd.display(img)
    # print(clock.fps())
