# Untitled - By: integem - Tue Nov 21 2023
# AprilTags3D定位例程

import sensor, image, time, math, lcd
from machine import UART, Timer
from fpioa_manager import fm
import json

lcd.init()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 如果分辨率大得多，内存就不够用了……
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # 必须关闭此功能，以防止图像冲洗…
sensor.set_auto_whitebal(False)  # 必须关闭此功能，以防止图像冲洗…
sensor.set_vflip(1)
clock = time.clock()

# 映射串口引脚
fm.register(6, fm.fpioa.UART1_RX, force=True)
fm.register(7, fm.fpioa.UART1_TX, force=True)

# 初始化串口
uart = UART(UART.UART1, 115200, read_buf_len=4096)
uart.write("Hello pico! K210 restart")


# 注意！与find_qrcodes不同，find_apriltags方法不需要对镜像进行镜头校正。

# 标签系列有什么区别？ 那么，例如，TAG16H5家族实际上是一个4x4的方形标签。
# 所以，这意味着可以看到比6x6的TAG36H11标签更长的距离。
# 然而，较低的H值（H5对H11），意味着4x4标签的假阳性率远高于6x6标签。
# 所以，除非你有理由使用其他标签系列，否则使用默认族TAG36H11。


# AprilTags库输出标签的姿势信息。 这是x / y / z平移和x / y / z旋转。
# x / y / z旋转以弧度表示，可以转换为度数。 至于翻译单位是无量纲的，
# 你必须应用一个转换函数。

# f_x是相机的x焦距。它应该等于以mm为单位的镜头焦距除以x传感器尺寸（以mm为单位）乘以图像中的像素数。
# 以下数值适用于配备2.8毫米镜头的OV7725相机。

# f_y是相机的y焦距。它应该等于以mm为单位的镜头焦距除以y传感器尺寸（以mm为单位）乘以图像中的像素数。
# 以下数值适用于配备2.8毫米镜头的OV7725相机。

# c_x是以像素为单位的图像x中心位置
# c_x是以像素为单位的图像x中心位置

f_x = (2.8 / 3.984) * 160  # find_apriltags 如果没有设置，则默认为这个
f_y = (2.8 / 2.952) * 120  # find_apriltags 如果没有设置，则默认为这个
c_x = 160 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.w * 0.5)
c_y = 120 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.h * 0.5)

thresholds = [(72, 20, 127, 41, 56, 6)]
# 蓝色盒子阈值 (23, 100, -128, -10, -40, -1)
# 红色盒子阈值 (72, 20, 127, 41, 56, 6)


def degrees(radians):
    return (180 * radians) / math.pi


while True:
    urat_send_data = {}
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([thresholds[0]], pixels_threshold=100)

    color_status = "none"
    object_info = "none"
    urat_send_data["ObjectWidth"] = 0
    urat_send_data["ObjectHeight"] = 0
    urat_send_data["ObjectStatus"] = color_status
    if blobs:
        color_status = "get"
        max_pixel = 0
        b_index = 0
        for b in blobs:
            if b[4] > max_pixel:
                max_pixel = b_index
        object_info = blobs[b_index]
        tmp = img.draw_rectangle(blobs[b_index][0:4])
        tmp = img.draw_cross(
            blobs[b_index].x() + int(object_info[2] / 2),
            blobs[b_index].y() + int(object_info[3] / 2),
        )
        urat_send_data["ObjectStatus"] = color_status
        urat_send_data["ObjectX"] = object_info.x()
        urat_send_data["ObjectY"] = object_info.y()
        urat_send_data["ObjectCX"] = blobs[b_index].x() + int(object_info[2] / 2)
        urat_send_data["ObjectCY"] = blobs[b_index].y() + int(object_info[3] / 2)
        urat_send_data["ObjectWidth"] = object_info[2]
        urat_send_data["ObjectHeight"] = object_info[3]

    apriltags = img.find_apriltags(
        fx=f_x, fy=f_y, cx=c_x, cy=c_y
    )  # defaults to TAG36H11

    for tag in apriltags:
        img.draw_rectangle(tag.rect(), color=(255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))
        print_args = (
            tag.id(),
            tag.x_translation(),
            tag.y_translation(),
            tag.z_translation(),
            degrees(tag.x_rotation()),
            degrees(tag.y_rotation()),
            degrees(tag.z_rotation()),
            color_status,
        )
        # 变换单位不详。旋转单位是度数。
        print(
            "id: %d, Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f, Color: %s" % print_args
        )
        # output1="id:"+str(tag.id())+",Tx: "+str(tag.x_translation())+",Ty: "+str(tag.y_translation())+",Tz: "+str(tag.z_translation())+",Color:"+str(color_status)+"\n"
        # dict to json
        urat_send_data["TagId"] = tag.id()
        urat_send_data["TagTx"] = tag.x_translation()
        urat_send_data["TagTy"] = tag.y_translation()
        urat_send_data["TagTz"] = tag.z_translation()

    urat_send_data_json = json.dumps(urat_send_data)
    print(urat_send_data_json)
    uart.write(urat_send_data_json)  # 数据回传

    lcd.display(img)
    # print(clock.fps())
