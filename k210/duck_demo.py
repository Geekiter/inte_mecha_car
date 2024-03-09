# generated by maixhub, tested on maixpy3 v0.4.8
# copy files to TF card and plug into board and power on
import sensor, image, lcd, time
import KPU as kpu
import gc, sys
from fpioa_manager import fm
from machine import UART, Timer
import json
from Maix import GPIO, utils
from fpioa_manager import fm

input_size = (224, 224)
labels = ["red_box", "duck"]
anchors = [2.41, 2.62, 1.06, 1.12, 1.94, 2.0, 1.41, 1.53, 0.59, 0.75]
img_mode = "kpu"


fm.register(16, fm.fpioa.GPIO1)
KEY = GPIO(GPIO.GPIO1, GPIO.IN)



def lcd_show_except(e):
    import uio

    err_str = uio.StringIO()
    sys.print_exception(e, err_str)
    err_str = err_str.getvalue()
    img = image.Image(size=input_size)
    img.draw_string(0, 10, err_str, scale=1, color=(0xFF, 0x00, 0x00))
    lcd.display(img)


def init_uart():
    # 映射串口引脚
    fm.register(6, fm.fpioa.UART1_RX, force=True)
    fm.register(7, fm.fpioa.UART1_TX, force=True)
    # 初始化串口
    uart = UART(UART.UART1, 115200, read_buf_len=4096)
    uart.write("Hello pico! K210 restart")
    return uart


thresholds = [(72, 20, 127, 41, 56, 6)]


def degrees(radians):
    return (180 * radians) / math.pi


def main(
    anchors,
    labels=None,
    model_addr="/sd/m.kmodel",
    sensor_window=input_size,
    lcd_rotation=0,
    sensor_hmirror=True,
    sensor_vflip=True,
):
    global uart
    global img_mode
    f_x = (2.8 / 3.984) * 160  # find_apriltags 如果没有设置，则默认为这个
    f_y = (2.8 / 2.952) * 120  # find_apriltags 如果没有设置，则默认为这个
    c_x = 160 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.w * 0.5)
    c_y = 120 * 0.5  # find_apriltags 如果没有设置，则默认为这个 (the image.h * 0.5)
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing(sensor_window)
    sensor.set_hmirror(sensor_hmirror)
    sensor.set_vflip(sensor_vflip)
    sensor.run(1)

    lcd.init(type=1)
    lcd.rotation(lcd_rotation)
    lcd.clear(lcd.WHITE)

    if not labels:
        with open("labels.txt", "r") as f:
            exec(f.read())

    if not labels:
        print("no labels.txt")
        img = image.Image(size=(320, 240))
        img.draw_string(90, 110, "no labels.txt", color=(255, 0, 0), scale=2)
        lcd.display(img)
        return 1
    try:
        img = image.Image("startup.jpg")
        lcd.display(img)
    except Exception:
        img = image.Image(size=(320, 240))
        img.draw_string(90, 110, "loading model...", color=(255, 255, 255), scale=2)
        lcd.display(img)

    uart = init_uart()
    uart_send_data = {}
    uart_json = {}
    task = None
    try:

        while True:
            print("gc heap size: ",utils.gc_heap_size())
            print("current mode: ", img_mode)
            try:
                read_data = uart.read(256)
                if read_data:
                    read_str = read_data.decode("utf-8")
                    # 查找{和}第一个出现的位置，然后截取字符串
                    if "{" in read_str and "}" in read_str:
                        read_str = read_str[read_str.index("{") : read_str.index("}") + 1]
                        print("read_str:", read_str)
                        uart_json = json.loads(read_str)

                if "img_mode" in uart_json:
                    img_mode = uart_json["img_mode"]
                    print("uart img_mode:", img_mode)
            except Exception as e:
                print(e)
                uart_json = {}



            if KEY.value() == 0:
                if img_mode == "kpu":
                    img_mode = "find_blobs"
                elif img_mode == "find_blobs":
                    img_mode = "find_apriltags"
                elif img_mode == "find_apriltags":
                    img_mode = "kpu"
                time.sleep(1)

            t = time.ticks_ms()
            t = time.ticks_ms() - t

            blobs = None

            if img_mode == "kpu":
                if task is None:
                    task = kpu.load(model_addr)
                    kpu.init_yolo2(
                        task, 0.5, 0.3, 5, anchors
                    )  # threshold:[0,1], nms_value: [0, 1]
                img = sensor.snapshot()
                objects = kpu.run_yolo2(task, img)
                if objects:
                    for obj in objects:
                        pos = obj.rect()
                        print(pos[0], pos[1], pos[2], pos[3])
                        if labels[obj.classid()] == "duck":
                            uart_send_data['DuckStatus'] = "get"
                            uart_send_data['DuckX'] = pos[0]
                            uart_send_data['DuckY'] = pos[1]
                            uart_send_data['DuckWidth'] = pos[2]
                            uart_send_data['DuckHeight'] = pos[3]
                        else:
                            uart_send_data['DuckStatus'] = "none"
                        img.draw_rectangle(pos, color=(0, 255, 255))
                        img.draw_string(
                            pos[0],
                            pos[1],
                            "%s : %.2f" % (labels[obj.classid()], obj.value()),
                            scale=2,
                            color=(255, 0, 0),
                        )
                else:
                    uart_send_data['DuckStatus'] = "none"
                #kpu.deinit(task)
                #img.draw_string(0, 200, "t:%dms" % (t), scale=2, color=(255, 0, 0))
            elif img_mode == "find_blobs":
                if task is not None:
                    kpu.deinit(task)
                    del task
                    gc.collect()
                    task = None
                # color_status = "none"
                # object_info = "none"
                # uart_send_data["ObjectWidth"] = 0
                # uart_send_data["ObjectHeight"] = 0
                # uart_send_data["ObjectStatus"] = color_status
                img = sensor.snapshot()
                blobs = img.find_blobs([thresholds[0]], pixels_threshold=100)
                if blobs:
                    color_status = "get"
                    max_pixel = 0
                    b_index = 0
                    for b in blobs:
                        if b[4] > max_pixel:
                            max_pixel = b_index
                    object_info = blobs[b_index]
                    img.draw_rectangle(blobs[b_index][0:4])
                    img.draw_cross(
                        blobs[b_index].x() + int(object_info[2] / 2),
                        blobs[b_index].y() + int(object_info[3] / 2),
                    )
                    uart_send_data["ObjectStatus"] = color_status
                    uart_send_data["ObjectX"] = object_info.x()
                    uart_send_data["ObjectY"] = object_info.y()
                    uart_send_data["ObjectCX"] = blobs[b_index].x() + int(
                        object_info[2] / 2
                    )
                    uart_send_data["ObjectCY"] = blobs[b_index].y() + int(
                        object_info[3] / 2
                    )
                    uart_send_data["ObjectWidth"] = object_info[2]
                    uart_send_data["ObjectHeight"] = object_info[3]
                else:
                    uart_send_data["ObjectStatus"] = "none"
                    
            elif img_mode == "find_apriltags":
                if task is not None:
                    kpu.deinit(task)
                    del task
                    gc.collect()
                    task = None
                img = sensor.snapshot()
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
                        "id: %d, Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f, Color: %s"
                        % print_args
                    )
                    # output1="id:"+str(tag.id())+",Tx: "+str(tag.x_translation())+",Ty: "+str(tag.y_translation())+",Tz: "+str(tag.z_translation())+",Color:"+str(color_status)+"\n"
                    # dict to json
                    uart_send_data["TagId"] = tag.id()
                    uart_send_data["TagTx"] = tag.x_translation()
                    uart_send_data["TagTy"] = tag.y_translation()
                    uart_send_data["TagTz"] = tag.z_translation()
                    uart_send_data["TagCx"] = tag.cx()
                    uart_send_data["TagCy"] = tag.cy()

            uart_send_data_json = json.dumps(uart_send_data)
            print(uart_send_data_json)
            uart.write(uart_send_data_json)
            lcd.display(img)
    except Exception as e:
        print(e)
        raise e
    finally:
        if not task is None:
            kpu.deinit(task)


if __name__ == "__main__":
    try:
        # main(anchors = anchors, labels=labels, model_addr=0x300000, lcd_rotation=0)
        main(anchors=anchors, labels=labels, model_addr="/sd/model-104690.kmodel")
    except Exception as e:
        sys.print_exception(e)
        lcd_show_except(e)
    finally:
        gc.collect()
