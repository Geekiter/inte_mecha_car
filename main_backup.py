from machine import Pin , PWM, ADC
from utime import sleep

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

# config
# import config
wifiName = "WOW10086"
wifiPassword = "chenkeyan"

# MQTT setting
myTopic = "apriltagSize"
clientID = "car1"

serverIP = "192.168.31.92"
port = 1883

# your device name
machineId = 'car'

# LED
Led_R = PWM(Pin(20))
Led_G = PWM(Pin(19))
Led_B = PWM(Pin(18))

# Initialize ADC
adc = ADC(Pin(26))  # Assuming you're using GPIO 26 for ADC
resistance_ratio=0.27
ADC_MAX = 4095  # Maximum ADC value for 12-bit ADC
V_REF = 3.3  # Reference voltage for the Pico's ADC

# infrared remote
IR_PIN = Pin(3, Pin.IN, Pin.PULL_UP)

grabState=0 #0 unknown, 1, close, 2, open
grabLastState=0
maxGrabLevel=70
minGrabLevel=30
currentGrabLevel=70
# exec_cmd
N = 0

# define for arm and claw control

# ina1 = Pin(7,Pin.OUT)
# ina2 = Pin(8, Pin.OUT)
ina1 = Pin(8,Pin.OUT)
ina2 = Pin(7, Pin.OUT)
pwma = PWM(Pin(6))
pwma.freq(1000)

# inb1 = Pin(9,Pin.OUT)
# inb2 = Pin(10, Pin.OUT)
inb1 = Pin(10,Pin.OUT)
inb2 = Pin(9, Pin.OUT)
pwmb = PWM(Pin(11))

pwmb.freq(1000)

timestamp = int(time.time())
filename1 = f"data_log_{timestamp}.csv"

# Record the start time
start_time = utime.ticks_ms()

def log_data(data, filename=filename1, is_header=False):
    # # If filename is not provided, create one with the current timestamp
    # if filename is None:
    #     timestamp = int(time.time())
    #     filename = f"data_log_{timestamp}.csv"

    # Check if the file exists to determine if we need to write the header
    #file_exists = os.path.isfile(filename)

    with open(filename, 'a') as file:
        # If the file does not exist and is_header is True, write the header
        if is_header:
            header = ','.join(data) + '\n'
            file.write(header)
        else:
            # Write the data
            line = ','.join(str(item) for item in data) + '\n'
            file.write(line)

def RotateACW(duty):
    ina1.value(1)
    ina2.value(0)
    duty_16 = int((duty*65536)/100)
    print(duty_16)
    pwma.duty_u16(duty_16)

def RotateACCW(duty):
    ina1.value(0)
    ina2.value(1)
    duty_16 = int((duty*65536)/100)
    pwma.duty_u16(duty_16)

def RotateBCW(duty):
    inb1.value(1)
    inb2.value(0)
    duty_16 = int((duty*65536)/100)
    pwmb.duty_u16(duty_16)

def RotateBCCW(duty):
    inb1.value(0)
    inb2.value(1)
    duty_16 = int((duty*65536)/100)
    pwmb.duty_u16(duty_16)
    
def StopMotor():
    ina1.value(0)
    ina2.value(0)
    pwma.duty_u16(0)
    inb1.value(0)
    inb2.value(0)
    pwmb.duty_u16(0)

# motor
# motor1 = PWM(Pin(10))
# motor2 = PWM(Pin(11))
# motor3 = PWM(Pin(12))
# motor4 = PWM(Pin(13))
speed = 65534
speedpct=100

# voice
buzzer = PWM(Pin(15))
Tone = [0, 392, 440, 494, 523, 587, 659, 698, 784]

Song = [Tone[3], Tone[3], Tone[3], Tone[3], Tone[3], Tone[3],
        Tone[3], Tone[5], Tone[1], Tone[2], Tone[3],
        Tone[4], Tone[4], Tone[4], Tone[4], Tone[4], Tone[3], Tone[3], Tone[3],
        Tone[5], Tone[5], Tone[4], Tone[2], Tone[1], Tone[8]]

Beat = [1, 1, 2, 1, 1, 2,
        1, 1, 1.5, 0.5, 4,
        1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 2, 2]

# MQTT
mqtt_client = 1
run = False


def RGB(R, G, B):
    if R < 0:
        R = random.randint(0, 65535)
    elif R >= 65535:
        R = 65534
    if G < 0:
        G = random.randint(0, 65535)
    elif G >= 65535:
        G = 65534
    if B < 0:
        B = random.randint(0, 65535)
    elif B >= 65535:
        B = 65534
    # print(R,G,B)
    Led_R.duty_u16(R)
    Led_G.duty_u16(G)
    Led_B.duty_u16(B)

def setup():
    global m
    m = MotorDriver()

def moveForward():
    m.MotorRunInstant('MA', 'forward', speedpct)
    m.MotorRunInstant('MB', 'forward', speedpct)
    m.MotorRunInstant('MC', 'forward', speedpct)
    m.MotorRunInstant('MD', 'forward', speedpct)
    
def moveBackward():
    m.MotorRunInstant('MA', 'backward', speedpct)
    m.MotorRunInstant('MB', 'backward', speedpct)
    m.MotorRunInstant('MC', 'backward', speedpct)
    m.MotorRunInstant('MD', 'backward', speedpct)
    
def moveForwardSpd(speedpct1):
    m.MotorRunInstant('MA', 'forward', speedpct1)
    m.MotorRunInstant('MB', 'forward', speedpct1)
    m.MotorRunInstant('MC', 'forward', speedpct1)
    m.MotorRunInstant('MD', 'forward', speedpct1)
    
def moveBackwardSpd(speedpct1):
    m.MotorRunInstant('MA', 'backward', speedpct1)
    m.MotorRunInstant('MB', 'backward', speedpct1)
    m.MotorRunInstant('MC', 'backward', speedpct1)
    m.MotorRunInstant('MD', 'backward', speedpct1)
    
def stopMove():
    
    m.MotorStop('MA')
    m.MotorStop('MB')
    m.MotorStop('MC')
    m.MotorStop('MD')
        

def rotateLeft():
    
    m.MotorRunInstant('MA', 'backward', speedpct)
    m.MotorRunInstant('MB', 'forward', speedpct)
    m.MotorRunInstant('MC', 'backward', speedpct)
    m.MotorRunInstant('MD', 'forward', speedpct)
    
#     motor(0,speed3,speed4,0)
#     motor2(0,speed3,speed4,0)

def rotateRight():

    
    m.MotorRunInstant('MA', 'forward', speedpct)
    m.MotorRunInstant('MB', 'backward', speedpct)
    m.MotorRunInstant('MC', 'forward', speedpct)
    m.MotorRunInstant('MD', 'backward', speedpct)

def rotateRightSpd(speedpct1):
    m.MotorRunInstant('MA', 'forward', speedpct1)
    m.MotorRunInstant('MB', 'backward', speedpct1)
    m.MotorRunInstant('MC', 'forward', speedpct1)
    m.MotorRunInstant('MD', 'backward', speedpct1)

def parallelLeft():
    
    m.MotorRunInstant('MA', 'forward', speedpct)
    m.MotorRunInstant('MB', 'backward', speedpct)
    m.MotorRunInstant('MC', 'backward', speedpct)
    m.MotorRunInstant('MD', 'forward', speedpct)
    
#     motor(0,speed3,speed4,0)
#     motor2(0,speed3,speed4,0)

def parallelRight():

    
    m.MotorRunInstant('MA', 'backward', speedpct)
    m.MotorRunInstant('MB', 'forward', speedpct)
    m.MotorRunInstant('MC', 'forward', speedpct)
    m.MotorRunInstant('MD', 'backward', speedpct)
# def motor(A1, A2, B1, B2):
#     motor1.duty_u16(A1)
#     motor2.duty_u16(A2)
#     motor3.duty_u16(B1)
#     motor4.duty_u16(B2)

def moveSpeed(leftSpeed, rightSpeed):
    margin=3000
    speedpct1=75
    speedpct2=75
    if leftSpeed > rightSpeed+margin:
        # move to left
        # m.MotorRunInstant('MA', 'backward', speedpct)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        # m.MotorRunInstant('MC', 'backward', speedpct)
        m.MotorRunInstant('MD', 'forward', speedpct1)
    elif leftSpeed < rightSpeed -margin:
        
        # move to right
        m.MotorRunInstant('MA', 'forward', speedpct1)
        # m.MotorRunInstant('MB', 'backward', speedpct)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        # m.MotorRunInstant('MD', 'backward', speedpct)
    else:
        #move straight
        m.MotorRunInstant('MA', 'forward', speedpct2)
        m.MotorRunInstant('MB', 'forward', speedpct2)
        m.MotorRunInstant('MC', 'forward', speedpct2)
        m.MotorRunInstant('MD', 'forward', speedpct2)

def moveSpeed2(tz, tx, last_direction):
    margin=20
    z_limit=220
    speedpct1=15
    speedpct2=15
    current_direction=last_direction
    print(f'**last_direction:{last_direction}**')
    # last_direction 0, move left, 1, move right, 2, straight, 3 stop
    if tz< z_limit:
        ratio=tz/z_limit
        speedpct1=int(float(speedpct1)*ratio)
        speedpct2=int(float(speedpct2)*ratio)

    if tx < -margin and last_direction !=0 :
        # move to left
        m.MotorStop('MA')
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorStop('MC')
        m.MotorRunInstant('MD', 'forward', speedpct1)
        current_direction=0
        print(f'move left ------------{speedpct1}')
    elif tx > margin and last_direction !=1 :
        
        # move to right
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorStop('MB')
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorStop('MD')
        print(f'{speedpct1} +++++++++++++++  move right')
        current_direction=1
        # m.MotorRunInstant('MD', 'backward', speedpct)
    elif tx <= margin and tx >= -margin and last_direction !=2:
        #move straight
        m.MotorRunInstant('MA', 'forward', speedpct2)
        m.MotorRunInstant('MB', 'forward', speedpct2)
        m.MotorRunInstant('MC', 'forward', speedpct2)
        m.MotorRunInstant('MD', 'forward', speedpct2)
        print(f' =========={speedpct2}===============')
        current_direction=2
    elif tz < z_limit and last_direction !=3:
        stopMove()
        current_direction=3
    return current_direction
        
    
        


def moveContiuously(leftSpeed, rightSpeed):
    leftpct=int(leftSpeed/speed*100)
    rightpct=int(rightSpeed/speed*100)
    print(f'drive pct: {leftpct}, {rightpct}')
    m.MotorRunInstant('MA', 'forward', rightpct)
    m.MotorRunInstant('MB', 'forward', leftpct)
    m.MotorRunInstant('MC', 'forward', rightpct)
    m.MotorRunInstant('MD', 'forward', leftpct)

def moveParallel(leftSpeed, rightSpeed):
    margin=2000
    speedpct1=100
    speedpct2=100
    if leftSpeed > rightSpeed+margin:
        # move to parallel left
        m.MotorRunInstant('MA', 'forward', speedpct2)
        m.MotorRunInstant('MB', 'backward', speedpct2)
        m.MotorRunInstant('MC', 'backward', speedpct1)
        m.MotorRunInstant('MD', 'forward', speedpct1)
    elif leftSpeed < rightSpeed -margin:
        
        # move to parallel right
        m.MotorRunInstant('MA', 'backward', speedpct2)
        m.MotorRunInstant('MB', 'forward', speedpct2)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorRunInstant('MD', 'backward', speedpct1)
        
def moveRotate(z_dis, x_dis,speedpct1):
    z_limit=200
    #nano margin
    #margin1=20
    #mv210 margin
    margin1=10
    alignment=True
    if z_dis> z_limit:
        margin=margin1*z_dis /z_limit#2 cm proportion to z_dis
    else:
        margin=margin1
        
    if x_dis < -margin:
        # move to rotate left
        m.MotorRunInstant('MA', 'backward', speedpct1)
        m.MotorRunInstant('MB', 'forward', speedpct1)
        m.MotorRunInstant('MC', 'backward', speedpct1)
        m.MotorRunInstant('MD', 'forward', speedpct1)
        alignment=False
    elif x_dis > margin:
        
        # move to rotate right
        m.MotorRunInstant('MA', 'forward', speedpct1)
        m.MotorRunInstant('MB', 'backward', speedpct1)
        m.MotorRunInstant('MC', 'forward', speedpct1)
        m.MotorRunInstant('MD', 'backward', speedpct1)
        alignment=False
    return alignment

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

def moveUpDown(z_dis, y_dis,speedpct1):
    z_limit=200
    # nano margin
    #margin1=20
    # mv210 margin
    margin1=30
    margin2=10
    alignment=True
    if z_dis> z_limit:
        marginUp=margin1*z_dis /z_limit#2 cm proportion to z_dis
        marginDown=margin2*z_dis /z_limit
    else:
        marginUp=margin1
        marginDown=margin2
    
        
    if y_dis < -marginDown:
        # move up
        armUp(speedpct1)
        alignment=False
        print('move up up up up')
    elif y_dis > marginUp:
        
        # arm down
        #armDown(speedpct1-30)
        armDown(speedpct1-35)
        alignment=False
        print('move down down down')
    return alignment
    
def closeClaw():
#  open claw
    global grabLastState
    global grabState
    global currentGrabLevel
    global maxGrabLevel
    global minGrabLevel
    
    if grabLastState!=2:
        currentGrabLevel=maxGrabLevel
        
    else:
        currentGrabLevel=max(currentGrabLevel-10,minGrabLevel)
        
    print(f"open Claw at {currentGrabLevel}")
    
    RotateBCW(currentGrabLevel)
    sleep(0.5)
    StopMotor()
    sleep(0.2)
    grabLastState=2

def openClaw():
# close claw
    global grabLastState
    global grabState
    global currentGrabLevel
    global maxGrabLevel
    global minGrabLevel
    
    if grabLastState!=1:
        currentGrabLevel=maxGrabLevel
        
    else:
        currentGrabLevel=max(currentGrabLevel-10,minGrabLevel)
        
    print(f"close Claw at {currentGrabLevel}")
    RotateBCCW(currentGrabLevel)
    sleep(0.5)
    StopMotor()
    sleep(0.2)
    grabLastState=1

def getDistance():
    trig = Pin(8, Pin.OUT)
    trig.value(0)
    utime.sleep_us(2)
    trig.value(1)
    utime.sleep_us(20)
    trig.value(0)
    echo = Pin(9, Pin.IN)
    while echo.value() == 0:
        start = utime.ticks_us()
    while echo.value() == 1:
        end = utime.ticks_us()
    d = (end - start) * 0.0343 / 2
    # print("d: ", end="")
    # print(d)
    return d


# def Obstacle_Avoidance():
#     motor(speed, 0, 0, speed)
#     utime.sleep(0.2)
#     motor(0, 0, 0, 0)
#     utime.sleep(0.5)
#     distance_left = int(getDistance())
#     motor(0, speed, speed, 0)
#     utime.sleep(0.4)
#     motor(0, 0, 0, 0)
#     utime.sleep(0.5)
#     distance_right = int(getDistance())
#     if distance_left > 20 and distance_right > 20:
#         if distance_left > distance_right:
#             motor(speed, 0, 0, speed)
#             utime.sleep(0.4)
#         else:
#             motor(speed, 0, speed, 0)
#             utime.sleep(0.2)
#     else:
#         motor(0, speed, speed, 0)
#         utime.sleep(0.5)


def Advance():
    distance = getDistance()
    # if distance < 20:
    #     # Obstacle_Avoidance()
    # else:
    #     # motor(0, speed, 0, speed)


# def Dodge():
#     distance = getDistance()
#     if distance < 20:
#         motor(0, speed, speed, 0)
#         utime.sleep(0.5)
#         motor(0, 0, 0, 0)
#     else:
#         motor(0, speed, 0, speed)


def Music():
    global buzzer
    for i in range(0, len(Song)):
        buzzer.duty_u16(2000)
        buzzer.freq(Song[i])
        for j in range(1, Beat[i] / 0.1):
            time.sleep_ms(25)
        buzzer.duty_u16(0)
        time.sleep(0.01)


def exec_cmd(key_val):
    global state_value
    global mode, firstLoop
    # if (key_val == 0x18):
    #     #         print("Button ^")
    #     moveForward()
    #     # motor(0, speed, 0, speed)  # Go forward
    # elif (key_val == 0x08):
    #     #         print("Button <")
    #     motor(speed, 0, 0, speed)  # Turn left
    # elif (key_val == 0x5a):
    #     #         print("Button >")
    #     motor(0, speed, speed, 0)  # Turn right
    # elif (key_val == 0x52):
    #     #         print("Button V")
    #     motor(speed, 0, speed, 0)  # Go back
    # elif (key_val == 0x45):
    #     #         print("Button 1")
    #     RGB(65534, 65534, 65534)
    # elif (key_val == 0x46):
    #     #         print("Button 2")
    #     Led_R.duty_u16(0)
    #     Led_G.duty_u16(0)
    #     Led_B.duty_u16(0)
    # elif (key_val == 0x47):
    #     #         print("Button 3")     buzzer
    #     RGB(0, 0, 0)
    # elif (key_val == 0x44):
    #     #         print("Button 4")     buzzer
    #     print("music")
    #     buzzer.duty_u16(2000)
    #     buzzer.freq(587)
    # elif (key_val == 0x40):
    #     # print("Button 5")     #buzzer
    #     # Advance()
    #     state_value = 2
    # elif (key_val == 0x43):
    #     #         print("Button 6")     buzzer
    #     # Tracking()
    #     state_value = 3
    # elif (key_val == 0x07):
    #     #         print("Button 7")     buzzer
    #     # Find_light()
    #     state_value = 4
    # elif (key_val == 0x15):
    #     #         print("Button 8")     buzzer
    #     # Dodge()
    #     state_value = 5
    # elif (key_val == 0x09):
    #     #         print("Button 9")     buzzer
    #     Music()
    # elif (key_val == 0x19):
    #     #         print("Button 0")     buzzer
    #     #         buzzer.duty_u16(2000)
    #     #         buzzer.freq(587)
    #     mode = True
    #     firstLoop = 0
    #     print(mode)
    # elif (key_val == 0x0d):
    #     #         print("Button 0")     buzzer
    #     state_value = 0
    # else:
    #     motor(0, 0, 0, 0)  # Stop
    #     buzzer.duty_u16(0)
#         print("STOP")


# WIFI 连接函数
wifi_wait_time = 0


def do_connect():
    import network
    global wifi_wait_time
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        try:
            print('connecting to network...')
            sta_if.active(True)
            sta_if.connect(wifiName, wifiPassword)
            while not sta_if.isconnected():
                utime.sleep(1)
                wifi_wait_time += 1
                if wifi_wait_time >= 10:
                    raise Exception("timeout")
        except Exception as e:
            print("Connection error:", e)
            RGB(65534, 65534, 0)
            while True:
                pass
    print('connect  WiFi ok')
    print(sta_if.ifconfig())


# 接收消息，并处理
def MsgOK(topic, msg):          # 回调函数，用于收到消息
    print((topic, msg))             # 打印主题值和消息值
    global state_value
    global speed
    global mqtt_client
    global mode, firstLoop
    if topic == myTopic.encode():     # 判断是不是发给myTopic的消息
        if msg == b'down':
            moveBackward()
            # motor(speed, 0, speed, 0)
            utime.sleep(0.2)
            state_value = 0
            stopMove()
            # motor(0, 0, 0, 0)
        elif msg == b'up':
            moveForward()
            # motor(0, speed, 0, speed)
            utime.sleep(0.2)
            state_value = 0
            stopMove()
            # motor(0, 0, 0, 0)
        elif msg == b'left':
            # motor(speed, 0, 0, speed)
            rotateLeft()
            utime.sleep(0.2)
            state_value = 0
            stopMove()
            # motor(0, 0, 0, 0)
        elif msg == b'right':
            # motor(0, speed, speed, 0)
            rotateRight()
            utime.sleep(0.2)
            state_value = 0
            stopMove()
            # motor(0, 0, 0, 0)
        elif msg == b'stop':
            state_value = 0
            stopMove()
            # motor(0, 0, 0, 0)
        elif msg == b'turnon':
            RGB(65534, 65534, 65534)
            mqtt_client.publish("Car Light", "see light")
        elif msg == b'turnoff':
            RGB(0, 0, 0)
            mqtt_client.publish("Car Light", "close light")
        elif msg == b'random':
            print("run random")
            RGB(-1, -1, -1)
        elif msg == b'advance':
            print("run advance")
            state_value = 2
            # Advance()
        elif msg == b'tracking':
            print("run tracking")
            state_value = 3
            # Tracking()
        elif msg == b'find light':
            print("run find light")
            state_value = 4
            # Find_light()
        elif msg == b'dodge':
            print("run dodge")
            state_value = 5
            # Dodge()
        elif msg == b'music':
            print("run dodge")
            state_value = 6
            # Dodge()
        elif msg == b'iRemote':
            mode = False
            firstLoop = 0
            # Dodge()
        else:
            print(msg.decode())


def connect_and_subscribe():
    client = MQTTClient(client_id=clientID, server=serverIP,
                        port=port, keepalive=6000)
    client.set_callback(MsgOK)
    client.connect()
    client.subscribe(myTopic)
    print("Connected to %s" % serverIP)
    return client


def restart_and_reconnect():
    print('Failed to connect to MQTT broker. Reconnecting...')
    time.sleep(10)
    machine.reset()


def connect_show_params(request):
    global mqtt_client
    global run
    global serverIP
    global port
    global machineId
    ''' request handler '''
    params = utils.get_request_query_params(request)
    print(params)
    ips = params['mqtt_ip'].split(":")
    serverIP = ips[0]
    port = ips[1]
    ''' will return {"param_one": "one", "param_two": "two"} '''
    server.send("HTTP/1.0 200 OK\r\n")
    server.send("Content-Type: text/html\r\n\r\n")
    if machineId != params['machineid']:
        return server.send("Not this car")
    if run == True:
        return server.send("mqtt is connected!")
    try:
        mqtt_client = connect_and_subscribe()
        server.send("ok")
        run = True
    except OSError as e:
        server.send("failed")


def stop_show_params(request):
    global mqtt_client
    global run
    global machineId
    ''' request handler '''
    params = utils.get_request_query_params(request)
    print(params)
    server.send("HTTP/1.0 200 OK\r\n")
    server.send("Content-Type: text/html\r\n\r\n")
    if machineId != params['machineid']:
        return server.send("Not this car")
    if run != True:
        return server.send("No mqtt connected!")
    try:
        mqtt_client.disconnect()
        server.send("ok")
        run = False
    except OSError as e:
        server.send("failed")


def status_show_params(request):
    global run
    global serverIP
    global port
    global machineId
    ''' request handler '''
    params = utils.get_request_query_params(request)
    print(params)

    if machineId != params['machineid']:
        server.send("HTTP/1.0 200 OK\r\n")
        server.send("Content-Type: text/html\r\n\r\n")
        return server.send("Not this car")
    json_str = json.dumps(
        {"run": run, "mqtt_ip": "{}:{}".format(serverIP, port)})
    server.send("HTTP/1.0 200 OK\r\n")
    server.send("Content-Type: application/json\r\n\r\n")
    server.send(json_str)


server = MicroPyServer()
''' add route '''
server.add_route("/connect", connect_show_params)
server.add_route("/stop", stop_show_params)
server.add_route("/status", status_show_params)


def parse_data(data):
    values_list = []
    lines = data.split('\n')
    
    for line in lines:
        if line.strip():  # Check if the line is not just whitespace
            values = {}
            data_items = line.split(',')
            for item in data_items:
                key, value = item.strip().split(':')
                try:
                    values[key] = float(value) if '.' in value else int(value)
                except ValueError:
                    values[key] = value
            values_list.append(values)
    
    return values_list


# 初始化小车状态

# motor(0, 0, 0, 0)
setup()
stopMove()
buzzer.duty_u16(0)
mode = 'uart'
firstLoop = 0
wifi_connect = False
state_value = 0
# 初始化UART
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart2 = UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))

RGB(65534, 65534, 65534)
xunzhao = 0
xunzhao1 = 0
utime.sleep(1)
count=0
count2=0
count_limit=10
count_start=3
startRunct=6
maxSpeed=65000
last_z=0
last_x=0
last_y=0
start_fine_ct=6
x_max=200
z_max=200
y_max=200
almost_there_ct=0
#for nano
#come_in_step=7
#for mv210
come_in_step=4
got_there=False
grabbed=True
grab_limit=3
grab_ct=0
release_wait_ct=0
release_num=50
z_limit=200
z_close_limit=200
target_id=21 # 第一个目标的id
grab_id=11 # 最终的id
received_id=0
received_id2=0
zoomfactor=40
z_target=220
z_grab_target=220
z_range=10
#log_data(['location',"x","y","z","count2","almost_there_ct","grab_ct"], is_header=True)
log_data(['time',"supplyVoltage","voltageAtADC"], is_header=True)
last_direction=3
grabbed_found=False
got_grabbed_area=False
rotateRight_ct=0
rotateRight_limit=15

while True:
    # firstLoop += 1
    # if firstLoop == 1:
    # RGB(0, 0, 65534)
    # if uart.any():  # 检查是否有可用的数据
    # data = uart.read(uart.any())  # 读取数据并解码
    # print("Received:", data)  # 打印接收到的数据
    RGB(0, 0, 0)
    # utime.sleep(0.1)
    count +=1
    # print(f"Received: {data}")  # 以十六进制形式打印
    try:
        #if uart.any():
        # print(f"Received data.")
        # this is from nano
        #data = uart.read().decode('utf-8').strip()  # 读取一个字节
        # this is from mv210
        if uart2.any():
            data = uart2.read().decode('utf-8').strip()  # 读取一个字节
            #print(f"whole data:{data}")
            # Parse data
            indx=0
            indx2=0
            try:
                print(data)
                parsed_values_list = parse_data(data)
                length1=len(parsed_values_list)
                if length1 >0:
                    for i in range(length1):
                
                    # assume the first value is the targeted id
                        received_id1=int(parsed_values_list[i].get('id', 'N/A'))
                        if received_id1==target_id:
                            indx=i
                            received_id=received_id1
                            print(f"received id:{received_id}")
                            z_dis = int(-zoomfactor*float(parsed_values_list[indx].get('Tz', 'N/A')))
                            x_dis = int(zoomfactor*float(parsed_values_list[indx].get('Tx', 'N/A')))
                            y_dis= int(-zoomfactor*float(parsed_values_list[indx].get('Ty', 'N/A')))
                            print(f"x={x_dis},y={y_dis},z={z_dis}")

                        if got_there and received_id1==grab_id:
                            indx2=i
                            received_id2=received_id1
                            grabbed_found=True

            except Exception as e:
                print("Error parsing data:", e)
                
            #continue

            # #print(f"Received: {data}")
            # # 找到第一个 '}' 的位置
            # first_end_brace = data.index('\n')
    
            # # 截取第一个 JSON 对象的字符串
            # first_json_str = data[:first_end_brace + 1].replace('{', '').replace('}', '')

            # json_obj = first_json_str.split(',')

            # first step to move to the area of the object
            if received_id== target_id or got_there :
                #print(f'{json_obj[0]}, {json_obj[1]}')
                
                #print(f'{json_obj[1]}, {json_obj[1]}')
                if got_there:
                    if grabbed_found:
                        z_dis = int(-zoomfactor*float(parsed_values_list[indx2].get('Tz', 'N/A')))
                        x_dis = int(zoomfactor*float(parsed_values_list[indx2].get('Tx', 'N/A')))
                        y_dis= int(-zoomfactor*float(parsed_values_list[indx2].get('Ty', 'N/A')))
                        x_diff=abs(x_dis-last_x)
                        z_diff=abs(z_dis-last_z)
                        y_diff=abs(y_dis-last_y)
                        print(f'receive2: {z_dis}, {x_dis}. {y_dis}')
                        print(f'diff2: {z_diff},{x_diff}')
                    else:
                        if rotateRight_ct < rotateRight_limit: # only rotate a limited counts
                            stopMove()
                            # rotateRightSpd(60) # rotate in right until it find it
                            utime.sleep(0.1)
                            
                            print('rotate')
                            
                            rotateRight_ct+=1
                            count=0

                else: 
                    z_dis = int(-zoomfactor*float(parsed_values_list[indx].get('Tz', 'N/A')))
                    x_dis = int(zoomfactor*float(parsed_values_list[indx].get('Tx', 'N/A')))
                    y_dis= int(-zoomfactor*float(parsed_values_list[indx].get('Ty', 'N/A')))
                    x_diff=abs(x_dis-last_x)
                    z_diff=abs(z_dis-last_z)
                    y_diff=abs(y_dis-last_y)
                    print(f'receive: {z_dis}, {x_dis}. {y_dis}')
                    print(f'diff: {z_diff},{x_diff}')

                if (z_dis != last_z or x_dis != last_x or y_dis != last_y) and z_diff < z_max and x_diff <x_max and y_diff <y_max:
                    print(f'move:------------------- {z_dis}, {x_dis}, {y_dis}')
                    #log_data(["a1",x_dis,y_dis,z_dis,count2])
                    count2 +=1
                    if count2>3:
                        # if count2 <= startRunct:
                        #     motor(0, maxSpeed, 0, maxSpeed)
                        # else:
                        #motor(0, motor_left, 0, motor_right)
                        #moveSpeed(motor_left, motor_right)

                        # ratio1=0.4
                        # motorleft1=motor_left*ratio1
                        # motorright1=motor_right*ratio1
                        # moveContiuously(motorleft1,motorright1)


                        #moveParallel(motor_left, motor_right)
                        if not got_there:

                            if z_dis > z_limit:
                                alignmenty=moveUpDown(z_dis,y_dis,60)#70
                            else:
                                alignmenty=moveUpDown(z_dis,y_dis,40)#50

                            # if count2< start_fine_ct:
                            #     # alignment=moveRotate(z_dis,x_dis, 70)
                            #     # utime.sleep(0.1)
                            #     # stopMove()
                            #     if z_dis > z_limit:
                            #         alignmenty=moveUpDown(z_dis,y_dis,70)
                            #     else:
                            #         alignmenty=moveUpDown(z_dis,y_dis,50)
                            # else:
                            #     # alignment=moveRotate(z_dis,x_dis, 50)
                            #     # utime.sleep(0.1)
                            #     # stopMove()
                            #     if z_dis > z_limit:
                            #         alignmenty=moveUpDown(z_dis,y_dis,70)
                            #     else:
                            #         alignmenty=moveUpDown(z_dis,y_dis,50)


                            # if alignment and alignmenty:
                            # if alignmenty: 
                                #z_target=120
                                #z_range=10
                                # for mv210
                                # z_target=220
                                # z_range=10
                                # log_data(["a2 aligned",x_dis,y_dis,z_dis,count2])
                                
                            if z_dis > z_target:
                                current_direction=moveSpeed2(z_dis,x_dis,last_direction)
                                last_direction=current_direction
                                # moveForwardSpd(30)
                                utime.sleep(0.1)
                                
                                got_there=False
                                #log_data(["a3 moveForward",x_dis,y_dis,z_dis,count2,almost_there_ct])
                            else: # reach to the area of target
                                got_there=True
                                count2=0 # restart for finding the grabbed target
                            almost_there_ct=0
                                
                        elif grabbed_found: # got there
                            if count2< start_fine_ct:
                                alignment=moveRotate(z_dis,x_dis, 55)
                                utime.sleep(0.1)
                                stopMove()
                                if z_dis > z_close_limit:
                                    alignmenty=moveUpDown(z_dis,y_dis,60)
                                else:
                                    alignmenty=moveUpDown(z_dis,y_dis,40)
                            else:
                                alignment=moveRotate(z_dis,x_dis, 35)
                                utime.sleep(0.1)
                                stopMove()
                                if z_dis > z_close_limit:
                                    alignmenty=moveUpDown(z_dis,y_dis,60)
                                else:
                                    alignmenty=moveUpDown(z_dis,y_dis,40)

                            
                            if z_dis > z_grab_target:
                                moveForwardSpd(20)
                                utime.sleep(0.1)
                                stopMove()
                                almost_there_ct=0
                                print('go to grab area')
                                #log_data(["a3 moveForward",x_dis,y_dis,z_dis,count2,almost_there_ct])
                            else: # reach to the area of grabbed target

                                almost_there_ct +=1
                                moveForwardSpd(20)
                                utime.sleep(0.1)
                                stopMove()
                                print(f'come in ct:{almost_there_ct}')
                                #log_data(["a4 come in",x_dis,y_dis,z_dis,count2,almost_there_ct])
                                if almost_there_ct > come_in_step: # if it can be seen and keep move in
                                    got_grabbed_area=True
                                    
                                # else:
                                #     log_data(["go there is true",x_dis,y_dis,z_dis,count2,almost_there_ct])
                                # stopMove()


                    count=0
                elif almost_there_ct >0: # but can not see the grabbed tag any more

                    almost_there_ct +=1
                    if almost_there_ct < come_in_step: # if it can not been seen
                        moveForwardSpd(20)
                        utime.sleep(0.1)
                        print('come in 2')
                        stopMove()
                        #log_data(["come in 2",x_dis,y_dis,z_dis,count2,almost_there_ct])
                    else:
                        got_grabbed_area=True
                        #log_data(["go there ----",x_dis,y_dis,z_dis,count2,almost_there_ct])
                        print('got there =============================================')
                            
                last_x=x_dis
                last_z=z_dis
                last_y=y_dis
                
                # xunzhao = 0
                # if xunzhao == 0:
                #     xunzhao += 1
                #     motor(0, speed - 1000, speed - 1000, 0)
                #     utime.sleep(0.2)
                #     motor(0, 0, 0, 0)
                #     utime.sleep(1)
                #     motor(speed - 1000, 0, 0, speed - 1000)
                #     utime.sleep(0.2)
                #     motor(0, 0, 0, 0)
                #     utime.sleep(1)
                # motor(0, speed, 0, speed)
                # utime.sleep(1)
                # motor(0, 0, 0, 0)
                # utime.sleep(1)
            # else:
            #    motor(0, speed, speed, 0)
            #    utime.sleep(0.1)
            #    motor(0, 0, 0, 0)
            #    utime.sleep(1)
        elif almost_there_ct >0: # but can not see the grabbed tag any more

            almost_there_ct +=1
            if almost_there_ct < come_in_step: # if it can not been seen
                moveForwardSpd(20)
                utime.sleep(0.1)
                print('come in 2')
                stopMove()
                #log_data(["come in 2",x_dis,y_dis,z_dis,count2,almost_there_ct])
            else:
                got_grabbed_area=True
                almost_there_ct=0
                #log_data(["go there ----",x_dis,y_dis,z_dis,count2,almost_there_ct])
                print('got there ========')
                
    except Exception as e:
        pass
        # xunzhao = 0
        # xunzhao1 += 1
        # if xunzhao1 >= 2000:
        #    print("发生错误：", e)
        #    xunzhao1 = 0
        #    motor(0, speed - 1000, speed - 1000, 0)
        #    utime.sleep(0.1)
        #    motor(0, 0, 0, 0)
        #    utime.sleep(1)
    if got_grabbed_area and not grabbed:
        grab_ct+=1
        if grab_ct > grab_limit:
            grabbed=True
            grab_ct=0
            got_there=False
            got_grabbed_area=False
            #log_data(["grabbed reset ----",None,None,None,None,None,grab_ct])
        else:
#             closeClaw(70)
#             closeClaw(70)
#             closeClaw(70)
            closeClaw()
            closeClaw()
            closeClaw()
            closeClaw()
            closeClaw()
            #log_data(["close Claw ----",None,None,None,None,None,grab_ct])
        
    if grabbed:
        release_wait_ct+=1
        if release_wait_ct > release_num:

            openClaw()
            openClaw()
            openClaw()
            openClaw()
            openClaw()
            
            # closeClaw()
            # closeClaw()
            # closeClaw()
            # closeClaw()
            # closeClaw()
#             openClaw()
#             openClaw(90)
#             openClaw(70)
            # openClaw(70)
            # armUp(70)
            # armDown(70)
            grabbed=False
            got_there=False
            grabbed_found=False
            got_grabbed_area=False
            grab_ct=0
            release_wait_ct=0
            almost_there_ct=0
            #log_data(["Open Claw ----",None,None,None,None,None,grab_ct])
            
    if count>count_limit and not got_there:
        # armDown(10)
        # armUp(80)
        # motor(0, 0, 0, 0)
        # moveForwardSpd(40)
        # moveUpDown(10,100, 10)
        # closeClaw()
        # moveRotate(120, 60, 30)
        stopMove()
        
        count2=0
        print('car stopped')
    utime.sleep(0.2)
    # read voltage
    adc_value = adc.read_u16() >> 4  # Scale from 16-bit to 12-bit
    # Convert ADC value to voltage
    voltage_at_adc = (adc_value / ADC_MAX) * V_REF
    # Calculate the original voltage
    original_voltage = voltage_at_adc /resistance_ratio
    # Calculate elapsed time
    elapsed_time = utime.ticks_diff(utime.ticks_ms(), start_time)
    #timestamp2=float(timestamp)+float(elapsed_time)/1000
    timestamp2=elapsed_time
    log_data([timestamp2,original_voltage,voltage_at_adc])
    RGB(0, 0, 65534)




    




