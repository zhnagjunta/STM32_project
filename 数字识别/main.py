import time, image, sensor, math, pyb, ustruct
from image import SEARCH_EX, SEARCH_DS
from pyb import Pin, Timer, LED

sensor.reset()
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.QQVGA)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_windowing((0, 50, 160, 70))
template01 = image.Image("/1.pgm")
template02 = image.Image("/2.pgm")
template03 = image.Image("/3.pgm")
template04 = image.Image("/4.pgm")
template05 = image.Image("/5.pgm")
template06 = image.Image("/6.pgm")
template07 = image.Image("/7.pgm")
template08 = image.Image("/8.pgm")
template3L = image.Image("/3L.pgm")
template3LL = image.Image("/3LL.pgm")
template3R = image.Image("/3R.pgm")
template3RR = image.Image("/3RR.pgm")
template4L = image.Image("/4L.pgm")
template4LL = image.Image("/4LL.pgm")
template4R = image.Image("/4R.pgm")
template4RR = image.Image("/4RR.pgm")
template5L = image.Image("/5LX.pgm")
template5LL = image.Image("/5LLX.pgm")
template5R = image.Image("/5R.pgm")
template5RR = image.Image("/5RR.pgm")
template6L = image.Image("/6L.pgm")
template6LX = image.Image("/6LX.pgm")
template6R = image.Image("/6R.pgm")
template6RR = image.Image("/6RR.pgm")
template6RRX = image.Image("/6RRX.pgm")
template7L = image.Image("/7L.pgm")
template7LL = image.Image("/7LL.pgm")
template7R = image.Image("/7R.pgm")
template7RR = image.Image("/7RR.pgm")
template7RX = image.Image("/7RX.pgm")
template7RRX = image.Image("/7RRX.pgm")
template8L = image.Image("/8L.pgm")
template8LL = image.Image("/8LL.pgm")
template8LLX = image.Image("/8LLX.pgm")
template8R = image.Image("/8R.pgm")
template8RR = image.Image("/8RR.pgm")
template8RX = image.Image("/8RX.pgm")
uart = pyb.UART(3, 115200, timeout_char=1000)
blue_led = LED(3)
green_led = LED(2)
red_led = LED(1)
Find_Task = 1
Target_Num = 0
find_flag = 0
x = 0
data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
LoR = 0


def UartReceiveDate():
    global Find_Task
    global Target_Num
    global x
    global data
    data[0] = uart.readchar()
    data[1] = uart.readchar()
    data[2] = uart.readchar()
    data[3] = uart.readchar()
    data[4] = uart.readchar()
    data[5] = uart.readchar()
    data[6] = uart.readchar()
    data[7] = uart.readchar()
    if data[x] == 42 and data[x + 3] == 38 and x < 5:
        Find_Task = data[x + 1]
        Target_Num = data[x + 2]
        Find_Task = Find_Task - 48
        Target_Num = Target_Num - 48
        print(Find_Task, Target_Num)
        x = 0
    elif x >= 5:
        x = 0
    x += 1
    print(Find_Task, Target_Num)


def FirstFindTemplate(template):
    R = img.find_template(template, 0.78, step=1, roi=(40, 25, 85, 39), search=SEARCH_EX)
    return R


def FirstFindedNum(R, Finded_Num):
    global Find_Task
    global find_flag
    img.draw_rectangle(R, color=(225, 0, 0))
    find_flag = 1
    Num = Finded_Num
    FH = bytearray([0x2C, 0x12, Num, LoR, find_flag, Find_Task, 0x5B])
    uart.write(FH)
    blue_led.on()
    print("目标病房号：", Num)


def FindTemplateL(template):
    R = img.find_template(template, 0.65, step=1, roi=(0, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplateR(template):
    R = img.find_template(template, 0.60, step=1, roi=(90, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplate8L(template):
    R = img.find_template(template, 0.74, step=1, roi=(0, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplate8R(template):
    R = img.find_template(template, 0.68, step=1, roi=(90, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplate7L(template):
    R = img.find_template(template, 0.76, step=1, roi=(0, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplate7R(template):
    R = img.find_template(template, 0.76, step=1, roi=(90, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplate6L(template):
    R = img.find_template(template, 0.74, step=1, roi=(0, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplate6R(template):
    R = img.find_template(template, 0.71, step=1, roi=(90, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplate5L(template):
    R = img.find_template(template, 0.74, step=1, roi=(0, 10, 70, 50), search=SEARCH_EX)
    return R


def FindTemplate5R(template):
    R = img.find_template(template, 0.74, step=1, roi=(90, 10, 70, 50), search=SEARCH_EX)
    return R


def FindedNum(R, Finded_Num):
    global Find_Task
    global find_flag
    img.draw_rectangle(R, color=(225, 0, 0))
    if R[0] > 90:
        LoR = 2
    elif 0 < R[0] < 70:  # 原本是60
        LoR = 1
    else:
        LoR = 0
    find_flag = 1
    Num = Finded_Num
    if LoR > 0:
        FH = bytearray([0x2C, 0x12, Num, LoR, find_flag, Find_Task, 0x5B])
        uart.write(FH)
        print("识别到的数字是：", Num, "此数字所在方位：", LoR)


clock = time.clock()
red_led.on()
pyb.delay(500)
red_led.off()
while (True):
    clock.tick()
    img = sensor.snapshot()
    UartReceiveDate()
    if Find_Task == 0:
        red_led.off()
        blue_led.off()
        green_led.off()
        LoR = 0
    if Find_Task == 1:
        r01 = FirstFindTemplate(template01)
        r02 = FirstFindTemplate(template02)
        r03 = FirstFindTemplate(template03)
        r04 = FirstFindTemplate(template04)
        r05 = FirstFindTemplate(template05)
        r06 = FirstFindTemplate(template06)
        r07 = FirstFindTemplate(template07)
        r08 = FirstFindTemplate(template08)
        if r01:
            FirstFindedNum(r01, 1)
        elif r02:
            FirstFindedNum(r02, 2)
        elif r03:
            FirstFindedNum(r03, 3)
        elif r04:
            FirstFindedNum(r04, 4)
        elif r05:
            FirstFindedNum(r05, 5)
        elif r06:
            FirstFindedNum(r06, 6)
        elif r07:
            FirstFindedNum(r07, 7)
        elif r08:
            FirstFindedNum(r08, 8)
    elif Find_Task == 2:
        red_led.on()
        green_led.on()
        if Target_Num == 3:
            if LoR == 0:
                r3L = FindTemplateL(template3L)
                r3R = FindTemplateR(template3R)
                r3LL = FindTemplateL(template3LL)
                r3RR = FindTemplateR(template3RR)
            if r3L and LoR == 0:
                FindedNum(r3L, 3)
            elif r3R and LoR == 0:
                FindedNum(r3R, 3)
            elif r3LL and LoR == 0:
                FindedNum(r3LL, 3)
            elif r3RR and LoR == 0:
                FindedNum(r3RR, 3)
        elif Target_Num == 4:
            if LoR == 0:
                r4L = FindTemplateL(template4L)
                r4R = FindTemplateR(template4R)
                r4LL = FindTemplateL(template4LL)
                r4RR = FindTemplateR(template4RR)
            if r4L and LoR == 0:
                FindedNum(r4L, 4)
            elif r4R and LoR == 0:
                FindedNum(r4R, 4)
            elif r4LL and LoR == 0:
                FindedNum(r4LL, 4)
            elif r4RR and LoR == 0:
                FindedNum(r4RR, 4)
        elif Target_Num == 5:
            if LoR == 0:
                r5L = FindTemplate5L(template5L)
                r5LL = FindTemplate5L(template5LL)
                r5R = FindTemplate5R(template5R)
                r5RR = FindTemplate5R(template5RR)
            if r5L and LoR == 0:
                FindedNum(r5L, 5)
            elif r5LL and LoR == 0:
                FindedNum(r5LL, 5)
            elif r5R and LoR == 0:
                FindedNum(r5R, 5)
            elif r5RR and LoR == 0:
                FindedNum(r5RR, 5)
        elif Target_Num == 6:
            if LoR == 0:
                r6L = FindTemplate6L(template6L)
                r6LX = FindTemplate6L(template6LX)
                r6R = FindTemplate6R(template6R)
                r6RR = FindTemplate6R(template6RR)
                r6RRX = FindTemplate6R(template6RRX)
            if r6L and LoR == 0:
                FindedNum(r6L, 6)
            elif r6LX and LoR == 0:
                FindedNum(r6LX, 6)
            elif r6RRX and LoR == 0:
                FindedNum(r6RRX, 6)
            elif r6R and LoR == 0:
                FindedNum(r6R, 6)
            elif r6RR and LoR == 0:
                FindedNum(r6RR, 6)
        elif Target_Num == 7:
            if LoR == 0:
                r7L = FindTemplate7L(template7L)
                r7LL = FindTemplate7L(template7LL)
                r7R = FindTemplate7R(template7R)
                r7RR = FindTemplate7R(template7RR)
                r7RX = FindTemplate7R(template7RX)
                r7RRX = FindTemplate7R(template7RRX)
            if r7L and LoR == 0:
                FindedNum(r7L, 7)
            elif r7LL and LoR == 0:
                FindedNum(r7LL, 7)
            elif r7RX and LoR == 0:
                FindedNum(r7RX, 7)
            elif r7R and LoR == 0:
                FindedNum(r7R, 7)
            elif r7RRX and LoR == 0:
                FindedNum(r7RRX, 7)
            elif r7RR and LoR == 0:
                FindedNum(r7RR, 7)
        elif Target_Num == 8:
            if LoR == 0:
                r8L = FindTemplate8L(template8L)
                r8LL = FindTemplate8L(template8LL)
                r8LLX = FindTemplate8R(template8LLX)
                r8R = FindTemplate8R(template8R)
                r8RR = FindTemplate8R(template8RR)
                r8RX = FindTemplate8R(template8RX)
            if r8L and LoR == 0:
                FindedNum(r8L, 8)
            elif r8LL and LoR == 0:
                FindedNum(r8LL, 8)
            elif r8LLX and LoR == 0:
                FindedNum(r8LLX, 8)
            elif r8R and LoR == 0:
                FindedNum(r8R, 8)
            elif r8RR and LoR == 0:
                FindedNum(r8RR, 8)
            elif r8RX and LoR == 0:
                FindedNum(r8RX, 8)
