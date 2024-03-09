import pyb ,sensor, image, math, time
from pyb import UART
from pyb import LED
import json

GRAYSCALE_THRESHOLD = [(0, 50)]
red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
roi_1 = [(20, 0, 120, 20),         #  北
            (20, 100, 120, 20),     # 南
            (0, 20, 20, 80),        #  西
            (140, 20, 20, 80),      #  东
            (60,40,40,40)]          #   中
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.reset()                      # 初始化摄像头
sensor.set_framesize(sensor.QQVGA)
clock = time.clock()
low_threshold = (5,95,5,122,5,122)
buf =[0 for i in range(5)]
DI =[0 for i in range(8)]
uart = UART(3,115200)
uart.init(115200, bits=8, parity=None, stop=1)
isten=0
red_led.on()
pyb.delay(500)
red_led.off()
while(True):
        m = -1
        isten = 0
        sensor.set_pixformat(sensor.RGB565) # 格式为 RGB565.
        img = sensor.snapshot().lens_corr(strength = 1.8, zoom = 1.0)
        img.binary([low_threshold],invert = 1)
        img.erode(1, threshold = 4)
        for r in roi_1:
            m += 1
            blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4],pixels_threshold=100, area_threshold=100, merge=True)
            img.draw_rectangle(r[0:4], color=(255,0,0))
            if blobs:
                most_pixels = 0
                largest_blob = 0
                for i in range(len(blobs)):
                #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                    if blobs[i].pixels() > most_pixels:
                        most_pixels = blobs[i].pixels()
                        #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于
                        largest_blob = i
                # Draw a rect around the blob.
                img.draw_rectangle(blobs[largest_blob].rect())
                #将此区域的像素数最大的颜色块画矩形和十字形标记出来
                img.draw_cross(blobs[largest_blob].cx(),
                               blobs[largest_blob].cy())
                buf[m] = 1
            else:
                buf[m] = 0
        for i in range(0, 160, 20):
            k = int(i/20)
            roi2 = (i, 55, 20, 20)
            img.draw_rectangle((i, 55, 20, 20), color=(0,255,0))
            blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=roi2, pixels_threshold=100, area_threshold=100,merge=True)
            if blobs:
                for n in range(len(blobs)):
                #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                   if blobs[n].pixels() > most_pixels:
                       most_pixels = blobs[n].pixels()
                       #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于
                       largest_blob = n
                   DI[k] = 1
            else:
                DI[k] = 0
        if buf[0]==1 and buf[1] ==1 and buf[2] ==1 and buf[3] ==1 and buf[4] ==1: #十字
            isten = 1
        if buf[1] ==1 and buf[2] ==1 and buf[3] ==1 and buf[4] ==1: #T
            isten = 1
        if buf[0]==1 and buf[1] ==1 and buf[2] ==1 and buf[4] ==1: #左T
            isten = 1
        if buf[0]==1 and buf[1] ==1 and buf[3] ==1 and buf[4] ==1: #右T
            isten = 1
        if buf[0]==0 and buf[1] ==0 and buf[2] ==0 and buf[3] ==0 and buf[4] ==0: #尽头
            isten = 1
        img_data = bytearray([0x55,0x56,DI[0],DI[1],DI[2],DI[3],DI[4],DI[5],DI[6],DI[7],isten])
        uart.write(img_data)
        print(img_data)

