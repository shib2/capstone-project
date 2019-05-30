import smbus
import time
import RPi.GPIO as GPIO
from time import sleep
from picamera import PiCamera
import cv2
import numpy as np
import argparse
from pyimagesearch.shapedetector import ShapeDetector
import imutils

def filter(n):
 # yellow, blue, green, orange
 limit_low = [[0, 150, 150], [100, 0, 0], [0, 100, 0], [0, 80, 180]]
 limit_high = [[20, 255, 255], [255, 100, 50], [100, 255, 90], [65, 130, 235]]
 low = limit_low[n]
 high = limit_high[n]
 image = cv2.imread('/home/pi/Desktop/Chinmai/Downloads2/opencv-python-color-detection/pic.jpg')
 low = np.array(low, dtype = "uint8")
 high = np.array(high, dtype = "uint8")
 color = cv2.inRange(image, low, high)
 output = cv2.bitwise_and(image, image, mask = color)
 cv2.imshow("images", np.hstack([image, output]))
 cv2.imwrite('/home/pi/Desktop/Chinmai/Downloads2/opencv-python-color-detection/filtered_image.jpg', output)
 

def detect():
 image = cv2.imread('/home/pi/Desktop/Chinmai/Downloads2/opencv-python-color-detection/filtered_image.jpg')
 resized = imutils.resize(image, width=300)
 ratio = image.shape[0] / float(resized.shape[0])
 gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
 blurred = cv2.GaussianBlur(gray, (15, 15), 0)
 thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
 cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 cnts = cnts[0] if imutils.is_cv2() else cnts[1]
 sd = ShapeDetector()
 for c in cnts:
# compute the center of the contour, then detect the name of the
# shape using only the contour
   M = cv2.moments(c)
   if M["m00"] != 0:
    cX = int((M["m10"] / M["m00"]) * ratio)
    cY = int((M["m01"] / M["m00"]) * ratio)
   else:
    cX = 10
    cY = 10
   shape = sd.detect(c)

# multiply the contour (x, y)-coordinates by the resize ratio,
# then draw the contours and the name of the shape on the image
   c = c.astype("float")
   c *= ratio
   c = c.astype("int")
   cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
   cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,	0.5, (255, 255, 255), 2)

# show the output image
   cv2.imshow("Image", image)
###################
 """c = 0
 M = cv2.moments(c)
 if M["m00"] != 0:
  cX = int((M["m10"] / M["m00"]) * ratio)
  cY = int((M["m01"] / M["m00"]) * ratio)
 else:
  cX = 10
  cY = 10
 shape = sd.detect(c)

# multiply the contour (x, y)-coordinates by the resize ratio,
# then draw the contours and the name of the shape on the image
 c = c.astype("float")
 c *= ratio
 c = c.astype("int")
 cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
 cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

# show the output image
 cv2.imshow("Image", image)
 #cv2.imwrite('/home/pi/Desktop/ImageProcessingFinal/FinalCaptures/Orange.jpg', image)
 #cv2.waitKey(0)	"""
	
##################


bus = smbus.SMBus(1)
bus.write_byte(0x29,0x80|0x12)
ver = bus.read_byte(0x29)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
pwm = GPIO.PWM(18, 60)
pwm.start(87)
GPIO.setup(5, GPIO.OUT)
GPIO.output(5, GPIO.LOW)

if ver == 0x44:
 print "Device found\n"
 bus.write_byte(0x29, 0x80|0x00)
 bus.write_byte(0x29, 0x01|0x02)
 bus.write_byte(0x29, 0x80|0x14)
 GPIO.output(5, GPIO.HIGH)
 camera = PiCamera()
 camera.resolution = (512,512)
 #camera.start_preview()
 sleep(3)
 try:
  while True:
   data = bus.read_i2c_block_data(0x29, 0)
   clear = clear = data[1] << 8 | data[0]
   red = data[3] << 8 | data[2]
   green = data[5] << 8 | data[4]
   blue = data[7] << 8 | data[6]
   #crgb = "\nC: %s, R: %s, G: %s, B: %s" % (clear, red, green, blue)
   #print crgb
   if clear > 1150:
    sleep(0.4)
    GPIO.output(5, GPIO.LOW)
    sleep(1)
    camera.capture('/home/pi/Desktop/Chinmai/Downloads2/opencv-python-color-detection/pic.jpg')
    crgb = "C: %s, R: %s, G: %s, B: %s\n" % (clear, red, green, blue)
    print crgb
    sleep(1)
    rg = float(red) / float(green)
    gb = float(green) / float(blue)
    rb = float(red) / float(blue)
    if rg > 2.0:
     print "Orange Object!"
     n = 3
    elif (gb > 2.0 and rb > 1.5):
     print "Yellow Object!"
     n = 0
    elif (gb > 1.0 and rb < 1.5):
     print "Green Object!"
     n = 2
    elif (rg < 1.0 and gb < 1.0 and rb < 1.0):
     print "Blue Object!"
     n = 1
    print n
    pic = cv2.imread('/home/pi/Desktop/Chinmai/Downloads2/opencv-python-color-detection/pic.jpg')
    cv2.imshow("Image captured",pic)
    filter(n)
    detect()
    cv2.waitKey(0)
    #sleep(3)
    cv2.destroyAllWindows()
    sleep(2)
    GPIO.output(5, GPIO.HIGH)
   # time.sleep(1) # assume the time of gap between two detection is 1 seconc, this will be modified for our specific requirements
 except KeyboardInterrupt:
   GPIO.cleanup()
   #camera.stop_preview()


else:
 print "Device not found\n"

#camera.stop_preview()
GPIO.cleanup()
