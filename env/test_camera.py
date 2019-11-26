import cv2
import time
import numpy as np
from matplotlib import pyplot as plt

cam = cv2.VideoCapture(2)
w=cam.get(cv2.CAP_PROP_FRAME_WIDTH)
h=cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(w,h)
print(cam.isOpened())
# w=cam.get(cv2.CAP_PROP_FRAME_WIDTH)
# h=cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
# fourcc=cv2.VideoWriter_fourcc(*'mp4v')
# out=cv2.VideoWriter('output.mp4',fourcc,15.0,(int(w),int(h)))
# while(1):
#     (ret, im) = cam.read()
#     if ret:
#         out.write(im)
#         cv2.imshow("1", im)
#     k=cv2.waitKey(25)
#     if k==27:
#         break
# cam.release()
# out.release()
# while(1):
_,im=cam.read()
cv2.imshow("0",im)
cv2.waitKey(1)
hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
h=hsv.copy()
h[:,:,1]=0
h[:,:,2]=0
cv2.imshow("1",h)
cv2.waitKey(1)
sensetivity=10
gl = (45, 0, 0)
gu = (55, 255, 200)
bl=(55,0,0)
bu=(70,255,255)
green=cv2.inRange(hsv,bl,bu)
cv2.imshow("2",green)
cv2.waitKey(0)



