import cv2
import numpy as np
from matplotlib import pyplot as plt

cam = cv2.VideoCapture(2)
print(cam.isOpened())
grs=[]
while(1):
    (_, im) = cam.read()
    gr=cv2.cvtColor(im,cv2.COLOR_RGB2GRAY)
    cv2.imshow("1", gr)
    k=cv2.waitKey(25)
    if k==27:
        break
    if k==ord('a'):
        print("add_image")
        grs.append(gr)




