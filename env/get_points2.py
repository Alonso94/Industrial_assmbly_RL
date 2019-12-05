from pprint import pprint
from pulseapi import *
from time import sleep
from threading import Thread, Semaphore
import queue
import cv2
import numpy as np

host = "10.10.10.20:8081"
# robot = RobotPulse(host)

SPEED = 10
# bufferless VideoCapture
class VideoCapture:

    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.q = queue.Queue()
        self.t = Thread(target=self._reader)
        self.t.daemon = True
        self.t.start()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()  # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            cv2.imshow("0",frame)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h = hsv.copy()
            h[:, :, 0] = 0
            h[:, :, 1] = 0
            cv2.imshow("2", h)
            # bl = (85, 0, 0)
            # bu = (105, 255, 255)
            # gl = (55, 0, 0)
            # gu = (60, 255, 255)
            bl = (80, 60, 0)
            bu = (110, 255, 255)
            gl = (55, 50, 0)
            gu = (80, 255, 255)
            # ol1=(0,50,50)
            # ou1=(15,255,255)
            # ol2 = (165, 50, 50)
            # ou2 = (180, 255, 255)
            er_kernel = np.ones((7, 7), np.uint8)
            di_kernel = np.ones((10, 10), np.uint8)
            goal1 = cv2.inRange(hsv, gl, gu)
            goal1 = cv2.erode(goal1, er_kernel,iterations=2)
            goal1=cv2.dilate(goal1,di_kernel,iterations=2)
            goal2=cv2.inRange(hsv, bl, bu)
            goal2 = cv2.erode(goal2, er_kernel,iterations=2)
            goal2 = cv2.dilate(goal2, di_kernel,iterations=2)
            # goal3=cv2.inRange(hsv, ol1, ou1)+cv2.inRange(hsv, ol2, ou2)
            # goal3 = cv2.erode(goal3, er_kernel,iterations=2)
            # goal3 = cv2.dilate(goal3, di_kernel,iterations=2)
            cnt, _ = cv2.findContours(goal1, 1, 1)
            cnt = sorted(cnt, key=cv2.contourArea, reverse=True)
            if len(cnt) > 0:
                rect = cv2.minAreaRect(cnt[0])
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
            cnt,_=cv2.findContours(goal2,1,1)
            cnt = sorted(cnt, key=cv2.contourArea, reverse=True)
            if len(cnt)>0:
                rect = cv2.minAreaRect(cnt[0])
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                print(box)
                center_x=(box[0,0]+box[1,0]+box[2,0]+box[3,0])/4
                center_y = (box[1, 0] + box[1, 0]) / 2
                print(center_x)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
            cv2.imshow("1",frame)
            cv2.imshow("goal1",goal1)
            cv2.imshow("goal2",goal2)
            # cv2.imshow("goal3", goal3)
            cv2.waitKey(25)
            self.q.put(frame)

    def read(self):
        return self.q.get()
VideoCapture(2)
# robot.open_gripper()
while(1):
    sleep(1)
# try:
#     # while True:
#     # robot.set_position(position([-0.377, 0.225, 0.365], [3.1241393089294434, 0.0349065847694874, -1.5533430576324463]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-195.28646850585938, -90.06591796875, -92.04893493652344, -90.3863525390625, 90.8438720703125, -14.649581909179688]), SPEED)
#     robot.await_motion()
#     sleep(2)
#     # robot.set_position(position([-0.311, 0.2, 0.205], [3.1241393089294434, 0.0698131695389748, -1.5882495641708374]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-193.93240356445312, -87.91191101074219, -131.54478454589844, -54.45489501953125, 90.073486328125, -14.645462036132812]), SPEED)
#     robot.await_motion()
#     robot.open_gripper()
#     robot.close_gripper()
#     sleep(2)
#     # robot.set_position(position([-0.331, 0.227, 0.378], [3.1066861152648926, 0.122173048555851, -1.5184364318847656]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-197.29696655273438, -84.29603576660156, -91.83195495605469, -100.7945556640625, 90.1064453125, -14.648208618164062]), SPEED)
#     robot.await_motion()
#     sleep(2)
#     # robot.set_position(position([-0.375, 0.441, 0.357], [3.054326295852661, 0.0174532923847437, -1.6406095027923584]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-218.333740234375, -121.41746520996094, -52.27980041503906, -100.11407470703125, 92.78091430664062, -42.25547790527344]), SPEED)
#     robot.await_motion()
#     sleep(2)
#     # robot.set_position(position([-0.375, 0.441, 0.357], [3.054326295852661, 0.0174532923847437, -1.6406095027923584]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-218.333740234375, -121.43669128417969, -52.28392028808594, -100.1168212890625, 92.78436279296875, -42.252044677734375]), SPEED)
#     robot.await_motion()
#     sleep(2)
#     robot.open_gripper()
#     robot.set_pose(pose([-218.333740234375, -121.43669128417969, -52.28392028808594, -100.1168212890625, 92.78436279296875, -42.252044677734375]), SPEED)
#     robot.await_motion()
# except RestApiException as e:
#     pprint("Exception when calling RestRobot %s\n" % e)
