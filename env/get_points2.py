from pprint import pprint
from pulseapi import *
from time import sleep
from threading import Thread, Semaphore
import queue
import cv2

host = "10.10.10.20:8081"
robot = RobotPulse(host)

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
            cv2.imshow("1",frame)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h = hsv.copy()
            h[:, :, 1] = 0
            h[:, :, 2] = 0
            cv2.imshow("2", h)
            # bl = (85, 0, 0)
            # bu = (105, 255, 255)
            # gl = (55, 0, 0)
            # gu = (60, 255, 255)
            bl = (85, 50, 50)
            bu = (105, 255, 255)
            gl = (60, 50, 50)
            gu = (70, 255, 255)
            goal = cv2.inRange(hsv, gl, gu) + cv2.inRange(hsv, bl, bu)

            cv2.imshow("3",goal)
            cv2.waitKey(25)
            self.q.put(frame)

    def read(self):
        return self.q.get()
VideoCapture(2)
sleep(25)
# try:
#     # while True:
#     # robot.set_position(position([-0.377, 0.225, 0.365], [3.1241393089294434, 0.0349065847694874, -1.5533430576324463]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-195.28646850585938, -90.06591796875, -92.04893493652344, -90.3863525390625, 90.8438720703125, -14.649581909179688]), SPEED)
#     robot.await_motion()
#     # robot.set_position(position([-0.311, 0.2, 0.205], [3.1241393089294434, 0.0698131695389748, -1.5882495641708374]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-193.93240356445312, -87.91191101074219, -131.54478454589844, -54.45489501953125, 90.073486328125, -14.645462036132812]), SPEED)
#     robot.await_motion()
#     robot.open_gripper()
#     robot.close_gripper()
#     # robot.set_position(position([-0.331, 0.227, 0.378], [3.1066861152648926, 0.122173048555851, -1.5184364318847656]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-197.29696655273438, -84.29603576660156, -91.83195495605469, -100.7945556640625, 90.1064453125, -14.648208618164062]), SPEED)
#     robot.await_motion()
#     # robot.set_position(position([-0.375, 0.441, 0.357], [3.054326295852661, 0.0174532923847437, -1.6406095027923584]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-218.333740234375, -121.41746520996094, -52.27980041503906, -100.11407470703125, 92.78091430664062, -42.25547790527344]), SPEED)
#     robot.await_motion()
#     # robot.set_position(position([-0.375, 0.441, 0.357], [3.054326295852661, 0.0174532923847437, -1.6406095027923584]), SPEED, MT_JOINT)
#     robot.set_pose(pose([-218.333740234375, -121.43669128417969, -52.28392028808594, -100.1168212890625, 92.78436279296875, -42.252044677734375]), SPEED)
#     robot.await_motion()
#     robot.open_gripper()
#     robot.set_pose(pose([-218.333740234375, -121.43669128417969, -52.28392028808594, -100.1168212890625, 92.78436279296875, -42.252044677734375]), SPEED)
#     robot.await_motion()
# except RestApiException as e:
#     pprint("Exception when calling RestRobot %s\n" % e)
