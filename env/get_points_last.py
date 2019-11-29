from pprint import pprint
from pulseapi import *
from time import sleep
from threading import Thread, Semaphore
import queue
import cv2
import numpy as np

host = "10.10.10.20:8081"
robot = RobotPulse(host)

SPEED = 10
class VideoCapture:

    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fourcc=cv2.VideoWriter_fourcc(*'mp4v')
        self.out=cv2.VideoWriter('output.mp4',fourcc,15.0,(int(w),int(h)))
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
            self.q.put(frame)
            self.out.write(frame)

    def read(self):
        return self.q.get()

    def close(self):
        self.out.release()
        self.cap.release()

try:
    vid=VideoCapture(2)
    # while True:
    # robot.set_position(position([-0.248, 0.28, 0.608], [3.1415927410125732, 0, -1.6580628156661987]), SPEED, MT_JOINT)
    robot.set_pose(pose([-210.006103515625, -110.00267028808594, -0.002044677734375, -160.0067138671875, 90.05218505859375, -34.99969482421875]), SPEED)
    robot.await_motion()
    # robot.set_position(position([-0.293, 0.419, 0.441], [3.071779489517212, 0.0523598790168762, -1.6929693222045898]), SPEED, MT_JOINT)
    # robot.set_pose(pose([-221.7779541015625, -109.59136962890625, -50.21711730957031, -114.81240844726562, 90.83221435546875, -48.448333740234375]), SPEED)
    # robot.await_motion()
    robot.set_pose(pose(
        [-222.15493774414062, -101.87278747558594, -70.3543701171875, -98.31298828125, 89.967041015625,
         -42.51708984375]), SPEED)
    robot.await_motion()
    # robot.set_position(position([-0.164, 0.265, 0.601], [-3.1241393089294434, -0.122173048555851, -1.6057028770446777]), SPEED, MT_JOINT)
    # robot.set_pose(pose([-217.16574096679688, -79.8431396484375, -48.34326171875, -135.4962158203125, 93.328857421875, -38.645782470703125]), SPEED)
    # robot.await_motion()
    robot.set_pose(pose(
        [-210.006103515625, -110.00267028808594, -0.002044677734375, -160.0067138671875, 90.05218505859375,
         -34.99969482421875]), SPEED)
    robot.await_motion()
    # robot.set_position(position([-0.348, 0.202, 0.204], [-3.1415927410125732, 0.0349065847694874, -1.5882495641708374]), SPEED, MT_JOINT)
    robot.set_pose(pose([-192.85369873046875, -94.72618103027344, -126.28096008300781, -50.85687255859375, 89.52142333984375, -13.353195190429688]), SPEED)
    robot.await_motion()
    robot.open_gripper()
    robot.close_gripper()
    robot.set_pose(pose(
        [-210.006103515625, -110.00267028808594, -0.002044677734375, -160.0067138671875, 90.05218505859375,
         -34.99969482421875]), SPEED)
    robot.await_motion()
    # robot.set_position(position([-0.278, 0.413, 0.409], [3.1415927410125732, 0, -1.5707963705062866]), SPEED, MT_JOINT)
    robot.set_pose(pose([-222.15493774414062, -101.87278747558594, -70.3543701171875, -98.31298828125, 89.967041015625, -42.51708984375]), SPEED)
    robot.await_motion()
    robot.close_gripper()
    robot.open_gripper()
    vid.close()
except RestApiException as e:
    pprint("Exception when calling RestRobot %s\n" % e)
