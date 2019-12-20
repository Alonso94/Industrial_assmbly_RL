from pprint import pprint
from pulseapi import *
from time import sleep
from threading import Thread, Semaphore
import queue
import cv2
import numpy as np
from scipy.spatial import distance as dist

host = "10.10.10.20:8081"
robot = RobotPulse(host)
task_part=0
SPEED = 10


def order_points(pts):
    # sort the points based on their x-coordinates
    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-roodinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    # now, sort the left-most coordinates according to their
    # y-coordinates so we can grab the top-left and bottom-left
    # points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    # now that we have the top-left coordinate, use it as an
    # anchor to calculate the Euclidean distance between the
    # top-left and right-most points; by the Pythagorean
    # theorem, the point with the largest distance will be
    # our bottom-right point
    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    (br, tr) = rightMost[np.argsort(D)[::-1], :]

    # return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="float32")

class VideoCapture:

    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        # fourcc=cv2.VideoWriter_fourcc(*'mp4v')
        # self.out=cv2.VideoWriter('output.mp4',fourcc,15.0,(int(w),int(h)))
        self.q = queue.Queue()
        self.goal_l = (80, 0, 0)
        self.goal_u = (110, 255, 255)
        self.cube_l = (55, 50, 50)
        self.cube_u = (80, 255, 255)
        self.er_kernel = np.ones((7, 7), np.uint8)
        self.di_kernel = np.ones((10, 10), np.uint8)

        self.goal_l = (80, 0, 0)
        self.goal_u = (120, 255, 255)
        self.cube_l = (55, 50, 50)
        self.cube_u = (80, 255, 255)
        self.er_kernel = np.ones((5, 5), np.uint8)
        self.di_kernel = np.ones((12, 12), np.uint8)
        self.task_part = 0
        self.part_1_center = np.array([310.0, 340.0])
        self.part_2_center = np.array([320.0, 290.0])
        self.part_1_area = 0.25
        self.part_2_area = 0.70

        self.t = Thread(target=self._reader)
        self.t.daemon = True
        self.t.start()

    def image_processeing(self,img,lower,upper,num_iter):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        binary = cv2.inRange(hsv, lower, upper)
        binary = cv2.erode(binary, self.er_kernel, iterations=num_iter[0])
        binary = cv2.dilate(binary, self.di_kernel, iterations=num_iter[1])
        cnt, _ = cv2.findContours(binary, 1, 1)
        cnt = sorted(cnt, key=cv2.contourArea, reverse=True)
        center=0
        area_percentage=0
        rotation=0
        if len(cnt) > 0:
            rect = cv2.minAreaRect(cnt[0])
            angle = rect[2]
            if angle < -45:
                angle += 90
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            center = np.average(box, axis=0)
            area = cv2.contourArea(cnt[0])
            area_percentage=area/(self.w*self.h)
            rotation = abs(angle)
        # print(center)
        return center,area_percentage,rotation

    def get_reward(self, img):
        reward = -0.1
        done = False
        if self.task_part == 0:
            center, area, rotation = self.image_processeing(img, self.goal_l, self.goal_u, [2, 2])
            distance = np.linalg.norm(center - self.part_1_center)
            area_difference = abs(area - self.part_1_area)
            # print(distance, area_difference, rotation)
            if distance < 3 and area_difference < 2 and rotation < 1:
                self.task_part = 1
                reward += 2
                return reward, done
        else:
            center, area, rotation = self.image_processeing(img, self.cube_l, self.cube_u, [2, 2])
            distance = np.linalg.norm(center - self.part_2_center)
            area_difference = abs(area - self.part_2_area)
            # print(distance,area_difference,rotation)
            if distance < 5 and area_difference < 5 and rotation < 1:
                reward += 2
                done = True
                return reward, done
        reward -= (0.01 * distance + 0.05 * area_difference + 0.1 * rotation)
        return reward, done

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
            # if task_part==0:
            #     frame=self.image_processeing(frame,self.goal_l,self.goal_u,[2,2])
            # else:
            #     frame = self.image_processeing(frame,self.cube_l,self.cube_u,[2,2])
            # self.out.write(frame)
            # print(self.get_reward(frame))
            cv2.imshow("1",frame)
            cv2.waitKey(25)

    def read(self):
        return self.q.get()

    def close(self):
        # self.out.release()
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
    print(robot.get_position())
    # x=input()
    # task_part=1
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
    # x=input()
    print(robot.get_position())
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
