import requests
import json
import cv2
import numpy as np
from threading import Thread, Semaphore
import queue
import time
import math
from gym import spaces

class Rozum:
    def __init__(self):
        self.host = "http://10.10.10.20:8081"
        self.joint_angles = self.get_joint_angles()
        self.position, self.orientation = self.get_position()

    def get_joint_angles(self):
        # in degrees
        response = requests.get(self.host + '/pose')
        return response.json()['angles']

    def send_joint_angles(self):
        # speed 10
        requests.put(self.host + '/pose?speed=10', data=json.dumps({
            "angles": self.joint_angles
        }))
        url = self.host + '/status/motion'
        response = requests.get(url)
        while (response.content != b'"IDLE"'):
            response = requests.get(url)

    def get_joints_current(self):
        response = requests.get(self.host + '/status/motors')
        currents = []
        motor_info = response.json()
        for motor in motor_info:
            currents.append(motor["rmsCurrent"])
        return currents

    def get_position(self):
        response = requests.get(self.host + '/position')
        pose_info = response.json()
        point = pose_info["point"]
        position = [point["x"], point["y"], point["z"]]
        rot = pose_info["rotation"]
        orientation = [rot["roll"], rot["pitch"], rot["yaw"]]
        return (position, orientation)

    def send_position(self):
        # speed 10
        res = requests.put(self.host + '/position?speed=10', data=json.dumps({
            "point": {
                "x": self.position[0],
                "y": self.position[1],
                "z": self.position[2]
            },
            "rotation": {
                "roll": self.orientation[0],
                "pitch": self.orientation[1],
                "yaw": self.orientation[2]
            }
        }))
        url = self.host + '/status/motion'
        response = requests.get(url)
        while (response.content != b'"IDLE"'):
            response = requests.get(url)

    def open_gripper(self):
        requests.put(self.host + '/gripper/open')

    def close_gripper(self):
        requests.put(self.host + '/gripper/close')

    def recover(self):
        requests.put(self.host + '/recover')

    def update_joint_angles(self, values):
        for i in range(len(self.joint_angles)):
            self.joint_angles[i] = values[i]

    def update_position(self, position, orientation):
        for i in range(3):
            self.position[i] = position[i]
            self.orientation[i] = orientation[i]


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
            self.q.put(frame)

    def read(self):
        return self.q.get()


class rozum_real:
    def __init__(self):
        self.robot = Rozum()
        self.action_bound = [-2, 2]
        self.action_dim = 6
        self.cam = VideoCapture(2)
        self.w = self.cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.h = self.cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.action_space = spaces.Box(low=-5, high=5, shape=[self.action_dim])
        self.observation_space = spaces.Box(low=0, high=100, shape=[self.action_dim + 4 + 4], dtype=np.float64)


        self.goal_l= (80, 40, 0)
        self.goal_u= (110, 255, 255)
        self.cube_l = (55, 50, 0)
        self.cube_u = (80, 255, 255)
        self.er_kernel = np.ones((7, 7), np.uint8)
        self.di_kernel = np.ones((10, 10), np.uint8)
        self.task_part=0
        self.part_1_center=np.array([300.0/640,335.0/480])
        self.part_2_center=np.array([320.0/640,290.0/480])
        self.part_1_area=0.25
        self.part_2_area=0.75
        self.target=np.array([300.0/640,335.0/480,0.25,0.0])
        # self.target=np.array([-0.375, 0.441, 0.357])
        # self.count=0

        self.currents_thread=Thread(target=self.current_reader)
        self.currents_thread.daemon=True
        self.currents_thread.start()

        self.robot.open_gripper()
        self.init_pose, _ = self.robot.get_position()
        # self.init_angles = [-200,-90,-90,-90,90,0]
        self.init_angles = [-210.0,-110.0,0.0,-160.0,90.0,-35.0]
        self.s=self.reset()
        self.angles = self.init_angles.copy()
        self.samples=self.prepare_samples_pilco()
        self.count=0
        self.t=0

        # print(self.angles)

    def prepare_samples_pilco(self):
        # print("sampling")
        samples=[]
        for i in range(self.action_dim):
            for j in range(10):
                sample=np.zeros([self.action_dim])
                ra=[-5*(i+1),5*(i+1)]
                sample[i]=np.random.uniform(*ra,1)
                # print(sample)
                samples.append(sample)
                samples.append(-sample)
                if len(samples)>20:
                    samples[-1]=samples[-1]+samples[-19]
                    samples[-2] = samples[-2] + samples[-20]
        return samples

    def current_reader(self):
        while True:
            self.currents = self.robot.get_joints_current()

    def sample_pilco(self):
        self.count+=1
        return self.samples[self.count%len(self.samples)]

    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    def step(self, action):
        self.t+=1
        action = np.clip(action, *self.action_bound)
        for i in range(self.action_dim):
            self.angles[i] += action[i]
        self.robot.update_joint_angles(self.angles)
        self.robot.send_joint_angles()
        img=self.cam.read()
        obs, reward, done,new_target = self.get_reward(img)
        angles = self.robot.get_joint_angles()
        self.t += 1
        self.s=np.concatenate((obs,angles,self.target),axis=None)
        self.pose=np.array(self.robot.get_position()[0])
        return self.s, reward, done, {}

    def reset(self):
        self.task_part = 0
        self.t=0
        self.target=np.array([300.0/640,335.0/480,0.25,0.0])
        self.angles=self.init_angles.copy()
        self.robot.update_joint_angles(self.angles)
        self.robot.send_joint_angles()
        img=self.cam.read()
        currents=self.currents.copy()
        center, area, rotation,binary= self.image_processing(img, self.goal_l, self.goal_u, [2, 2])
        obs = np.array([center[0]/640, center[1]/480, area, rotation])
        self.s = np.concatenate((obs, self.angles, self.target), axis=None)
        self.pose = np.array(self.robot.get_position()[0])
        return self.s

    def image_processing(self, img, lower, upper, num_iter):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        binary = cv2.inRange(hsv, lower, upper)
        binary = cv2.erode(binary, self.er_kernel, iterations=num_iter[0])
        binary = cv2.dilate(binary, self.di_kernel, iterations=num_iter[1])
        cv2.imshow("1",binary)
        cv2.waitKey(1)
        cnt, _ = cv2.findContours(binary, 1, 1)
        cnt = sorted(cnt, key=cv2.contourArea, reverse=True)
        center = np.array([0.0, 0.0])
        area_percentage = 0
        rotation = 0
        if len(cnt) > 0:
            rect = cv2.minAreaRect(cnt[0])
            angle = rect[2]
            if angle < -45:
                angle += 90
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            center = np.average(box, axis=0)
            area = cv2.contourArea(cnt[0])
            area_percentage = area / (640 * 480)
            rotation = abs(angle)/90
        binary=binary[...,np.newaxis]
        return center, area_percentage, rotation,binary

    # def get_reward(self, img):
    #     reward = -0.01
    #     done = False
    #     new_target=False
    #     if self.task_part == 0:
    #         center, area, rotation = self.image_processeing(img, self.goal_l, self.goal_u, [2, 2])
    #         obs = np.array([center[0]/640, center[1]/480, area, rotation])
    #         if area>0.3:
    #             reward-=2
    #             done=True
    #             return obs,reward,done,new_target
    #         distance = np.linalg.norm(center - self.part_1_center, axis=-1)
    #         area_difference = abs(area - self.part_1_area)
    #         # print(distance, area_difference, rotation)
    #         if distance < 0.01 and area>self.part_1_area and rotation < 0.2:
    #             self.reset()
    #             self.task_part = 1
    #             self.target=np.array([320.0/640,290.0/480,0.75,0.0])
    #             # self.target=np.array([-0.311, 0.2, 0.205])
    #             new_target=True
    #             reward += 2
    #             self.det_goal = self.robot.get_joint_angles()
    #             return obs, reward, done,{}
    #     else:
    #         self.new_target = False
    #         center, area, rotation = self.image_processeing(img, self.cube_l, self.cube_u, [2, 2])
    #         obs = np.array([center[0]/640, center[1]/480, area, rotation])
    #         distance = np.linalg.norm(center - self.part_2_center, axis=-1)
    #         area_difference = abs(area - self.part_2_area)
    #         # print(distance,area_difference,rotation)
    #         if distance < 0.01 and area>=self.part_2_area and rotation < 0.2:
    #             reward += 5
    #             done = True
    #             self.robot.close_gripper()
    #             self.angles = self.init_angles.copy()
    #             self.robot.update_joint_angles(self.angles)
    #             self.robot.send_joint_angles()
    #             self.angles = self.det_goal.copy()
    #             self.robot.update_joint_angles(self.angles)
    #             self.robot.send_joint_angles()
    #             self.robot.open_gripper()
    #             new_target=True
    #             return obs, reward, done,{}
    #     if obs[2] < 0.01:
    #         reward -= 5
    #         # self.count += 1
    #         # if self.count > 20:
    #         #     self.count = 0
    #         done = True
    #         new_target=True
    #         return obs, reward, done,{}
    #     reward+=2.5*(1/(1+math.pow(distance,1.2)))+1.5*(1/(1+math.pow(area_difference,1.2)))+(1/(1+math.pow(rotation,1.2)))
    #     return obs, reward, done,new_target

#     def get_reward(self, img):
#         reward = -0.1
#         done = False
#         if self.task_part == 0:
#             center, area, rotation,binary = self.image_processing(img, self.goal_l, self.goal_u, [1, 1])
#             obs=np.array([center[0],center[1], area, rotation])
#             distance = np.linalg.norm(center - self.part_1_center, axis=-1)
#             area_difference = abs(area - self.part_1_area)
#             # print(distance, area_difference, rotation)
#             if distance < 0.01 and area > self.part_1_area and rotation < 1:
#                 self.task_part=1
#                 self.target=np.array([320.0/640,290.0/480,0.75,0.0])
#                 self.det_goal=self.robot.get_joint_angles()
#                 self.angles = self.init_angles.copy()
#                 self.robot.update_joint_angles(self.angles)
#                 self.robot.send_joint_angles()
#                 reward += 5
#                 return obs,reward, done,binary
#         else:
#             center, area, rotation,binary = self.image_processing(img, self.cube_l, self.cube_u, [1, 1])
#             obs=np.array([center[0],center[1], area, rotation])
#             distance = np.linalg.norm(center - self.part_2_center, axis=-1)
#             area_difference = abs(area - self.part_2_area)
#             # print(distance,area_difference,rotation)
#             if distance < 0.01 and area > self.part_2_area and rotation < 1:
#                 reward += 5
#                 done = True
#                 self.robot.close_gripper()
#                 self.angles = self.init_angles.copy()
#                 self.robot.update_joint_angles(self.angles)
#                 self.robot.send_joint_angles()
#                 self.angles = self.det_goal.copy()
#                 self.robot.update_joint_angles(self.angles)
#                 self.robot.send_joint_angles()
#                 self.robot.open_gripper()
#                 return obs,reward, done,binary
#         if obs[2]<0.01:
# #             reward-=5
#             done=True
#             return obs,reward, done,binary
#         # reward -= (0.0025 * distance + 0.015 * area_difference + 0.015 * rotation)
# #         reward+= np.exp(-(0.0025 * distance + 1.5 * area_difference + 0.015 * rotation))
#         reward+=0.5*(1/(1+math.pow(distance,1.2)))+0.3*(1/(1+math.pow(area_difference,1.2)))+0.2*(1/(1+math.pow(rotation,1.2)))
#         return obs,reward, done,binary

    def get_reward(self, img):
        done=False
        time_discount=1-self.t/300
        if self.task_part == 0:
            center, area, rotation,binary = self.image_processing(img, self.goal_l, self.goal_u, [2, 2])
            obs=np.array([center[0],center[1], area, rotation])
            distance = np.linalg.norm(center - self.part_1_center, axis=-1)
            area_difference = abs(area - self.part_1_area)
        else:
            center, area, rotation,binary = self.image_processing(img, self.cube_l, self.cube_u, [2, 2])
            obs=np.array([center[0],center[1], area, rotation])
            distance = np.linalg.norm(center - self.part_2_center, axis=-1)
            area_difference = abs(area - self.part_2_area)
        if self.t>=200:
            done=True
            reward=-10
            return obs,reward,done,binary
        if area>0.45 or area<0.05:
            done=True
            reward=-100
            return obs,reward,done,binary
        if distance<0.01:
            if area_difference<0.01:
                if rotation<0.1:
                    reaching_reward=100
                    reward=reaching_reward#*time_discount
                    if self.task_part==0:
                        self.task_part=1
                        self.target = np.array([320.0 / 640, 290.0 / 480, 0.75, 0.0])
                        self.det_goal = self.robot.get_joint_angles()
                        self.angles = self.init_angles.copy()
                        self.robot.update_joint_angles(self.angles)
                        self.robot.send_joint_angles()
                    else:
                        done = True
                        self.robot.close_gripper()
                        self.angles = self.init_angles.copy()
                        self.robot.update_joint_angles(self.angles)
                        self.robot.send_joint_angles()
                        self.angles = self.det_goal.copy()
                        self.robot.update_joint_angles(self.angles)
                        self.robot.send_joint_angles()
                        self.robot.open_gripper()
                else:
                    reward=-10*rotation
            else:
                reward=-10*area_difference
        else:
            distance_reward=1-math.pow(distance,0.4)
            area_reward=math.pow(1-area_difference,1/distance)
            rot_reward=math.pow(1-rotation,1/distance)
            reward=distance_reward*area_reward*time_discount
        if self.task_part==1:
            reward+=100
        return obs,reward,done,binary