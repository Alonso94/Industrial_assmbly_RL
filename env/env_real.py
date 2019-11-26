import requests
import json
import cv2
import numpy as np
from threading import Thread, Semaphore
import queue

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
        self.action_bound = [-1, 1]
        self.action_dim = 6
        self.cam = VideoCapture(2)

        self.goal_l= (80, 0, 0)
        self.goal_u= (120, 255, 255)
        self.cube_l = (55, 50, 50)
        self.cube_u = (80, 255, 255)
        self.er_kernel = np.ones((5, 5), np.uint8)
        self.di_kernel = np.ones((12, 12), np.uint8)
        self.task_part=0



        self.currents_thread=Thread(target=self.current_reader)
        self.currents_thread.daemon=True
        self.currents_thread.start()

        self.robot.open_gripper()
        self.init_pose, _ = self.robot.get_position()
        # self.init_angles = [-200,-90,-90,-90,90,0]
        self.init_angles = [-210,-110,0,-160,90,-35]
        self.reset()
        self.angles = self.init_angles
        print(self.angles)

    def current_reader(self):
        while True:
            self.currents = self.robot.get_joints_current()

    def step(self, action):
        action = np.clip(action, *self.action_bound)
        for i in range(self.action_dim):
            self.angles[i] += action[i]
        self.robot.update_joint_angles(self.angles)
        self.robot.send_joint_angles()
        currents=self.currents
        img=self.cam.read()
        reward,done=self.get_reward(img)
        return img,reward,done,{}

    def reset(self):
        self.robot.update_joint_angles(self.init_angles)
        self.robot.send_joint_angles()
        img=self.cam.read()
        currents=self.currents
        return img

    def image_processeing(self,img,lower,upper,num_iter):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        binary = cv2.inRange(hsv, lower, upper)
        binary = cv2.erode(binary, self.er_kernel, iterations=num_iter[1])
        binary = cv2.dilate(binary, self.di_kernel, iterations=num_iter[2])
        return binary

    def get_reward(self,img):
        reward=-0.1
        done=False
        if self.task_part==0:
            processed_img=self.image_processeing(img,self.goal_l,self.goal_u,[1,3])
        else:
            processed_img = self.image_processeing(img, self.cube_l, self.cube_u, [2, 2])

        #TODO define reward
        return reward,done

robot=Rozum()
robot.close_gripper()