import vrep.vrep as vrep
import vrep.vrepConst as const_v
import time
import sys
import numpy as np
import cv2
import math
import os


class rozum_sim:

    def __init__(self):
        self.DoF = 6
        self.action_bound = [-5, 5]
        self.action_dim = self.DoF

        # os.chdir("/Vrep_server")
        # os.system("git pull")
        # os.chdir("/")
        #
        # self.vrep_root = "/V-REP_PRO_EDU_V3_5_0_Linux/"
        # self.scene_file = "/Vrep_server/env/rozum_model.ttt"
        #
        # os.chdir(self.vrep_root)
        # os.system("xvfb-run --auto-servernum --server-num=1 -s \"-screen 0 640x480x24\" ./vrep.sh -h -s " + self.scene_file + " &")

        vrep.simxFinish(-1)
        time.sleep(1)

        # get the ID of the running simulation
        self.ID = vrep.simxStart('127.0.0.1', 19999, True, False, 5000, 5)
        # check the connection
        if self.ID != -1:
            print("Connected")
        else:
            sys.exit("Error")
        # get handles
        # for camera
        self.cam_handle = self.get_handle('Vision_sensor')
        (code, res, im) = vrep.simxGetVisionSensorImage(self.ID, self.cam_handle, 0, const_v.simx_opmode_streaming)

        # joints
        self.joint_handles = []
        for i in range(self.DoF):
            tmp = self.get_handle("joint%d" % (i))
            self.joint_handles.append(tmp)
            code, angle = vrep.simxGetJointPosition(self.ID, tmp, const_v.simx_opmode_streaming)

        # gripper tip
        self.tip_handle = self.get_handle("Tip")
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.tip_handle, -1, const_v.simx_opmode_streaming)
        # cube
        self.cube_handle = self.get_handle("Cube")
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.cube_handle, -1, const_v.simx_opmode_streaming)
        # get the goal handle
        self.goal_handle = self.get_handle("Goal")
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.goal_handle, -1, const_v.simx_opmode_streaming)

        # angles' array
        self.angles = self.get_angles()

        # gripper handles (used in closing and opening gripper)
        self.gripper_motor = self.get_handle('RG2_openCloseJoint')
        # task part
        self.task_part = 0

        self.init_angles = self.get_angles()

        self.init_pose_cube = self.get_position(self.cube_handle)
        # print(self.init_pose_cube)
        self.init_goal_pose = self.get_position(self.goal_handle)
        # print(self.init_goal_pose)
        self.open_gripper()
        self.reset()
        self.tip_position = self.get_position(self.tip_handle)

        self.goal_l = (80, 0, 0)
        self.goal_u = (120, 255, 255)
        self.cube_l = (55, 50, 50)
        self.cube_u = (80, 255, 255)
        self.er_kernel = np.ones((2, 2), np.uint8)
        self.di_kernel = np.ones((2, 22), np.uint8)
        self.task_part = 0
        self.part_1_center = np.array([300.0, 335.0])
        self.part_2_center = np.array([320.0, 290.0])
        self.part_1_area = 0.25
        self.part_2_area = 0.75

    def get_handle(self, name):
        (check, handle) = vrep.simxGetObjectHandle(self.ID, name, const_v.simx_opmode_blocking)
        if check != 0:
            print("Couldn't find %s" % name)
        return handle

    def get_position(self, handle):
        (code, pose) = vrep.simxGetObjectPosition(self.ID, handle, -1, const_v.simx_opmode_buffer)
        # print(code)
        return np.array(pose)

    def close_gripper(self):
        code=vrep.simxSetJointForce(self.ID, self.gripper_motor, 20, const_v.simx_opmode_blocking)
        # print(code)
        code=vrep.simxSetJointTargetVelocity(self.ID, self.gripper_motor, -0.05, const_v.simx_opmode_blocking)
        # print(code)
        # time.sleep(0.1)

    def open_gripper(self):
        code=vrep.simxSetJointForce(self.ID, self.gripper_motor, 20, const_v.simx_opmode_blocking)
        # print(code)
        code=vrep.simxSetJointTargetVelocity(self.ID, self.gripper_motor, 0.05, const_v.simx_opmode_blocking)
        # print(code)
        #time.sleep(0.1)

    def get_image(self, cam_handle):
        (code, res, im) = vrep.simxGetVisionSensorImage(self.ID, cam_handle, 0, const_v.simx_opmode_buffer)
        # print(code)
        img = np.array(im, dtype=np.uint8)
        img.resize([res[0], res[1], 3])
        img=cv2.flip(img,0)
        return img

    def move_joint(self, num, value):
        # in radian
        code=vrep.simxSetJointTargetPosition(self.ID, self.joint_handles[num], value*math.pi/180, const_v.simx_opmode_blocking)
        # print(code)
        time.sleep(0.3)

    def get_angles(self):
        angles = []
        for i in range(self.DoF):
            code, angle = vrep.simxGetJointPosition(self.ID, self.joint_handles[i], const_v.simx_opmode_buffer)
            angles.append(angle*180/math.pi)
        return angles

    def sample_action(self):
        return np.random.uniform(*self.action_bound, size=self.action_dim)

    # def get_reward(self):
    #     reward = -0.1
    #     done = False
    #     tmp_tip_position = self.get_position(self.tip_handle)
    #     if tmp_tip_position[2] - self.tip_position[2] > 0.2:
    #         reward -= 1
    #         return reward, True
    #     self.tip_position = tmp_tip_position
    #     if self.task_part == 0:
    #         goal_pose = self.get_position(self.cube_handle)
    #         distance = np.linalg.norm(self.tip_position - goal_pose)
    #         if distance <= 0.02:
    #             self.close_gripper()
    #             self.task_part += 1
    #             reward += 1.0
    #             self.move_joint(1, 0.0)
    #             return reward, done
    #         reward += (1 - distance)
    #     else:
    #         goal_pose = self.get_position(self.goal_handle)
    #         goal = goal_pose
    #         goal[2] += 0.1
    #         distance = np.linalg.norm(self.tip_position - goal)
    #         if abs(goal_pose[2] - self.tip_position[2]) < 0.02:
    #             self.open_gripper()
    #             reward += 1
    #             cube_pose = self.get_position(self.cube_handle)
    #             if abs(cube_pose[2] - goal_pose[2]) < 0.02:
    #                 reward += 2
    #             return reward, True
    #         reward += (1 - distance)
    #     return reward, done

    def step(self, action):
        action = np.clip(action, *self.action_bound)
        self.angles = self.get_angles()
        for i in range(self.DoF):
            self.angles[i] += action[i]
            self.move_joint(i, self.angles[i])
        img = self.get_image(self.cam_handle)
        reward, done = self.get_reward(img)
        return img, reward, done, {}

    def reset(self):
        self.task_part = 0
        self.angles = self.init_angles
        for i in range(self.DoF):
            self.move_joint(i, self.angles[i])
        vrep.simxSetObjectPosition(self.ID, self.cube_handle, -1, self.init_pose_cube, const_v.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(self.ID, self.goal_handle, -1, self.init_goal_pose, const_v.simx_opmode_oneshot_wait)
        img = self.get_image(self.cam_handle)
        return img

    def image_processeing(self,img,lower,upper,num_iter):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        h=hsv.copy()
        h[:,:,1]=0
        h[:,:,2]=0
        binary = cv2.inRange(hsv, lower, upper)
        binary = cv2.erode(binary, self.er_kernel, iterations=num_iter[0])
        binary = cv2.dilate(binary, self.di_kernel, iterations=num_iter[1])
        cv2.imshow("1",binary)
        cv2.waitKey(1)
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
            area_percentage=area/(256*256)
            rotation = abs(angle)
        # print(center)
        return center,area_percentage,rotation

    def get_reward(self, img):
        reward = -0.1
        done = False
        if self.task_part == 0:
            center, area, rotation = self.image_processeing(img, self.goal_l, self.goal_u, [1, 1])
            distance = np.linalg.norm(center - self.part_1_center)
            area_difference = abs(area - self.part_1_area)
            # print(distance, area_difference, rotation)
            if distance < 3 and area_difference < 2 and rotation < 1:
                self.task_part = 1
                reward += 2
                self.close_gripper()
                return reward, done
        else:
            center, area, rotation = self.image_processeing(img, self.cube_l, self.cube_u, [1, 1])
            distance = np.linalg.norm(center - self.part_2_center)
            area_difference = abs(area - self.part_2_area)
            # print(distance,area_difference,rotation)
            if distance < 5 and area_difference < 5 and rotation < 1:
                reward += 2
                done = True
                self.open_gripper()
                return reward, done
        reward -= (0.01 * distance + 0.05 * area_difference + 0.1 * rotation)
        return reward, done

    def render(self):
        im=self.get_image(self.cam_handle)
        cv2.imshow("render",im)
        cv2.waitKey(10)

env=rozum_sim()
while True:
    a=env.sample_action()
    _,r,_,_=env.step(a)
    print(r)