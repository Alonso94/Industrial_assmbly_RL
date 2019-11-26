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
        self.tip_position = self.get_position(self.tip_handle)
        # print(self.tip_position)
        # print(self.cam_handle)
        # im=self.get_image(self.cam_handle)
        # sensorImage = np.array(im, dtype=np.uint8)
        # print(sensorImage)

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

    def get_reward(self):
        reward = -0.1
        done = False
        tmp_tip_position = self.get_position(self.tip_handle)
        if tmp_tip_position[2] - self.tip_position[2] > 0.2:
            reward -= 1
            return reward, True
        self.tip_position = tmp_tip_position
        if self.task_part == 0:
            goal_pose = self.get_position(self.cube_handle)
            distance = np.linalg.norm(self.tip_position - goal_pose)
            if distance <= 0.02:
                self.close_gripper()
                self.task_part += 1
                reward += 1.0
                self.move_joint(1, 0.0)
                return reward, done
            reward += (1 - distance)
        else:
            goal_pose = self.get_position(self.goal_handle)
            goal = goal_pose
            goal[2] += 0.1
            distance = np.linalg.norm(self.tip_position - goal)
            if abs(goal_pose[2] - self.tip_position[2]) < 0.02:
                self.open_gripper()
                reward += 1
                cube_pose = self.get_position(self.cube_handle)
                if abs(cube_pose[2] - goal_pose[2]) < 0.02:
                    reward += 2
                return reward, True
            reward += (1 - distance)
        return reward, done

    def step(self, action):
        action = np.clip(action, *self.action_bound)
        self.angles = self.get_angles()
        for i in range(self.DoF):
            self.angles[i] += action[i]
            self.move_joint(i, self.angles[i])
        img = self.get_image(self.cam_handle)
        reward, done = self.get_reward()
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

    def render(self):
        im=self.get_image(self.cam_handle)
        cv2.imshow("render",im)
        cv2.waitKey(10)