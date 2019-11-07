import requests
import json
from collections import namedtuple
import time
from getpass import getpass

class Rozum:
    def __init__(self):
        self.host = "http://10.10.10.20:8081"
        self.joint_angles=self.get_joint_angles()
        self.position,self.orientation=self.get_position()

    def get_joint_angles(self):
        # in degrees
        response=requests.get(self.host+'/pose')
        return response.json()['angles']

    def send_joint_angles(self):
        # speed 10
        requests.put(self.host+'/pose?speed=10',data=json.dumps({
            "angles": self.joint_angles
        }))
        url = self.host + '/status/motion'
        response = requests.get(url)
        while (response.content != b'"IDLE"'):
            print(self.get_joints_current())
            response = requests.get(url)

    def get_joints_current(self):
        response=requests.get(self.host+'/status/motors')
        currents=[]
        motor_info=response.json()
        for motor in motor_info:
            currents.append(motor["rmsCurrent"])
        return currents

    def get_position(self):
        response=requests.get(self.host+'/position')
        pose_info=response.json()
        point=pose_info["point"]
        position=[point["x"],point["y"],point["z"]]
        rot=pose_info["rotation"]
        orientation=[rot["roll"],rot["pitch"],rot["yaw"]]
        return (position,orientation)

    def send_position(self):
        # speed 10
        res=requests.put(self.host + '/position?speed=10', data=json.dumps({
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
        print(res.content)
        url = self.host + '/status/motion'
        response = requests.get(url)
        while (response.content != b'"IDLE"'):
            print(self.get_joints_current())
            response = requests.get(url)

    def open_gripper(self):
        requests.put(self.host+'/gripper/open')

    def close_gripper(self):
        requests.put(self.host+'/gripper/close')

    def recover(self):
        requests.put(self.host+'/recover')

    def update_joint_angles(self,values):
        for i in range(len(self.joint_angles)):
            self.joint_angles[i]=values[i]

    def update_position(self,position,orientation):
        for i in range(3):
            self.position[i]=position[i]
            self.orientation[i]=orientation[i]

robot=Rozum()
robot.joint_angles[2]+=20
robot.send_joint_angles()