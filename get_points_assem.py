from pprint import pprint
from pulseapi import *
from time import sleep

host = "10.10.10.20:8081"
robot = RobotPulse(host)
versions=Versions(host)

SPEED = 10  # set the desired speed
TCP_VELOCITY_1CM = 0.01
robot.open_gripper()
robot.close_gripper()

print("start\n")
try:
    while True:
        # robot.set_position(position([-0.079, 0.236, 0.924], [-1.5184364318847656, -0.4188790321350098, 3.0368728637695312]), SPEED, MT_JOINT)
        robot.set_pose(pose([217.8804931640625, -78.25218200683594, -5.52130126953125, -107.50787353515625, -14.111251831054688, 183.25881958007812]),speed=SPEED)
        robot.await_motion()
        robot.open_gripper()
        # robot.set_position(position([-0.326, -0.049, 0.076], [-1.5707963705062866, 0, 0]), SPEED, MT_JOINT)
        robot.set_pose(pose([226.2662460599198, -128.0789054552217, -100.85526255076712, -131.06582816835908, 46.26624605991982, 179.99999547912873]), speed=SPEED)
        robot.await_motion()
        robot.open_gripper()
        # robot.set_position(position([-0.33, 0.017, 0.055], [-1.5707963705062866, 0, 0]), SPEED, MT_JOINT)
        robot.set_pose(pose([217.41287555304305, -128.65024317231138, -107.23506530467557, -124.114686326454, 37.412875553043094, 179.99999419889483]), speed=SPEED)
        robot.await_motion()
        robot.close_gripper()
        # robot.set_position(position([-0.322, 0.018, 0.379], [-1.5707963705062866, 0, 0]), SPEED, MT_JOINT)
        robot.set_pose(pose([218.14841456066696, -102.39591145893041, -65.89449037267514, -191.70959310763834, 38.148414560666886, 179.99999432607078]), speed=SPEED)
        robot.await_motion()
        # robot.set_position(position([-0.607, 0.046, 0.27], [-1.7976890802383423, -0.0349065847694874, 0.0349065847694874]), SPEED, MT_JOINT)
        robot.set_pose(pose([197.97414509681806, -143.58879105184516, -27.388911525944252, -148.95654250283593, 20.442732889339283, 144.22037994505]), speed=SPEED)
        robot.await_motion()
        robot.close_gripper()
        robot.open_gripper()
        # robot.set_position(position([-0.08, 0.24, 0.925], [-1.5707963705062866, -0.4363323152065277, 3.054326295852661]), SPEED, MT_JOINT)
        robot.set_pose(pose([220.68391413151318, -81.23687995992026, -4.377122576233717, -94.38598318465966, -15.683914005007352, 174.99999330363045]), speed=SPEED)
        robot.await_motion()
except RestApiException as e:
    pprint("Exception when calling RestRobot %s\n" % e)
