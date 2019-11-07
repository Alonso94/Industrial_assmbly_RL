import requests
import json
import time
from getpass import getpass

url = "http://10.10.10.20:8081"

response = requests.get(url)
if response.status_code == 200:
    print("success")
new_url = url + '/pose'
print(new_url)
response = requests.get(new_url)
print(response.json()['angles'])
print(response.status_code)
new_url=url+'/freeze'
response=requests.put(new_url)
new_url = url + '/status/motion'
response = requests.get(new_url)
print(response.content==b'"IDLE"')
response=requests.put(url+'/pose?speed=10',data={
            "angles": [243.06298828125, -100.13008117675781, -77.58268737792969, -2.41265869140625, -33.067474365234375, 0.070037841796875]

        })
print(response.content)
# while(1):
#     response = requests.get(url + '/pose')
#     print(response.json()['angles'])
#     new_url = url + '/position?speed=10&motionType=joint'
#     response = requests.put(new_url, data=json.dumps(
#     {
#         "point": {
#             "x": -0.325,
#             "y": 0.05,
#             "z": 0.475
#         },
#         "rotation": {
#             "roll": -1.57,
#             "pitch": 0.0,
#             "yaw": 0.0
#         }
#     }))
#     new_url = url + '/status/motion'
#     response = requests.get(new_url)
#     while (response.content!=b'"IDLE"'):
#         response = requests.get(new_url)
#     new_url = url + '/position?speed=10&motionType=joint'
#     response = requests.put(new_url, data=json.dumps(
#     {
#         "point": {
#             "x": -0.325,
#             "y": 0.05,
#             "z": 0.675
#         },
#         "rotation": {
#             "roll": -1.57,
#             "pitch": 0.0,
#             "yaw": 0.0
#         }
#     }))
#     new_url = url + '/status/motion'
#     response = requests.get(new_url)
#     while (response.content!=b'"IDLE"'):
#         response = requests.get(new_url)
#
# new_url=url+'/status/motors'
# response=requests.get(new_url)
# print(response.json()[1])
#
# new_url=url+'/gripper/open'
# requests.put(new_url)
# time.sleep(1)
# new_url=url+'/gripper/close'
# requests.put(new_url)
#
#
