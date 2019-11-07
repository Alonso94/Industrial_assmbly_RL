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
print(response.status_code)
new_url = url + '/position?speed=10&motionType=joint'
response = requests.put(new_url, data=json.dumps(
{
    "point": {
        "x": -0.325,
        "y": -0.05,
        "z": 0.475
    },
    "rotation": {
        "roll": -1.57,
        "pitch": 0.0,
        "yaw": 0.0
    }
}))

time.sleep(1)

new_url=url+'/status/motors'
response=requests.get(new_url)
print(response.json()[1])

new_url=url+'/gripper/open'
requests.put(new_url)
time.sleep(1)
new_url=url+'/gripper/close'
requests.put(new_url)


