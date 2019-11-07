import requests
import json
import time
from getpass import getpass
url="http://192.168.1.87:8080"

response=requests.get(url)
if response.status_code==200:
    print("success")
response=requests.get(url,'GET/pose')
print(response.status_code)
