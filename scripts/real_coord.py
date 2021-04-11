#!/usr/bin/env python3

import rospy
import json
from heron_msgs.msg import Drive, Course
from sensor_msgs.msg import JointState, NavSatFix
from geometry_msgs.msg import Vector3Stamped
from time import sleep
import os
import yaml
import rosnode
import math
import matplotlib.pyplot as plt
from ast import literal_eval


path = os.path.realpath(__file__)[:-21]

x1 = 59.869323 
y1 = 30.300351
x2 = 59.868426 
y2 = 30.302579

with open(f'{path}/utils/ways/way0') as f:
    file_c = open(f'{path}/utils/ways/way1', 'w')
    for line in f:
        js = literal_eval(line)
        js['lat'] -= 9.96939886
        js['lon'] -= 21.4033096
        file_c.write(str({'lat': js["lat"], 'lon': js["lon"]}) + '\n')
    file_c.close()

# coord = []
# with open(f'{path}/utils/data_drone.json') as f:
#     file_c = open(f'{path}/utils/ways/way0', 'w')
#     for line in f:
#         try:
#             js = literal_eval(line)
#             #print(js)
#             if (js["Lat"] > x2) and (js["Lat"] < x1) and (js["Lon"] > y1) and (js["Lon"] > y1):
#                 coord.append({'lat': js["Lat"], 'lon': js["Lon"]})
#                 file_c.write(str({'lat': js["Lat"], 'lon': js["Lon"]}) + '\n')
#         except:
#             pass
#     file_c.close()

        


