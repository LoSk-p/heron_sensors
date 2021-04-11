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

boat_nomber = 3
path = os.path.realpath(__file__)[:-22]
linsx = [[],[],[]]
linsy = [[],[],[]]
colors = ['r', 'b', 'g']
#for i in range(boat_nomber):
color = colors[0]
with open(f'{path}utils/ways/way1') as f:
    for line in f:
        js = literal_eval(line)
        print(line)
        linsx[0].append(float(js['lat']))
        linsy[0].append(float(js['lon']))
#for i in range(boat_nomber):
plt.plot(linsx[0], linsy[0], color=colors[0])
plt.show()
