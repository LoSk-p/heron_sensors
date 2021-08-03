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
import rospkg

boat_number = 3
rospack = rospkg.RosPack()
rospack.list() 
path = rospack.get_path('heron_sensors')
linsx = [[],[],[]]
linsy = [[],[],[]]
colors = ['r', 'b', 'g']
for i in range(boat_number):
    color = colors[i]

    with open(f'{path}/utils/logs/{os.argv[1]}_{i}') as f:
        for line in f:
            line = line.split(';')
            # js = literal_eval(line)
            # print(line)
            linsx[i].append(float(line[0]))
            linsy[i].append(float(line[1]))
for i in range(boat_number):
    plt.plot(linsx[i], linsy[i], color=colors[i])
plt.show()
