#!/usr/bin/env python3
import rospy
import json
from heron_msgs.msg import Drive, Course, Helm
from sensor_msgs.msg import JointState, NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped, Twist
from heron_sensors.msg import DronesStatus
from mavros_msgs.msg import RCIn
import time
import yaml
import rosnode
import math
import matplotlib.pyplot as plt
import os
from ast import literal_eval
import numpy as np

class Viz:
    def __init__(self):
        rospy.init_node('vizualiser')
        self.lat = 0
        self.lon = 0
        self.num = 0
        rospy.Subscriber('/drones_status', DronesStatus,self.viz)

    def viz(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.num = data.boat_nomber

    def spin(self):
        colors = ['r', 'b', 'c', 'm', 'g', 'y']
        while True:
            if self.lat != 0 and self.lon != 0:
                plt.plot(self.lat, self.lon, f'{colors[self.num]}o')
                plt.draw()
                plt.pause(0.01)

Viz().spin()