import rospy
import json
from heron_msgs.msg import Drive, Course, Helm
from sensor_msgs.msg import JointState, NavSatFix, Imu
from geometry_msgs.msg import Vector3Stamped, Twist
from heron_sensors.msg import DronesStatus
from mavros_msgs.msg import OverrideRCIn
import time
import yaml
import rosnode
import math
import matplotlib.pyplot as plt
import os
from ast import literal_eval
import numpy as np


real = False
with_pollution_looking = True
looking_value_temp = 7.5
path = os.path.realpath(__file__)[:-16]
fig, ax = plt.subplots()
if with_pollution_looking:
    with open(f'{path}utils/ph-with-coords.csv') as map_temp:
        lat = []
        lon = []
        temp = []
        for line in map_temp:
            line = line.split(';')
            # lat.append(float(line[0]))
            # lon.append(float(line[1]))
            temp.append(float(line[2]))
    # lat = np.arange(49.8988, 49.8988 + 201*5.47*10**(-6), 5.47*10**(-6))
    # lon = np.arange(8.89844, 8.89844 + 201*1.612*10**(-5), 1.612*10**(-5))
    lat = np.arange(0, 50.25, 0.25)
    lon = np.arange(0, 50.25, 0.25)
    print(f'lat {lat}')
    X, Y = np.meshgrid(lat, lon)
    print(f'X, Y {X}, {Y}')
    temp = np.array(temp)
    temp = temp.reshape((201, 201))
    temp = np.transpose(temp)
    print(f'temp: {temp}')
    with open(f'{path}utils/ph-grid_new.csv', 'w') as f:
        for i in range(len(temp[0])):
            for j in range(len(temp)):
                if lon[j] > 41.28:
                    temp[j][i] += 0.25
                elif (lon[j] > 27.75) and (lon[j] < 39.4) and (lat[i] < 6.14):
                    temp[j][i] += 0.25
                elif (lon[j] > 26.67) and (lon[j] < 33.3) and (lat[i] > 17.64) and (lat[i] < 24.69):
                    temp[j][i] += 0.25
                elif (lon[j] < 14.36) and (lat[i] > 17.33):
                    temp[j][i] -= 0.3
                elif (lon[j] > 14.72) and (lon[j] < 27.35) and (lat[i] > 32):
                    temp[j][i] -= 0.4
                elif (lon[j] > 21.61) and (lon[j] < 27.35) and (lat[i] > 26.68) and (lat[i] < 29.4):
                    temp[j][i] += 0.25
                if j != (len(temp) - 1):
                    f.write(f'{temp[i][j]};')
                else:
                    f.write(f'{temp[i][j]}')
            f.write(f'\n')
            
            
    # self.fig, self.ax = plt.subplots(1, 2)
    #self.fig, self.ax = plt.subplots()
    # CS = self.ax[0].contour(X, Y, temp)
    # self.ax[0].clabel(CS, inline=True, fontsize=10)
    # self.ax[0].set_title('Simplest default with labels')
    CS_common = ax.contour(X, Y, temp, 20)
    CS_lvl = ax.contour(X, Y, temp, levels=[7.5], colors='b')

    ax.clabel(CS_common, inline=True, fontsize=10)
    ax.clabel(CS_lvl, inline=True, fontsize=10)
    ax.set_title('Simplest default with labels')
    plt.show()