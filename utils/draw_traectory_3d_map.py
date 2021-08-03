#!/usr/bin/env python3

import matplotlib.pyplot as plt
import os
from ast import literal_eval
import numpy as np
from scipy import interpolate
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D
from time import time
import sys
import rospkg

filename = sys.argv[1]

rospack = rospkg.RosPack()
rospack.list() 
path = rospack.get_path('heron_sensors')
print(path)

x0 = 49.8988
y0 = 8.89844
x1 = 49.8998995
y1 = 8.90168012
shape_x = 413 + 1
shape_y = 1248 + 1
step_x = (x1 - x0)/shape_x
step_y = (y1 - y0)/shape_y

lat = np.arange(x0, x1, step_x)
lon = np.arange(y0, y1, step_y)
lat, lon = np.meshgrid(lat, lon)
data = []

with open(f'{path}/utils/map/for_control') as map_temp:
    for line in map_temp:
        line = line.split()
        data_line = []
        for el in line:
            data_line.append(float(el))
        data.append(data_line)

data = np.matrix(data)
fig = plt.figure()
fig.suptitle('pH level')
ax = fig.add_subplot(111, projection='3d')
ax.plot_wireframe(lat, lon, data, zorder=1)
ax.plot_surface(lat, lon, data, alpha=0.2, zorder=2)
# ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
# ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))

lat = []
lon = []
value = []
with open(f"{path}/utils/logs/{filename}") as f:
    for line in f:
        line = literal_eval(line)
        lat.append(float(line["lat"]))
        lon.append(float(line["lon"]))
        value.append(float(line["value"]))
ax.plot(lat, lon, value, "r")
ax.scatter(lat[0], lon[0], value[0], color="r", s = 150, marker="*", zorder=5, label="Start point")
ax.legend()
plt.show()


