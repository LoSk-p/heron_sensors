#!/usr/bin/env python3

import matplotlib.pyplot as plt
import os
from ast import literal_eval
import numpy as np
from scipy import interpolate
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D


path = os.path.realpath(__file__)[:-16]

with open(f'{path}utils/test-may2', 'r') as f:
    lines = f.readlines()

i = 0
lat = []
lon = []
temp = []
for line in lines:
    line = line.strip()
    line = literal_eval(line)
    lat.append(float(line['lat']))
    lon.append(float(line['lon']))
    temp.append(float(line['temperature']))

# print(len(lat))
# print(len(lon))
latitude = np.arange(min(lat), max(lat), 0.000001)
longitude = np.arange(min(lon), max(lon), 0.000001)


xx, yy = np.meshgrid(latitude, longitude)

spline = interpolate.Rbf(lat,lon,temp,function='multiquadric',smooth=1, episilon=1)
zz = spline(xx,yy)
print(zz)
dataMatrix = np.matrix(zz)
print(dataMatrix)
np.savetxt('ph_may.txt', dataMatrix, delimiter = ' ', fmt='%.2f')

with open('ph-may-grid.csv', 'w') as f:
    y = 0
    for line in zz:
        x = 0
        for l in line:
            #print(zz[y][x])
            f.write(f"{str(x)};{str(y)};{str(zz[y][x])}\n")
            x += 1
        y += 1

fig = plt.figure()
fig.suptitle('pH level')
ax = fig.add_subplot(111, projection='3d')
#ax.scatter3D(lon, lat, ph, c='r')
ax.plot_wireframe(xx, yy, zz)
ax.plot_surface(xx, yy, zz,alpha=0.2)
ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
plt.show()