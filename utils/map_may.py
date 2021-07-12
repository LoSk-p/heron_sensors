#!/usr/bin/env python3

import matplotlib.pyplot as plt
import os
from ast import literal_eval
import numpy as np
from scipy import interpolate


path = os.path.realpath(__file__)[:-16]

with open(f'{path}utils/test-may2', 'r') as f:
    lines = f.readlines()

i = 0
lat = []
lon = []
ph = []
for line in lines:
    line = line.strip()
    line = literal_eval(line)
    lat.append(float(line['lat']))
    lon.append(float(line['lon']))
    ph.append(float(line['ph']))

# print(len(lat))
# print(len(lon))
latitude = np.arange(min(lat), max(lat), 0.000001)
longitude = np.arange(min(lon), max(lon), 0.000001)


xx, yy = np.meshgrid(latitude, longitude)

spline = interpolate.Rbf(lat,lon,ph,function='multiquadric',smooth=1, episilon=1)
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
            print(zz[y][x])
            f.write(f"{str(x)};{str(y)};{str(zz[y][x])}\n")
            x += 0.25
        y += 0.25