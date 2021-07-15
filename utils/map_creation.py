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

'''
Create a simulation map from experimental data. For execution requires filenames from experimental_data folder as parameters.
Parameter 1 is name of the file with gps and time data
Parameter 2 is name of the file with data from sensors
 
Example: python3 map_creation.py gps_time_lodka sensor_data_with_time
'''

path = os.path.realpath(__file__)[:-21]

def files_merge(gps_file, sensor_data_file) -> str:

    merged_file = f'merged_data_{time()}'
    with open(f'{path}utils/experimental_data/{gps_file}') as f1, open(f'{path}utils/experimental_data/{merged_file}', "w") as f3:
        for line1 in f1:
            with open(f'{path}utils/experimental_data/{sensor_data_file}') as f2:
                for line2 in f2:
                    line2 = literal_eval(line2)
                    time1 = float(line1.split(' ')[1].split(";")[0])
                    lat = float(line1.split(' ')[3].split(";")[0])
                    lon = float(line1.split(' ')[5].split("\n")[0])
                    
                    time2 = float(line2["time"])
                    temperature = line2["temperature"]
                    ph = line2["pH"]
                    conductivity = line2["conductivity"]

                    if int(time1) == int(time2):
                        f3.write(f'{{"time": {time1}, "lat": {lat}, "lon": {lon}, "temperature": {temperature}, "conductivity": {conductivity}, "ph": {ph}}}\n')
    
    return merged_file


def map_creation():
    merged_file = files_merge(sys.argv[1], sys.argv[2])
    with open(f'{path}utils/experimental_data/{merged_file}', 'r') as f:
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

    latitude = np.arange(min(lat), max(lat), 0.000001)
    longitude = np.arange(min(lon), max(lon), 0.000001)
    xx, yy = np.meshgrid(latitude, longitude)
    spline = interpolate.Rbf(lat,lon,temp,function='multiquadric',smooth=1, episilon=1)
    zz = spline(xx,yy)
    dataMatrix = np.matrix(zz)
    np.savetxt(f'{path}utils/map/for_control', dataMatrix, delimiter = ' ', fmt='%.2f')

    with open(f'{path}utils/map/for_painting', 'w') as f:
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

if __name__ == '__main__':  
    try:  
        map_creation()
    except KeyboardInterrupt:
        exit()