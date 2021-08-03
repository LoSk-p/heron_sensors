#!/usr/bin/env python3

import math
import numpy as np
import rospy
import matplotlib.pyplot as plt
import os
import logging, sys
import rospkg
logging.basicConfig(level=logging.INFO)
boat_number = 4

x = []
y = []
# x = [2, 5, 10, 9 ,5]
# y = [4, 8, 7, 3, 2]
rospack = rospkg.RosPack()
rospack.list() 
path = rospack.get_path('heron_sensors')
with open(f'{path}/utils/borders') as f:
    for line in f:
        line = line.split(';')
        x.append(float(line[0]))
        y.append(float(line[1]))

#plt.plot(x, y, 'bo')
#plt.show()


min_lat = min(x)
max_lat = max(x)
min_lon = min(y)
max_lon = max(y)
len_lat = max_lat - min_lat
len_lon = max_lon - min_lon
mid_lat = sum(x)/len(x)
k_lon = 111320
k_lat = 111319*math.cos(mid_lat*math.pi/180)
if len_lat > len_lon:
    lines = 2
    cols = boat_number // 2
    if boat_number % 2 == 1:
        cols += 1
else:
    cols = 2
    lines = boat_number // 2
    if boat_number % 2 == 1:
        lines += 1

areas = [[]]
for i in range(len(x)):
    areas[0].append({'lat': x[i], 'lon': y[i]})
#print(areas[0])
area_nomber = 0

while area_nomber < boat_number - 1:
    max_x = -5
    max_y = -5
    min_x = 1000
    min_y = 1000
    area_nom = len(areas) - 1
    logging.debug(f'area[nom]_min max {areas[area_nom]}')
    for coord in areas[area_nom]:
        logging.debug(f'coord min max {coord}')
        if coord['lat'] > max_x:
            max_x = coord['lat']
        if coord['lon'] > max_y:
            max_y = coord['lon']
        if coord['lat'] < min_x:
            min_x = coord['lat']
        if coord['lon'] < min_y:
            min_y = coord['lon']
        logging.debug(f'min_y {min_y}, max_y {max_y}')
    len_lat = (max_x - min_x)
    len_lon = (max_y - min_y)
    logging.debug(f'len lon {len_lon}')
    logging.debug(f'next area {areas}')
    if len_lat/k_lat > len_lon/k_lon:  # Если область широкая делим по вертикали
        div_lat = min_x + len_lat/2
        obl1 = []
        obl2 = []
        logging.debug(f'area[nom] {areas[area_nom]}')
        for j in range(len(areas[area_nom])):
            if areas[area_nom][j]['lat'] < div_lat:
                obl1.append(areas[area_nom][j])
            else:
                obl2.append(areas[area_nom][j])
            if j == len(areas[area_nom]) - 1:
                j_next = 0
            else:
                j_next = j + 1
            logging.debug(f'div_lat {div_lat}')
            logging.debug(f'shir obl1 norm dots: {obl1}')
            logging.debug(f'shir obl2 norm dots: {obl2}')
            if ((areas[area_nom][j]['lat'] < div_lat) and (areas[area_nom][j_next]['lat'] > div_lat)) or ((areas[area_nom][j]['lat'] > div_lat) and (areas[area_nom][j_next]['lat'] < div_lat)):
                a = (areas[area_nom][j]['lon'] - areas[area_nom][j_next]['lon'])/(areas[area_nom][j]['lat'] - areas[area_nom][j_next]['lat'])
                b = areas[area_nom][j]['lon'] - a*areas[area_nom][j]['lat']
                lon_per = a*div_lat + b
                obl1.append({'lat': div_lat, 'lon': lon_per})
                obl2.append({'lat': div_lat, 'lon': lon_per})
                logging.debug(f"shir per x:{areas[area_nom][j]['lat']}, y:{areas[area_nom][j]['lon']}, x_next:{areas[area_nom][j_next]['lat']}, y_next:{areas[area_nom][j_next]['lon']}, a: {a}, b: {b}")
                logging.debug(f'shir lon_per: {lon_per}, div_lat: {div_lat}')
                logging.debug(f'shir obl1: {obl1}')
                logging.debug(f'shir obl2: {obl2}')
        areas.pop(area_nom)
        areas.insert(0, obl1)
        areas.insert(0, obl2)
        area_nomber += 1
    else:
        logging.debug(f'area[nom] {areas[area_nom]}')
        logging.debug(f'dlin min_y {min_y}, max_y {max_y}')
        logging.debug(f'dlin len lon {len_lon}')
        div_lon = min_y + len_lon/2
        obl1 = []
        obl2 = []
        for j in range(len(areas[area_nom])):
            if areas[area_nom][j]['lon'] < div_lon:
                obl1.append(areas[area_nom][j])
            else:
                obl2.append(areas[area_nom][j])
            if j == len(areas[area_nom]) - 1:
                j_next = 0
            else:
                j_next = j + 1
                logging.debug(f'div_lon {div_lon}')
            logging.debug(f'dlin obl1 norm dots: {obl1}')
            logging.debug(f'dlin obl2 norm dots: {obl2}')
            if ((areas[area_nom][j]['lon'] < div_lon) and (areas[area_nom][j_next]['lon'] > div_lon)) or ((areas[area_nom][j]['lon'] > div_lon) and (areas[area_nom][j_next]['lon'] < div_lon)):
                if areas[area_nom][j]['lat'] != areas[area_nom][j_next]['lat']:
                    a = (areas[area_nom][j]['lon'] - areas[area_nom][j_next]['lon'])/(areas[area_nom][j]['lat'] - areas[area_nom][j_next]['lat'])
                    b = areas[area_nom][j]['lon'] - a*areas[area_nom][j]['lat']
                    lat_per = div_lon/a - b/a
                    logging.debug(f"dlin per x:{areas[area_nom][j]['lat']}, y:{areas[area_nom][j]['lon']}, x_next:{areas[area_nom][j_next]['lat']}, y_next:{areas[area_nom][j_next]['lon']}, a: {a}, b: {b}")
                    logging.debug(f'dlin lat_per: {lat_per}, div_lon: {div_lon}')
                else:
                    lat_per = areas[area_nom][j]['lat']
                    logging.debug(f"dlin per x:{areas[area_nom][j]['lat']}, y:{areas[area_nom][j]['lon']}, x_next:{areas[area_nom][j_next]['lat']}, y_next:{areas[area_nom][j_next]['lon']}, a: {a}, b: {b}")
                    logging.debug(f'dlin lat_per: {lat_per}, div_lon: {div_lon}')

                obl1.append({'lat': lat_per, 'lon': div_lon})
                obl2.append({'lat': lat_per, 'lon': div_lon})
                logging.debug(f'dlin obl1: {obl1}')
                logging.debug(f'dlin obl2: {obl2}')
        areas.pop(area_nom)
        areas.insert(0, obl1)
        areas.insert(0, obl2)
        area_nomber += 1
            
print(areas)

with open(f'{path}/utils/borders_several', 'w') as f:
    for area in areas:
        f.write(f'{area}\n')

for area in areas:
    x_dr = []
    y_dr = []
    #print(area)
    for dot in area:
        x_dr.append(dot['lat'])
        y_dr.append(dot['lon'])
        #print(x_dr,y_dr)
    x_dr.append(x_dr[0])
    y_dr.append(y_dr[0])
    plt.plot(x_dr,y_dr)
ax = plt.gca()
ax.set_xlabel("Широта")
ax.set_ylabel("Долгота")
ax.set_xlim([49.8987, 49.9001])
plt.show()