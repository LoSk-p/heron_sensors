#!/usr/bin/env python3

# Построение маршрута по заданным координатам границ области

import math
import numpy as np
import rospy
import matplotlib.pyplot as plt
import os
from ast import literal_eval
import shutil

#x = [49.8995632732, 49.8992564727, 49.899885637, 49.8999917653, 49.8998692169]
#y = [8.89912952085, 8.90169695784, 8.90068128035, 8.90012873463, 8.89840325559]
path = os.path.realpath(__file__)[:-28]
shutil.rmtree(f'{path}utils/ways/')
os.mkdir(f'{path}utils/ways')
nom_area = 0
with open(f'{path}utils/borders_several') as f:
    for line in f:
        x = []
        y = []
        line = line[1:-3]
        line = line.split('},')
        #print(line)
        for coord in line:
            #print(coord.strip())
            coord = literal_eval(coord.strip() + "}")
            x.append(float(coord['lat']))
            y.append(float(coord['lon']))
            #print(x)
            #print(y)
        for i in range(len(x)):
            for j in range(len(x)):
                if (x[i] == x[j]) and (i != j):
                    x[j] += 10**(-6)
        plt.plot(x, y)
        #plt.show()
        step = 15

        def next_prev(i):
            if i == 0:
                i_prev = len(x) - 1
                i_next = i + 1
            elif i == (len(x) - 1):
                i_next = 0
                i_prev = i - 1
            else:
                i_next = i + 1
                i_prev = i - 1
            return [i_next, i_prev]
                
        # 1) Перевод координат в метры 1 широты - 111320 м
        #                           1 долготы - 111319.44 * cos(широта_средняя)
        # 2) Шаг 1 метр
        # 3) Шаг широты - cos угла наклона = cos(y2-y1)/(x2-x1)

        min_lat = min(x)
        max_lat = max(x)
        mid_lat = sum(x)/len(x)
        k_lon = 111320
        k_lat = 111319*math.cos(mid_lat*math.pi/180)

        way = []
        way_gr = [[],[]]
        x_boat = min_lat
        i_boat = x.index(x_boat)
        y_boat = y[i_boat]
        [i_next, i_prev] = next_prev(i_boat)
        i1 = i2 = i_boat
        line1 = [[x[i_boat], y[i_boat]],[x[i_next], y[i_next]]]
        line2 = [[x[i_boat], y[i_boat]],[x[i_prev], y[i_prev]]]
        way.append({'lat': x_boat, 'lon': y_boat})
        way_gr[0].append(x_boat)
        way_gr[1].append(y_boat)
        x_boat1 = x_boat2 = x_boat
        while line1[1] != line2[1]:
            #plt.plot(way[0],way[1])
            #plt.show()
            [i_next1, i_prev1] = next_prev(i1)
            [i_next2, i_prev2] = next_prev(i2)
            line1 = [[x[i1], y[i1]],[x[i_next1], y[i_next1]]]
            line2 = [[x[i2], y[i2]],[x[i_prev2], y[i_prev2]]]
            #print(line1, line2)
            angle1 = math.atan((line1[1][1]-line1[0][1])/(line1[1][0]-line1[0][0]))
            b1 = line1[0][1] - math.tan(angle1)*line1[0][0]
            angle2 = math.atan((line2[1][1]-line2[0][1])/(line2[1][0]-line2[0][0]))
            b2 = line2[0][1] - math.tan(angle2)*line2[0][0]
            step1_x = step*math.cos(angle1)/k_lat
            step2_x = step*math.cos(angle2)/k_lat
            while x_boat1 != line1[1][0] and x_boat2 != line2[1][0]:
                if abs(x_boat1 - line1[1][0]) > step1_x:
                    if x_boat1 > line1[1][0]:
                        x_boat1 -= step1_x
                    else:
                        x_boat1 += step1_x
                else:
                    x_boat1 = line1[1][0]
                    i1 = i_next1
                y_boat = math.tan(angle1)*x_boat1 + b1
                way_gr[0].append(x_boat1)
                way_gr[1].append(y_boat)
                way.append({'lat': x_boat1, 'lon': y_boat})

                if abs(x_boat2 - line2[1][0]) > step2_x:
                    if x_boat2 > line2[1][0]:
                        x_boat2 -= step2_x
                    else:
                        x_boat2 += step2_x
                else:
                    x_boat2 = line2[1][0]
                    i2 = i_prev2
                y_boat = math.tan(angle2)*x_boat2 + b2
                way_gr[0].append(x_boat2)
                way_gr[1].append(y_boat)
                way.append({'lat': x_boat2, 'lon': y_boat})
                #print([x_boat2, y_boat])
        print(nom_area)
        with open(f'{path}utils/ways/way{nom_area}', 'w') as f:
            for coord in way:
                f.write(f'{coord}\n')
        nom_area += 1
        plt.plot(way_gr[0],way_gr[1])
        #plt.show()
#error 5m
#result_way = [[49.898851720556436, 49.8988499747083, 49.89884971649555, 49.89898866328749, 49.89896317061848, 49.89895492292866, 49.89895780983654, 49.89916105977763, 49.899084025604715, 49.89932937961646, 49.899187802988315, 49.89949812888644, 49.89926805661681, 49.89956492948694, 49.899317332501475, 49.899718350801585, 49.899381558134166, 49.899726940286484, 49.899433533657955, 49.89973827445681, 49.899490494254906, 49.899768134327715, 49.89953726014872, 49.89978905617401, 49.89959003031277, 49.89979719461467, 49.89963615213545, 49.899794875778205, 49.89970723445916, 49.89980342232466, 49.89979462216071, 49.89982147672671, 49.89983444982033, 49.89983771812855, 49.89983948073573], [8.901666714429597, 8.901667885864237, 8.901665035276329, 8.901453992848113, 8.901333732269743, 8.901317345298633, 8.901311623352505, 8.901192345102128, 8.900940688588344, 8.900971358854687, 8.900737403430588, 8.90083155921828, 8.900536069508615, 8.90069567096478, 8.90033732675066, 8.900640001553205, 8.900140571249887, 8.900428320326933, 8.899940548518389, 8.900222296979992, 8.899743588427679, 8.900085336831257, 8.899543926550034, 8.899885844300586, 8.899343711444226, 8.89967680665668, 8.89917241078062, 8.899464159770805, 8.899001927066138, 8.899251902131475, 8.898936925200793, 8.898934631540898, 8.898945390680543, 8.89893872005729, 8.898933027813174]]
#error 1m
#result_way = [[49.898814142224815, 49.89887537728009, 49.898854951649, 49.89895339617646, 49.89891450444299, 49.89890735873214, 49.89893404287132, 49.89920977791276, 49.899062514232185, 49.899430987555206, 49.89914949394947, 49.899568949922354, 49.89920261169182, 49.899727968719915, 49.89927603813837, 49.89982890655062, 49.89932709525126, 49.89982712078735, 49.89938534626436, 49.89982554161595, 49.89944862406383, 49.89984273672956, 49.8995057563064, 49.89983857935558, 49.89955678900811, 49.899843362792474, 49.899601215571586, 49.899833377104684, 49.89967555702544, 49.89985261446259, 49.89972785777787], [8.901705746081904, 8.901563951206011, 8.901495667000578, 8.901428971973985, 8.901295655167777, 8.901278293444014, 8.901131401105358, 8.901140253155333, 8.900904199266245, 8.9010027695183, 8.900707858830229, 8.900865335429929, 8.900504407391598, 8.900729237849141, 8.900307404975047, 8.900674114714246, 8.900104577206212, 8.900464606394396, 8.899905023725648, 8.900255039342566, 8.899708668985012, 8.90012433497504, 8.899509762138578, 8.89991409660089, 8.89930516631969, 8.899706224161665, 8.899126490654679, 8.899493022816541, 8.89893545407503, 8.899294699321434, 8.898712977434187]]
#plt.plot(result_way[0], result_way[1])
ax = plt.gca()
ax.set_xlabel("Широта")
ax.set_ylabel("Долгота")
plt.show()