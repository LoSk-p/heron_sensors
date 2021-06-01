#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt

lat = []
lon = []
with open("gps.json", 'r') as f:
    for line in f:
        line = json.loads(line)
        lat.append(line["Lat"])
        lon.append(line["Lon"])

plt.plot(lat, lon, label='Реальный дрон')

boat_nomber = 1
linsx = [[],[],[]]
linsy = [[],[],[]]
colors = ['r', 'b', 'g']
for i in range(boat_nomber):
    color = colors[i]
    with open("../utils/0coord-err-1m") as f:
        for line in f:
            line = line.split(';')
            # js = literal_eval(line)
            # print(line) 
            linsx[i].append(float(line[0]))
            linsy[i].append(float(line[1]))
for i in range(boat_nomber):
    plt.plot(linsx[i], linsy[i], color=colors[i], label='Виртуальный дрон')

ax = plt.gca()
ax.set_xlabel("Широта")
ax.set_ylabel("Долгота")
ax.legend()
plt.show()