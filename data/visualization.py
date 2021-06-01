#!/usr/bin/env python3

import folium
import json

map = folium.Map(location=[59.868842, 30.301311], zoom_start = 25)

with open("gps.json", 'r') as f:
    for line in f:
        line = json.loads(line)
        try:
            
            print("in try")
            lat = float(line["Lat"])
            lon = float(line["Lon"])
            # temp = line["temp"]
            # cond = line["cond"]
            # ph = line["ph"]
            if lat != 0:
                # folium.CircleMarker(location=[lat, lon], popup=f'temperature: {temp} \n conductivity: {cond} \n ph: {ph}', radius=15).add_to(map)
                folium.CircleMarker(location=[lat, lon], popup=f'temperature: {1} \n conductivity: {1} \n ph: {1}', radius=15).add_to(map)

        except KeyError:
            pass


map.save("map1.html")
