#!/usr/bin/env python3

import folium
import json

map = folium.Map(location=[47.621763, 9.422411], zoom_start = 10)

with open("test_30|04.json", 'r') as f:
    for line in f:
        line = json.loads(line)
        try:
            
            print("in try")
            lat = float(line["Lat"])
            lon = float(line["Lon"])
            temp = line["temp"]
            cond = line["cond"]
            ph = line["ph"]
            if lat != 0:
                folium.CircleMarker(location=[lat, lon], popup=f'temperature: {temp} \n conductivity: {cond} \n ph: {ph}', radius=15).add_to(map)

        except KeyError:
            pass


map.save("map1.html")
