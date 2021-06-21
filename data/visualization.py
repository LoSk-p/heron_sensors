#!/usr/bin/env python3

import folium
from folium import plugins
import json

map = folium.Map(location=[59.868842, 30.301311], zoom_start = 25)

gridgroup = folium.FeatureGroup(name='Grid_Layer', control=True)
##Create a FeatureGroup of Marker
pointsgroup = folium.FeatureGroup(name='Points_Layer', control=True)
# marker_cluster = plugins.MarkerCluster().add_to(pointsgroup)
lat = {}
lon = {}
ph = {}
temp = {}
cond = {}

with open("test-may2", 'r') as f:
    for line in f:
        line = json.loads(line)
        try:
        
            lat = float(line["lat"])
            lon = float(line["lon"])
            temp = line["temperature"]
            cond = line["conductivity"]
            ph = line["ph"]
            
            if lat != 0:
                # folium.CircleMarker(location=[lat, lon], popup=f'temperature: {temp} \n conductivity: {cond} \n ph: {ph}', radius=15).add_to(map)
                folium.CircleMarker(location=[lat, lon], radius=1).add_to(pointsgroup)

        except Exception as e:
            print(e)
map.add_child(pointsgroup)
map.add_child(gridgroup)
folium.LayerControl().add_to(map)

map.save("map1.html")
