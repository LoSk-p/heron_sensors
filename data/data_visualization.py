#!/usr/bin/env python3.8

import folium
from folium import plugins
import json
from dataclasses import dataclass, field
import pandas as pd


@dataclass
class DataVisualization:
    lat: list = field(default_factory=list)
    lon: list = field(default_factory=list)
    ph: list = field(default_factory=list)
    temp: list = field(default_factory=list)
    cond: list = field(default_factory=list)
    data: list = field(default_factory=list)
    location: list = field(default_factory=list)
    filename: str = "test-may2"

    def __post_init__(self): 
        d = {}
        with open(self.filename, 'r') as f:
            for line in f:
                line = json.loads(line)
                
                self.lat.append(float(line["lat"]))
                self.lon.append(float(line["lon"]))
                self.temp.append(line["temperature"])
                self.cond.append(line["conductivity"])
                self.ph.append(line["ph"])
                self.location.append([self.lat, self.lon])
            
            d.update({"lat": self.lat, "lon": self.lon, "temperature": self.temp,
            "conductivity": self.cond, "ph": self.ph})
            self.data = pd.DataFrame.from_dict(d)
            # print(self.data)


        
    def drawing(self):

        map = folium.Map(location=[59.868842, 30.301311], zoom_start = 25)
        routegroup = folium.FeatureGroup(name='Route_Layer', control=True)
        tempgroup = folium.FeatureGroup(name='Temperature_Layer', control=True)
        # folium.CircleMarker(location=[self.lat, self.lon], radius=1).add_to(routegroup)

        folium.Choropleth(
            geo_data=self.location,
            name='Population in 2018',
            data=self.data,
            columns=['lat', 'temperature'],
            fill_color='YlOrRd',
            fill_opacity=0.7,
            line_opacity=0.2,
            line_color='white', 
            line_weight=0,
            highlight=False, 
            smooth_factor=1.0,
            #threshold_scale=[100, 250, 500, 1000, 2000],
            legend_name= 'Population in Helsinki').add_to(tempgroup)

        folium.LayerControl().add_to(map)

        map.save("data-map.html")


if __name__ == '__main__':  
    try:  
        vis = DataVisualization()
        vis.drawing()
        
    except KeyboardInterrupt:
        print("exception")
        exit()



