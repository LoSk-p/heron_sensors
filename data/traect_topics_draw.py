#!/usr/bin/env python3
import json

end = False
data = {"Lat": 0, "Lon": 0}
with open('gps.json', 'w') as gps:
    with open('gps_data_lodka', 'r') as file:
        for line in file:
            if end:
                data = {"Lat": 0, "Lon": 0}
            end = False
            line = line.strip()
            line = line.split(':')
            if line[0] == 'latitude':
                data["Lat"] = float(line[1].strip())
            if line[0] == 'longitude':
                data["Lon"] = float(line[1].strip())
                end = True
                gps.write(f'{{"Lat": {data["Lat"]}, "Lon": {data["Lon"]}}}\n')

