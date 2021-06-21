#!/usr/bin/env python3

from ast import literal_eval

with open("gps_time_lodka") as f1, open("test-may2", "a") as f3:

    for line1 in f1:
        with open("sensor_data_with_time") as f2:
            for line2 in f2:
                line2 = literal_eval(line2)

                time1 = float(line1.split(' ')[1].split(";")[0])
                lat = float(line1.split(' ')[3].split(";")[0])
                lon = float(line1.split(' ')[5].split("\n")[0])
                
                time2 = float(line2["time"])
                temperature = line2["temperature"]
                ph = line2["pH"]
                conductivity = line2["conductivity"]
                print(int(time1), int(time2))
                if int(time1) == int(time2):

                    f3.write(f'{{"time": {time1}, "lat": {lat}, "lon": {lon}, "temperature": {temperature}, "conductivity": {conductivity}, "ph": {ph}}}\n')





