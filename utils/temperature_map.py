#!/usr/bin/env python3
import random
from ast import literal_eval
import matplotlib.pyplot as plt

timestamp = 1605639398
f2 = open('map', 'w')



with open("../scripts/coord-err-1m") as f1:
    for line in f1:
        lat = float(line.split(';')[0])
        lon = float(line.split('; ')[1].split('\n')[0])
        timestamp += 5
        ph = round(random.uniform(4.5, 8.3), 3)
        cond = round(random.uniform(0.01, 1), 3)
        data = {"lat": lat, "lon": lon, "timestamp": timestamp, "ph": ph, "conductivity": cond}
        f2.write(f'{data} \n')

f2.close()

# coordinates for the center of pollution ()
x0 = 49.89973114614826
y0 = 8.900325679552948
r = 0.0002
f3 = open('with_temp', 'w')
x = []
y = []
z = []
with open('map') as f:
    for line in f:

        js = literal_eval(line)
        lat = js["lat"]
        lon = js["lon"]
        if r**2 >= ((lat - x0)**2 + (lon-y0)**2):
            temp = (((lat - x0)**2 ) + (lon-y0)**2)*(10**(8))

        else:
            temp = 20                                                                                  
        x.append(lat)
        y.append(lon)
        z.append(temp)
        js.update(temp= temp)
        f3.write(f'{js} \n')

f3.close()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='parametric curve')
plt.show()







