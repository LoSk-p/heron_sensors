#!/usr/bin/env python3
import matplotlib.pyplot as plt

with open('/home/alena/ang_vel') as f:
    x = []
    y = []
    vel = False
    for line in f:
        line = line.strip()
        if line == 'angular_velocity:':
            vel = True
        line = line.split(':')
        if line[0].strip() == 'secs':
            x.append(int(line[1].strip()) - 1621956623)
        if vel:
            if line[0] == 'z':
                z_vel = float(line[1].strip())
                y.append(z_vel)
                vel = False
x = range(42)
print(x)
print(len(y))

plt.plot(x,y)
plt.show()