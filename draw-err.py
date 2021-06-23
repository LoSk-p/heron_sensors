#!/usr/bin/env python3
import matplotlib.pyplot as plt
x = []
y = []

with open("error") as f1:
    for line in f1:
        x.append(float(line.split("; ")[0]))
        y.append(float(line.split("; ")[1].rstrip()))
        

plt.plot(x, y)
plt.show()
# print(x)
# print(y)