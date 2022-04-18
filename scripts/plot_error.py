#!/usr/bin/env python3

from cProfile import label
import numpy as np
import math
import matplotlib.pyplot as plt

dx = np.array([-0.04, -0.04, -0.03, -0.03, -0.03, -0.79, -0.53, 0.47, 0.65, 0.65])
gtx = np.array([0.3, 0.2, -0.03, 0.2, 0.06, -0.7, -0.53, 0.65, 0.73, 0.91])
dy = np.array([1.76, 1.2, 1.13, 1.54, 1.32, 0.67, 0.85, 1, 0.9, 1.38])
gty = np.array([1.98, 1.25, 1.14, 1.53, 1.34, 0.6, 0.85, 0.81, 0.65, 1.17])


ex = (dx - gtx) / gtx
ey = (dy - gty) / gty
print(ex)
print(ey)

plt.subplot(1,3,1)
for i in range(len(dx)):
    plt.plot([dx[i],gtx[i]], [dy[i],gty[i]], 'r')
plt.scatter(dx, dy, label="Computed")
plt.scatter(gtx, gty, label="Ground Truth")
plt.title("Ground Truth v.s. Computed Positions (m)")
plt.legend()

bins = np.arange(10) / 10.0 - 0.5
print(bins)
plt.subplot(1,3,2)
plt.hist(ex, bins)
plt.title("X Position Error")

plt.subplot(1,3,3)
plt.hist(ey, bins)
plt.title("Y Position Error")

plt.show()