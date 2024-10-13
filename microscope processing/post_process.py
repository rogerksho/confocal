import time
import serial
from funcs import *
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import csv

#filename = "scans/1722716630.csv"
#filename = "scans/0x_0y.csv"
#filename = "scans/1722807888.csv"
#filename = "scans/combined.csv"
#filename = "scans/1724904091.csv"
filename = "scans/climberstape_2mm_shifted.csv"
#filename = "scans/1728420326.csv"
#filename = "scans/reconstructed_corrupted.csv"


reverse_t = 0
med_filt_active = 1
med_filt_size = 7
plot_3d = 0

XY_step_size = 0.003175 # mm/step

z_map = list()

row_size = 1

with open(filename, newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    
    if reverse_t:
        ticker = True
        for row in spamreader:
            if ticker:
                z_map.append(row)
            else:
                z_map.append(list(reversed(row)))
            ticker = not ticker
    else:
        ctr = 0
        for row in spamreader:
            if ctr >= 5:
                z_map.append(row)
            else:
                ctr += 1
            
row_size = len(z_map)
column_size = len(z_map[0])

z_map_int = [[int(numeric_string)*360/16384/1.8*(0.003175) for numeric_string in row] for row in z_map]

max_z = np.amax(z_map_int)

for i in range(len(z_map_int)):
    for j in range(len(z_map_int[0])):
        z_map_int[i][j] = max_z - z_map_int[i][j]

if med_filt_active:
    z_map_int = signal.medfilt2d(z_map_int, kernel_size=med_filt_size)

if plot_3d:
    hf = plt.figure()
    ha = hf.add_subplot(111, projection='3d')
    
    x = np.arange(0, row_size*XY_step_size, XY_step_size)
    y = np.arange(0, column_size*XY_step_size, XY_step_size)
    
    print(len(x))
    print(len(z_map[0]))

    X, Y = np.meshgrid(x, y)  # `plot_surface` expects `x` and `y` data to be 2D
    ha.plot_surface(X, Y, z_map_int)
    
    plt.axis([0, max(x), 0, max(y)])

    plt.show()
else:
    plt.imshow(z_map_int, origin='lower', cmap='Greys')
    plt.show()