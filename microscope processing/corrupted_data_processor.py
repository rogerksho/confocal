import time
import serial
from funcs import *
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import csv

filename = "scans/corrupted_data.txt"

ctr = 1

preamble = "x1 = 0.00 y1 = 0.00 x2 = 1.00 y2 = 1.00 scan width is 1.00mm scan length is 1.00mm scan_height is 1.00mm scan size is 316 x 316 x 631 starting scan..."

reconstructed_map = [[0 for i in range(0, 316)] for j in range(0, 316)]

with open(filename) as fp:
    lines = fp.readlines()
    for line in lines:
        split_string = line.split(':')
        coords = split_string[0][1:-1].split(',')
        x = int(coords[0])
        y = int(coords[1])
        
        data = int(split_string[1])
        
        reconstructed_map[x][y] = data
        
with open(f"./scans/reconstructed_corrupted.csv","w+") as my_csv:
    csvWriter = csv.writer(my_csv,delimiter=',')
    preWriter = csv.writer(my_csv, delimiter='\t',
        lineterminator='\r\n',
        quoting = csv.QUOTE_NONE
        )

    csvWriter.writerows(reconstructed_map)