#!/usr/bin/python
# -*- encoding: utf-8 -*-

#  ______                              _
#  / _____)             _              | |
# ( (____  _____ ____ _| |_ _____  ____| |__
#  \____ \| ___ |    (_   _) ___ |/ ___)  _ \
#  _____) ) ____| | | || |_| ____( (___| | | |
# (______/|_____)_|_|_| \__)_____)\____)_| |_|
#   (C)2020 Semtech
#
# Description:
#    Spectral Scan CSV result file plot - v0.3.0
#
# License: Revised BSD License, see LICENSE.TXT file include in the project

#Library importation
import pylab as pl
import numpy as np
import csv
import sys

#Read argument
if len(sys.argv) >= 2:
    filename = sys.argv[1]
else:
    print ("Usage: %s <filename>" %sys.argv[0])
    sys.exit()

#Initiate array
rssi = []
freq = []

#Process .csv file
with open(filename, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in reader:
        f=int(row[0])//1000 #frequency
        freq.append(f)
        rssi_line=[]
        rssi_val=[]
        for k in range(1,(len(row)-1)//2):
            rssi_line.append(int(row[k*2]))
            rssi_val.append(int(row[k*2-1]))
        rssi.append(rssi_line)

#Set x to frequency axis and y to signal level axis
A = np.array(rssi).T
A = np.flip(A,0)
rssi_val =np.flip(rssi_val,0)

#Find max value to scale heatmap then plot.
maxx = max(max(rssi))
print(maxx, A)
h=pl.imshow(A , cmap='afmhot', vmin=0, vmax=maxx, aspect='auto', origin='lower', interpolation='nearest')
pl.xticks(range(0,len(freq),10),freq[::10])
pl.yticks(range(0,len(rssi[1]),2),rssi_val[0::2])
pl.show()






