# -*- coding: utf-8 -*-
"""
Created on Sat Oct 26 19:49:18 2019

@author: Muhammad
"""

import matplotlib.pyplot as plt
import csv

ind = []
e1 = []
e2 = []
e3 = []
e4 = []
e5 = []
e6 = []

with open('error.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    
    for row in plots:
#        row_instances = row.split(",")
#        print(row_instances)
        ind.append(float(row[0]))
        e1.append(float(row[1]))
        e2.append(float(row[2]))
        e3.append(float(row[3]))
        e4.append(float(row[4]))
        e5.append(float(row[5]))
        e6.append(float(row[6]))

plt.plot(ind,e1, ind, e2, ind, e3, ind, e4,ind,  e5,ind,  e6)
#plt.plot(ind,  e2)
plt.xlabel('Time (sec)')
plt.ylabel('Error')
plt.title('Oscillatory Case')
#plt.legend()
plt.show()

