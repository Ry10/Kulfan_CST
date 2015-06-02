__author__ = 'ryanbarr'

import numpy as np
import matplotlib.pylab as plt
from kulfan_to_coord import CST_shape
from pyOpt import Optimization, SLSQP
import os, sys, time
import pdb

file_name = 'DU21_A17.pfl'
airfoil_coord_file = open(file_name, "r")

# Read the header
header = airfoil_coord_file.readline().replace("\n"," ").replace("\t"," ").split()
header2 = airfoil_coord_file.readline().replace("\n"," ").replace("\t"," ").split()

# Read the body
body = airfoil_coord_file.read().replace("\n"," ").replace("\t"," ").split()
w = 0
points = len(body)/2
x1 = np.zeros(len(body)/2)
y1 = np.zeros(len(body)/2)
airfoil_coord_file.close()
airfoil_coord_file = open(file_name, "r")
header = airfoil_coord_file.readline().replace("\n"," ").replace("\t"," ").split()
header2 = airfoil_coord_file.readline().replace("\n"," ").replace("\t"," ").split()
for i in range(points):
    coordinate_point = airfoil_coord_file.readline().replace("\n"," ").replace("\t"," ").split()
    x1[i] = coordinate_point[0]
    y1[i] = coordinate_point[1]

global airfoil_CST
dz = 0
N = len(body)/2

def objfunc(x):

    wl = [x[0], x[1], x[2], x[3]]
    wu = [x[4], x[5], x[6], x[7]]

    global airfoil_CST
    airfoil_CST = CST_shape(wl, wu, dz, N)
    coordinates = airfoil_CST.inv_airfoil_coor(x1)
    x2 = coordinates[:][0]
    y2 = coordinates[:][1]

    f = 0
    for i in range(N-1):
        f += abs(y1[i]*100 - y2[i]*100)**2

    g = []

    fail = 0
    return f, g, fail


# =============================================================================
#
# =============================================================================
opt_prob = Optimization('CST Parameterization', objfunc)
opt_prob.addVar('x1','c', lower=-2.0,upper=2.0, value=-1.0)
opt_prob.addVar('x2','c', lower=-2.0,upper=2.0, value=-1.0)
opt_prob.addVar('x3','c', lower=-2.0,upper=2.0, value=-1.0)
opt_prob.addVar('x4','c', lower=-2.0,upper=2.0, value=-1.0)
opt_prob.addVar('x5','c', lower=-2.0, upper=2.0, value=1.0)
opt_prob.addVar('x6','c', lower=-2.0, upper=2.0, value=1.0)
opt_prob.addVar('x7','c', lower=-2.0, upper=2.0, value=1.0)
opt_prob.addVar('x8','c', lower=-2.0, upper=2.0, value=1.0)
opt_prob.addObj('f')
print opt_prob

# Instantiate Optimizer (SLSQP) & Solve Problem
slsqp = SLSQP()
slsqp.setOption('IPRINT',-1)
slsqp(opt_prob, sens_type='FD')
print opt_prob.solution(0)

wl_new, wu_new = airfoil_CST.getVar()

def plot():
    if N % 2 == 0:
        z = 0
    else:
        z = 1
    airfoil_CST2 = CST_shape(wl_new, wu_new, dz, N+z)
    coordinates = airfoil_CST2.airfoil_coor()
    x_coor = coordinates[0]
    y_coor = coordinates[1]
    ax = plt.subplot(111)
    ax.plot(x_coor, y_coor, 'g', label='CST')
    ax.plot(x1, y1, 'b', label='original')
    legend = ax.legend(loc='lower center', frameon=False)
    plt.xlabel('x/c')
    plt.ylabel('y/c')
    plt.ylim(ymin=-0.5, ymax=0.5)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.yaxis.set_ticks_position('left')
    ax.xaxis.set_ticks_position('bottom')
    plt.show()

# UNCOMMENT TO PLOT
# plot()

print 'wl = ' + str(wl_new)
print 'wu = ' + str(wu_new)
