#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-f", "--file", dest="filename", help="file to plot")
args = parser.parse_args()

print("plotting {}".format(args.filename))
filename = args.filename

data = np.loadtxt(filename)
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,1],data[:,2],data[:,3],'.-')
plt.show()

print("Exiting!")