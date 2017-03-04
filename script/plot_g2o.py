#!/usr/bin/env python
# coding=utf-8
# created by steve 17-2-15 下午1:51

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

from array import array

if __name__ == '__main__':
    g2o_file = open("../data1.g2o")

    odo_list = array("d")
    markers_list = array("d")

    for line in g2o_file:
        if 'VERTEX_SE3' in line:
            id = line.split(' ')[1]
            id = float(id)
            if 10000 > id > 1000:
                odo_list.append(float(line.split(' ')[2]))
                odo_list.append(float(line.split(' ')[3]))
                odo_list.append(float(line.split(' ')[4]))
            if id > 0 and id < 1000:
                markers_list.append(float(line.split(' ')[2]))
                markers_list.append(float(line.split(' ')[3]))
                markers_list.append(float(line.split(' ')[4]))

    odo = np.frombuffer(odo_list, dtype=np.float).reshape(-1, 3)
    marker = np.frombuffer(markers_list, dtype=np.float).reshape(-1, 3)


    after_pf = np.loadtxt("../log.txt")

    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.plot(odo[:,0],odo[:,1],odo[:,2],'r-+')
    ax.plot(marker[:,0],marker[:,1],marker[:,2],'b*')
    ax.plot(after_pf[:,0],after_pf[:,1],after_pf[:,2],'y-+')



    plt.figure(3)
    plt.plot(odo[:, 0], odo[:, 1], 'r*-')
    plt.plot(marker[:, 0], marker[:, 1], 'b+')
    # plt.plot(after_pf[:,0],after_pf[:,1],'y+-')
    plt.grid(True)
    plt.show()
