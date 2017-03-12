#!/usr/bin/env python
# coding=utf-8
# created by steve 17-2-15 下午1:51

import numpy as np
import scipy as sp
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

from array import array

if __name__ == '__main__':
    g2o_file = open("../Save1.g2o")

    odo_list = array("d")
    odo_id = array("d")
    markers_list = array("d")

    tag_array = np.zeros([4,3])

    for line in g2o_file:
        if 'VERTEX_SE3' in line:
            id = line.split(' ')[1]
            id = float(id)
            if 10000000 > id > 1000:
                odo_list.append(float(line.split(' ')[2]))
                odo_list.append(float(line.split(' ')[3]))
                odo_list.append(float(line.split(' ')[4]))
                odo_id.append(id)
            if id > 0 and id < 1000:
                markers_list.append(float(line.split(' ')[2]))
                markers_list.append(float(line.split(' ')[3]))
                markers_list.append(float(line.split(' ')[4]))
                if (id-12) < 3:
                    tag_array[int(id-12),0] = float(line.split(' ')[2])
                    tag_array[int(id-12),1] = float(line.split(' ')[3])
                    tag_array[int(id-12),2] = float(line.split(' ')[4])


    odo = np.frombuffer(odo_list, dtype=np.float).reshape(-1, 3)
    odo_id = np.frombuffer(odo_id,dtype=np.float).reshape(-1)
    marker = np.frombuffer(markers_list, dtype=np.float).reshape(-1, 3)


    # after_pf = np.loadtxt("../log.txt")

    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.plot(odo[:,0],odo[:,1],odo[:,2],'r-+')
    ax.plot(marker[:,0],marker[:,1],marker[:,2],'b*')
    ax.plot(tag_array[:,0],tag_array[:,1],tag_array[:,2],'D')
    # ax.plot(after_pf[:,0],after_pf[:,1],after_pf[:,2],'y-+')

    # Save Beaconset
    tag_array[:,2] = 1.12
    np.savetxt("beaconset",tag_array)


    # Generator pose dictionary
    time_log = open('../id2timelog.txt')

    # time_pose = open('../time2pose.txt')
    time_pose = np.zeros([odo.shape[0],4])
    tp_index = 0

    for line in time_log.readlines():
        the_time = float(line.split(' ')[0])
        the_id = float(line.split(' ')[1])

        time_pose[tp_index,0] = the_time
        time_pose[tp_index,1:] = odo[np.argmin(np.abs(odo_id-the_id)),:]

        tp_index += 1


    np.savetxt("time2pose.csv",time_pose,delimiter=',')





















    plt.figure(3)
    # plt.plot(odo[:, 0], odo[:, 1], 'r*-')
    plt.plot(marker[:, 0], marker[:, 1], 'b+')
    plt.plot(tag_array[:,0],tag_array[:,1],'D')
    # plt.plot(after_pf[:,0],after_pf[:,1],'y+-')
    plt.grid(True)
    plt.show()
