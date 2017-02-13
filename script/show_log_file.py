#!/usr/bin/env python
# coding=utf-8
# created by steve 17-2-13 下午5:03



import numpy as np

import matplotlib.pyplot as plt


if __name__ == '__main__':
    data = np.loadtxt("./log_file.csv")

    plt.figure(1)

    plt.plot(data[:,0],data[:,1],'r-+')
    plt.grid(True)

    plt.show()
