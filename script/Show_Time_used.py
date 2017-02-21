import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    tlog = np.loadtxt("../time_use_log.txt")

    plt.figure(1)
    plt.grid(True)

    for i in range(tlog.shape[1]):
        plt.plot(tlog[:,i])

    plt.legend()


    plt.show()