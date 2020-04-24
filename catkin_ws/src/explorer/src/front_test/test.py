import numpy as np
import matplotlib.pyplot as plt


plt.figure(1)

for i in range(18):
    data = np.loadtxt("front{}.txt".format(i))
    if i < 10:
        plt.scatter(data[:, 0], data[:, 1], marker='x', label="{}_{}".format(i, len(data)))
    else:
        plt.scatter(data[:, 0], data[:, 1], label="{}_{}".format(i, len(data)))
    # point = np.array([np.mean(data[:, 0]), np.mean(data[:, 1])])
    # point1 = np.array([np.mean(data[:, 0]), np.sum(data[:, 1])/len(data)])
    # points = point.vstack((point, ))
    plt.scatter(np.mean(data[:, 0]), np.mean(data[:, 1]), marker='o')
plt.legend()
plt.show()