import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt("ex1.csv", delimiter=",")

position = [x[1] for x in data]
velocity = [x[2] for x in data]

plt.plot(position, velocity)


plt.show()