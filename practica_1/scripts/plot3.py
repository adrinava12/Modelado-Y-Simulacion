import matplotlib.pyplot as plt
import numpy as np

fase3_1 = np.genfromtxt("csv/fase3-1.csv", delimiter=",")
fase3_2 = np.genfromtxt("csv/fase3-2.csv", delimiter=",")
fase3_3 = np.genfromtxt("csv/fase3-3.csv", delimiter=",")

position_1 = [x[1] for x in fase3_1]
velocity_1 = [x[2] for x in fase3_1]

position_2 = [x[1] for x in fase3_2]
velocity_2 = [x[2] for x in fase3_2]

position_3 = [x[1] for x in fase3_3]
velocity_3 = [x[2] for x in fase3_3]

plt.plot(position_1, velocity_1)
plt.plot(position_2, velocity_2, linestyle='--')
plt.plot(position_3, velocity_3, linestyle='--')

plt.grid(True)
plt.legend(["Escenario 3.1", "Escenario 3.2", "Escenario 3.3"])
plt.title("Relacion entre velocidad y posicion")
plt.xlabel("Posicion husky")
plt.ylabel("Velocidad husky")

plt.savefig("Fase3.pdf", format="pdf", bbox_inches="tight")

plt.show()
