import matplotlib.pyplot as plt
import numpy as np

fase4 = np.genfromtxt("csv/fase4.csv", delimiter=",")
fase3_3 = np.genfromtxt("csv/fase3-3.csv", delimiter=",")

position_1 = [x[1] for x in fase4]
velocity_1 = [x[2] for x in fase4]

position_2 = [x[1] for x in fase3_3]
velocity_2 = [x[2] for x in fase3_3]

plt.plot(position_1, velocity_1)
plt.plot(position_2, velocity_2)

plt.grid(True)
plt.legend(["Escenario 4", "Escenario 3.3"])
plt.title("Relacion entre velocidad y posicion")
plt.xlabel("Posicion husky")
plt.ylabel("Velocidad husky")

plt.savefig("Fase4.pdf", format="pdf", bbox_inches="tight")

plt.show()
