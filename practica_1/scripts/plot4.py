import matplotlib.pyplot as plt
import numpy as np

fase3_1 = np.genfromtxt("csv/fase4.csv", delimiter=",")

position_1 = [x[1] for x in fase3_1]
velocity_1 = [x[2] for x in fase3_1]

plt.plot(position_1, velocity_1)


plt.legend(["Escenario 4"])
plt.title("Relacion entre velocidad y posicion")
plt.xlabel("Posicion husky")
plt.ylabel("Velocidad husky")

plt.savefig("Fase4.pdf", format="pdf", bbox_inches="tight")

plt.show()
