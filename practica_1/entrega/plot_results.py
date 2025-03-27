import matplotlib.pyplot as plt
import numpy as np

fase3_1 = np.genfromtxt("csv/fase3-1.csv", delimiter=",")
fase3_2 = np.genfromtxt("csv/fase3-2.csv", delimiter=",")
fase3_3 = np.genfromtxt("csv/fase3.csv", delimiter=",")
fase4 = np.genfromtxt("csv/fase4.csv", delimiter=",")

position_3_1 = [x[1] for x in fase3_1]
velocity_3_1 = [x[2] for x in fase3_1]

position_3_2 = [x[1] for x in fase3_2]
velocity_3_2 = [x[2] for x in fase3_2]

position_3_3 = [x[1] for x in fase3_3]
velocity_3_3 = [x[2] for x in fase3_3]

position_4 = [x[1] for x in fase4]
velocity_4 = [x[2] for x in fase4]

# Plot 3-1, 3-2, 3-3
plt.figure()
plt.plot(position_3_1, velocity_3_1)
plt.plot(position_3_2, velocity_3_2, linestyle='--')
plt.plot(position_3_3, velocity_3_3, linestyle='--')

plt.grid(True)
plt.legend(["Escenario 3.1", "Escenario 3.2", "Escenario 3.3"])
plt.title("Relacion entre velocidad y posicion")
plt.xlabel("Posicion husky (m)")
plt.ylabel("Velocidad husky (m/s)")

plt.savefig("Fase3.pdf", format="pdf", bbox_inches="tight")

plt.show(block=False)


# Plot 4
plt.figure()
plt.plot(position_4, velocity_4)

plt.grid(True)
plt.legend(["Escenario 4"])
plt.title("Relacion entre velocidad y posicion")
plt.xlabel("Posicion husky (m)")
plt.ylabel("Velocidad husky (m/s)")

plt.savefig("Fase4.pdf", format="pdf", bbox_inches="tight")

plt.show(block=False)


# Plot 3-3, 4
plt.figure()
plt.plot(position_3_3, velocity_3_3)
plt.plot(position_4, velocity_4)

plt.grid(True)
plt.legend(["Escenario 3.3", "Escenario 4"])
plt.title("Relacion entre velocidad y posicion")
plt.xlabel("Posicion husky (m)")
plt.ylabel("Velocidad husky (m/s)")

plt.savefig("Fase3.3-4.pdf", format="pdf", bbox_inches="tight")

plt.show()
