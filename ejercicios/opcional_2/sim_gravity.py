import pybullet as p
import pybullet_data
import argparse
import time

parser = argparse.ArgumentParser(description="URDF viewer example")
parser.add_argument("--urdf", type=str, required=True, help="Ruta al archivo URDF.")
args = parser.parse_args()
urdf_path = args.urdf

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,0)

planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

esphereId = p.loadURDF(urdf_path,startPos, startOrientation)

delta = 1./240. # Salto de tiempo entre cada calculo
p.setTimeStep(delta)

#Parametros para las formulas MRUA
g = -9.81 # Gravedad
v_0 = 0 # Velocidad inicial
y_0 = 3 # Posicion inicial

t = 0 # Tiempo actual

try:
    while True:
        # Implementacion formulas MRUA
        y = y_0 + v_0 *t + 0.5 * g * t**2

        if y > 0.5:
            # Recolocacion de la esfera en nueva posicion
            p.resetBasePositionAndOrientation(esphereId, [0, 0, y], [0, 0, 0, 1])
        else:
            # Mantenerse en el suelo una vez a caido
            p.resetBasePositionAndOrientation(esphereId, [0, 0, 0.5], [0, 0, 0, 1])

        t += delta

        p.stepSimulation()
        time.sleep(0.2)

except KeyboardInterrupt:
      pass
	
p.disconnect()  