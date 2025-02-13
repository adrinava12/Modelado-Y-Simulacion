import pybullet as p
import pybullet_data
import argparse
import time

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
huskyId = p.loadURDF("husky/husky.urdf")

metaId = p.loadURDF("meta.urdf")
barreraId = p.loadURDF("barrera.urdf")
rampaID = p.loadURDF("rampa.urdf")

posicion_meta = [20, 0, 1]
orientacion_meta = p.getQuaternionFromEuler([0,0,1.57])

posicion_barrera = [17, 0, 1]
orientacion_barrera = p.getQuaternionFromEuler([0,0,1.57])

posicion_rampa = [10, 0, 1]
orientacion_rampa = p.getQuaternionFromEuler([0,0,1.57])

p.resetBasePositionAndOrientation(metaId, posicion_meta, orientacion_meta)
p.resetBasePositionAndOrientation(barreraId, posicion_barrera, orientacion_barrera)
p.resetBasePositionAndOrientation(rampaID, posicion_rampa, orientacion_rampa)

p.setRealTimeSimulation(1)

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
            
except KeyboardInterrupt:
      pass
	
p.disconnect()
