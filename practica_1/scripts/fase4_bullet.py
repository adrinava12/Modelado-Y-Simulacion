import pybullet as p
import pybullet_data
import numpy as np
import time

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
huskyId = p.loadURDF("husky/husky.urdf")

metaId = p.loadURDF("URDFs/meta.urdf", useFixedBase=True)
barreraId = p.loadURDF("URDFs/barrera.urdf", useFixedBase=True)
rampaID = p.loadURDF("URDFs/rampa.urdf", useFixedBase=True)

posicion_husky = [0, 0, 1]
orientacion_husky = p.getQuaternionFromEuler([0, 0, 1.57])

posicion_meta = [0, 20, 0]
orientacion_meta = p.getQuaternionFromEuler([0,0,0])

posicion_barrera = [-1.5, 17, 0]
orientacion_barrera = p.getQuaternionFromEuler([0, 0,0])

posicion_rampa = [0, 10, 0]
orientacion_rampa = p.getQuaternionFromEuler([0,0,0])

p.resetBasePositionAndOrientation(huskyId, posicion_husky, orientacion_husky)
p.resetBasePositionAndOrientation(metaId, posicion_meta, orientacion_meta)
p.resetBasePositionAndOrientation(barreraId, posicion_barrera, orientacion_barrera)
p.resetBasePositionAndOrientation(rampaID, posicion_rampa, orientacion_rampa)

p.setRealTimeSimulation(1)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,        # Alejamiento de la cámara
    cameraYaw=90,              # Rotación horizontal
    cameraPitch=-20,           # Rotación vertical
    cameraTargetPosition=[7, 11, 5]  # Punto al que mira la cámara
)

# 2 - front_left_wheel, 3 - front_right_wheel, 4 - rear_left_wheel, 5 - rear_right_wheel
joints = [2, 3, 4, 5]

data = []
start_time = time.time()

#Cambio dinamica barrera
p.changeDynamics(barreraId, 0, localInertiaDiagonal=[10.41, 0, 10.41])

# Cambio de dinamicas de husky
for joint in joints:
    p.changeDynamics(huskyId, joint, lateralFriction=0.93)
    p.changeDynamics(huskyId, joint, spinningFriction=0.005)
    p.changeDynamics(huskyId, joint, rollingFriction=0.003)

# Posicion inicial del husky
prev_husky_pos, _ = p.getBasePositionAndOrientation(huskyId)
try:
    while True:
        p.setJointMotorControlArray(huskyId,
                                joints,
                                p.VELOCITY_CONTROL,
                                targetVelocities=[11,11,11,11])
        
        p.setJointMotorControlArray(huskyId,
                                joints,
                                p.TORQUE_CONTROL,
                                forces=[25,25,25,25])
        
        # Posicion actual del husky
        husky_pos, _ = p.getBasePositionAndOrientation(huskyId)
        if husky_pos[1] - prev_husky_pos[1] >= 0.01:
            # Tomar tiempo, posicion, velocidad, velocidad ruedas, fuerza ruedas
            pos_husky_y = husky_pos[1] # Posicion robot en eje Y

            prev_husky_pos = husky_pos

            # Calculo de tiempo actual
            actual_time = time.time() - start_time
            start_time = time.time()

            # Insertar datos en lista
            output = p.getJointStates(huskyId, joints)
            data.append([actual_time, pos_husky_y, p.getBaseVelocity(huskyId)[0][1]] + [11, 11, 11, 11] + [joint[1] for joint in output])


        # Terminar al tocar a la meta
        contacts = p.getContactPoints(bodyA=huskyId, bodyB=metaId)
        if len(contacts) > 0:
            break
            
except KeyboardInterrupt:
      pass

np_data = np.array(data, dtype=float)

# Guardar en un archivo CSV asegurando formato numérico
np.savetxt("csv/fase3-3.csv", np_data, delimiter=",", fmt="%.6f")
	
p.disconnect()
