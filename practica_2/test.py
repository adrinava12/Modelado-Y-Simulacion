import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI) # connect motor with gui
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0,0, -9.8)

planeId = p.loadURDF("plane.urdf") #load model

startPos = [0,0,1]
euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 3.14])

robotId = p.loadURDF("rover_scara/urdf/rover_scara.urdf",startPos, startOrientation)

# Creacion de cubo
cube_size = 0.5 / 2

collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[cube_size, cube_size, cube_size]
)

visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[cube_size, cube_size, cube_size],
    rgbaColor=[1, 0, 0, 1]  # Color rojo
)

# Posición inicial del cubo
start_position = [0, 4, 1]  # x, y, z
euler_angles = [0, 0, 0.8]
quaternion = p.getQuaternionFromEuler(euler_angles)

# Crear el cubo como un cuerpo rígido
cube_id = p.createMultiBody(
    baseMass=1,  # Masa del cubo
    baseCollisionShapeIndex=collision_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=start_position,
    baseOrientation=quaternion
)

wheels = [7, 8, 9, 10, 11, 12]
arm_links = [1, 2, 3, 4]

while True:
    position = p.getBasePositionAndOrientation(robotId)

    # Move forward until reaching point [0,3,0]
    if position[0][1] <= 0.5:
        p.setJointMotorControlArray(robotId,
                    wheels,
                    p.VELOCITY_CONTROL,
                    targetVelocities=[4] * 6)
        
        p.stepSimulation()
        time.sleep(0.005)

    # Point [0, 3, 0] has been reach
    else:
        break

# Detener tras llegar a [0, 3, 0]
p.setJointMotorControlArray(robotId,
                wheels,
                p.VELOCITY_CONTROL,
                targetVelocities=[0] * 6)

path_aprox_cube = [
    [-1.4, 1.7, 2.9], # Este punto ya esta bien
    [-1.3, 1.9, 2.9]
]

path_get_cube = [
    [0.0, 3.9, 1.3],
    [0.0, 3.9, 0.5]
]

path_drop_cube = [
    [0, -0.3, 2.9],
    [0, -0.3, 2.4]
]

data_csv = [["Tiempo", "NúmeroJoints", "G_parcial"]]
tiempo = 0
step_counter = 0

# Límites articulares 
lowerLimits = [-3.0, -0.7, -2.5, 0.0]  # Limite inferior
upperLimits = [3.0, 4.0, 0.0, 0.0]     # Limite superior 
jointRanges = [6.0, 4.7, 2.5, 6.28]    # Rango de cada articulación
restPoses = [0.0, 0.0, 0.0, 0.0]       # Posiciones de descanso
jointDamping = [0.01] * 12             # Factores de amortiguación

# Mover a cada punto
for target in path_aprox_cube:
    while True:

        joint_inv = p.calculateInverseKinematics2(robotId, [1, 2, 3, 4],
                    [target] * 4,
                    lowerLimits=lowerLimits,
                    upperLimits=upperLimits,
                    jointRanges=jointRanges, 
                    restPoses=restPoses,
                    jointDamping=jointDamping
                    )

        p.setJointMotorControlArray(robotId, 
                                    arm_links, 
                                    p.POSITION_CONTROL, 
                                    targetPositions=[joint_inv[1], joint_inv[2], joint_inv[3], joint_inv[4]],
                                    forces=[90] * 4,
                                    positionGains=[0.05] * 4,
                                    velocityGains=[2] * 4)

        link_index = 4  # Punta brazo
        link_state = p.getLinkState(robotId, link_index)

        # Extraer la posición del joint
        cartesian_position = link_state[0]  # (x, y, z)


        # Comprobar si se alcanzó la posición
        if abs(cartesian_position[0] - tuple(target)[0]) < 0.2 and abs(cartesian_position[1] - tuple(target)[1]) < 0.2 and abs(cartesian_position[2] - tuple(target)[2]) < 0.2:
            break

        # Calculo de G_parcial cada 0.01 segundos
        if step_counter % 2 == 0:
            joint_states = p.getJointStates(robotId, arm_links)  # arm_links = [1,2,3,4]
            joint_forces = [abs(js[3]) for js in joint_states] 
            G_parcial = sum(joint_forces)
            data_csv.append([tiempo, 4, G_parcial])
        
        tiempo += 0.005
        step_counter += 1
        p.stepSimulation()
        time.sleep(0.005)

obj = path_get_cube[0]
while True:
    if obj == path_get_cube[0]:
        p.setJointMotorControlArray(robotId, 
                            arm_links, 
                            p.POSITION_CONTROL, 
                            targetPositions=[-2.4, 0.1, -1.5, 0],
                            forces=[5000] * 4,
                            positionGains=[0.001] * 4,
                            velocityGains=[0.1] * 4)
    else:
        p.setJointMotorControlArray(robotId, 
                    arm_links, 
                    p.POSITION_CONTROL, 
                    targetPositions=[-2.4, 0.1, -2.3, 0],
                    forces=[5000] * 4,
                    positionGains=[0.001] * 4,
                    velocityGains=[0.1] * 4)
    
    link_index = 4  # Punta brazo
    link_state = p.getLinkState(robotId, link_index)

    # Extraer la posición del joint
    cartesian_position = link_state[0]  # (x, y, z)

    if abs(cartesian_position[0] - tuple(obj)[0]) < 0.2 and abs(cartesian_position[1] - tuple(obj)[1]) < 0.2 and abs(cartesian_position[2] - tuple(obj)[2]) < 0.2:
        if obj == path_get_cube[1]:
            break
        obj = path_get_cube[1]
    
    # Calculo de G_parcial cada 0.01 segundos
    if step_counter % 2 == 0:
        joint_states = p.getJointStates(robotId, arm_links)  # arm_links = [1,2,3,4]
        joint_forces = [abs(js[3]) for js in joint_states] 
        G_parcial = sum(joint_forces)
        data_csv.append([tiempo, 4, G_parcial])
    
    tiempo += 0.005
    step_counter += 1
    p.stepSimulation()
    time.sleep(0.005)

# Coger cubo
p.setJointMotorControlArray(robotId, 
            [5, 6],  # Finger links
            p.POSITION_CONTROL, 
            targetPositions=[0.3, 0.3],
            forces=[50] * 2,
            positionGains=[0.001] * 2,
            velocityGains=[0.1] * 2)  

# Moverse a cajon
for target in path_aprox_cube:
    while True:

        joint_inv = p.calculateInverseKinematics2(robotId, [1, 2, 3, 4],
                    [target] * 4,
                    lowerLimits=lowerLimits,
                    upperLimits=upperLimits,
                    jointRanges=jointRanges, 
                    restPoses=restPoses,
                    jointDamping=jointDamping
                    )

        p.setJointMotorControlArray(robotId, 
                                    arm_links, 
                                    p.POSITION_CONTROL, 
                                    targetPositions=[joint_inv[1], joint_inv[2], joint_inv[3], joint_inv[4]],
                                    forces=[90] * 4,
                                    positionGains=[0.01] * 4,
                                    velocityGains=[4] * 4)

        link_index = 4  # Punta brazo
        link_state = p.getLinkState(robotId, link_index)

        # Extraer la posición del joint
        cartesian_position = link_state[0]  # (x, y, z)

        # Comprobar si se alcanzó la posición
        if abs(cartesian_position[0] - tuple(target)[0]) < 0.2 and abs(cartesian_position[1] - tuple(target)[1]) < 0.2 and abs(cartesian_position[2] - tuple(target)[2]) < 0.2:
            break

        # Calculo de G_parcial cada 0.01 segundos
        if step_counter % 2 == 0:
            joint_states = p.getJointStates(robotId, arm_links)  # arm_links = [1,2,3,4]
            joint_forces = [abs(js[3]) for js in joint_states] 
            G_parcial = sum(joint_forces)
            data_csv.append([tiempo, 4, G_parcial])
        
        tiempo += 0.005
        step_counter += 1
        p.stepSimulation()
        time.sleep(0.005)


#Dejar cubo
obj = path_drop_cube[0]
while True:
    if obj == path_drop_cube[0]:
        p.setJointMotorControlArray(robotId, 
                            arm_links, 
                            p.POSITION_CONTROL, 
                            targetPositions=[1.1, -0.4, 0, 0],
                            forces=[120] * 4,
                            positionGains=[0.001] * 4,
                            velocityGains=[0.1] * 4)
    else:
        p.setJointMotorControlArray(robotId, 
                    arm_links, 
                    p.POSITION_CONTROL, 
                    targetPositions=[1.1, -0.4, -0.8, 0],
                    forces=[120] * 4,
                    positionGains=[0.01] * 4,
                    velocityGains=[4] * 4)
    
    link_index = 4  # Punta brazo
    link_state = p.getLinkState(robotId, link_index)

    # Extraer la posición del joint
    cartesian_position = link_state[0]  # (x, y, z)

    if abs(cartesian_position[0] - tuple(obj)[0]) < 0.2 and abs(cartesian_position[1] - tuple(obj)[1]) < 0.2 and abs(cartesian_position[2] - tuple(obj)[2]) < 0.2:
        if obj == path_drop_cube[1]:
            break
        obj = path_drop_cube[1]

    # Calculo de G_parcial cada 0.01 segundos
    if step_counter % 2 == 0:
        joint_states = p.getJointStates(robotId, arm_links)  # arm_links = [1,2,3,4]
        joint_forces = [abs(js[3]) for js in joint_states] 
        G_parcial = sum(joint_forces)
        data_csv.append([tiempo, 4, G_parcial])
    
    tiempo += 0.005
    step_counter += 1
    p.stepSimulation()
    time.sleep(0.005)

# Soltar cubo
p.setJointMotorControlArray(robotId, 
            [5, 6],  # Finger links
            p.POSITION_CONTROL, 
            targetPositions=[0.0, 0.0],
            forces=[50] * 2,
            positionGains=[0.001] * 2, # kp
            velocityGains=[0.1] * 2)   # kd

# Volver home
for i in range (500):
    p.setJointMotorControlArray(robotId, 
                    arm_links, 
                    p.POSITION_CONTROL, 
                    targetPositions=[1.1, -0.4, 0, 0],
                    forces=[120] * 4,
                    positionGains=[0.01] * 4,
                    velocityGains=[4] * 4)

    # Calculo de G_parcial cada 0.01 segundos
    if step_counter % 2 == 0:
        joint_states = p.getJointStates(robotId, arm_links)  # arm_links = [1,2,3,4]
        joint_forces = [abs(js[3]) for js in joint_states] 
        G_parcial = sum(joint_forces)
        data_csv.append([tiempo, 4, G_parcial])
    
    tiempo += 0.005
    step_counter += 1
    p.stepSimulation()
    time.sleep(0.005)


# Guardar datos en CSV
header = data_csv[0]
numeric_data = np.array(data_csv[1:], dtype=float)

G_total = sum(row[2] for row in data_csv[1:])  # Excluye la cabecera
print(f"G_total del recorrido completo: {G_total:.6f}")

# Guardar en CSV con cabecera
np.savetxt("csv/g_parcial.csv", numeric_data, delimiter=",", header=",".join(header), comments="", fmt="%.6f")
	
p.disconnect()