import pybullet as p
import time
import pybullet_data

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

# Imprime el numero de cada link
numjoint = p.getNumJoints(robotId)
for j in range (numjoint):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))
    print("Link - %s" % (p.getJointInfo(robotId,j)[12]))

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

seg1_pos_id = p.addUserDebugParameter("seg1_pos", -3, 3, 0)
seg2_pos_id = p.addUserDebugParameter("seg2_pos", -0.7, 4, 0)
seg3_pos_id = p.addUserDebugParameter("seg3_pos", -2.5, 0, 0)
seg4_pos_id = p.addUserDebugParameter("seg4_pos", -3.14, 3.14, 0)

posX_id = p.addUserDebugParameter("posX", -10, 10, 0)
posY_id = p.addUserDebugParameter("posY", -10, 10, 0)
posZ_id = p.addUserDebugParameter("posZ", 0, 5, 0)

# Límites articulares 
lowerLimits = [-3.0, -0.7, -2.5, 0.0]  # Limite inferior
upperLimits = [3.0, 4.0, 0.0, 0.0]     # Limite superior 
jointRanges = [6.0, 4.7, 2.5, 6.28]    # Rango de cada articulación
restPoses = [0.0, 0.0, 0.0, 0.0]       # Posiciones de descanso
jointDamping = [0.01] * 12             # Factores de amortiguación

# Mover a cada punto
for target in path_aprox_cube:
    while True:
        seg1_pos = p.readUserDebugParameter(seg1_pos_id)
        seg2_pos = p.readUserDebugParameter(seg2_pos_id)
        seg3_pos = p.readUserDebugParameter(seg3_pos_id)
        seg4_pos = p.readUserDebugParameter(seg4_pos_id)

        posX = p.readUserDebugParameter(posX_id)
        posY = p.readUserDebugParameter(posY_id)
        posZ = p.readUserDebugParameter(posZ_id)

        # Con targets
        joint_inv = p.calculateInverseKinematics2(robotId, [1, 2, 3, 4],
                    [target] * 4,
                    lowerLimits=lowerLimits,
                    upperLimits=upperLimits,
                    jointRanges=jointRanges, 
                    restPoses=restPoses,
                    jointDamping=jointDamping
                    )
        
        # Con sliders
        # joint_inv = p.calculateInverseKinematics2(robotId, [1, 2, 3, 4],
        #             [[posX, posY, posZ]] * 4,
        #             lowerLimits=lowerLimits,
        #             upperLimits=upperLimits,
        #             jointRanges=jointRanges, 
        #             restPoses=restPoses,
        #             jointDamping=jointDamping
        #             )

        print(joint_inv[3])

        # Imprime esta actual de joints
        joint_act = p.getJointStates(robotId, arm_links)

        # Posicion actual de las articulaciones y valores dados por cinemataca inversa
        # for i, state in zip(arm_links, joint_act):
            # print(f"Joint {i}: Posición = {state[0]}, Inverso = {joint_inv[i]}")
        # print("-----------------------------------------------------------")
        # print(joint_act)

        p.setJointMotorControlArray(robotId, 
                                    arm_links, 
                                    p.POSITION_CONTROL, 
                                    targetPositions=[joint_inv[1], joint_inv[2], joint_inv[3], joint_inv[4]],
                                    forces=[90] * 4,
                                    positionGains=[0.05] * 4, # kp
                                    velocityGains=[2] * 4)   # kd

        # p.setJointMotorControlArray(robotId, 
        #                             arm_links, 
        #                             p.POSITION_CONTROL, 
        #                             targetPositions=[seg1_pos, seg2_pos, seg3_pos, seg4_pos],
        #                             forces=[90] * 4,
        #                             positionGains=[0.1] * 4, # kp
        #                             velocityGains=[2] * 4)   # kd
        

        link_index = 4  # Punta brazo
        link_state = p.getLinkState(robotId, link_index)

        # Extraer la posición del joint
        cartesian_position = link_state[0]  # (x, y, z)

        print(f"Posición cartesiana: {cartesian_position}")
        print("--------------------------------------------------------------")


        # Comprobar si se alcanzó la posición
        if abs(cartesian_position[0] - tuple(target)[0]) < 0.2 and abs(cartesian_position[1] - tuple(target)[1]) < 0.2 and abs(cartesian_position[2] - tuple(target)[2]) < 0.2:
            print("Se ha llegado a un punto")
            break
        else:
            # Imprimir diferencia entre posicion actual y punto objetivo
            # print("X: ", abs(cartesian_position[0] - tuple(target)[0]))
            # print("Y: ", abs(cartesian_position[1] - tuple(target)[1]))
            # print("Z: ", abs(cartesian_position[2] - tuple(target)[2]))

            # Con sliders
            print("X: ", abs(cartesian_position[0] - posX))
            print("Y: ", abs(cartesian_position[1] - posY))
            print("Z: ", abs(cartesian_position[2] - posZ))
            print("--------------------------------------------------------------")
        
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
                            positionGains=[0.001] * 4, # kp
                            velocityGains=[0.1] * 4)   # kd
    else:
        p.setJointMotorControlArray(robotId, 
                    arm_links, 
                    p.POSITION_CONTROL, 
                    targetPositions=[-2.4, 0.1, -2.3, 0],
                    forces=[5000] * 4,
                    positionGains=[0.001] * 4, # kp
                    velocityGains=[0.1] * 4)   # kd
    
    link_index = 4  # Punta brazo
    link_state = p.getLinkState(robotId, link_index)

    # Extraer la posición del joint
    cartesian_position = link_state[0]  # (x, y, z)

    if abs(cartesian_position[0] - tuple(obj)[0]) < 0.2 and abs(cartesian_position[1] - tuple(obj)[1]) < 0.2 and abs(cartesian_position[2] - tuple(obj)[2]) < 0.2:
        if obj == path_get_cube[1]:
            break
        obj = path_get_cube[1]
    
    p.stepSimulation()
    time.sleep(0.005)

# Coger cubo
p.setJointMotorControlArray(robotId, 
            [5, 6],  # Finger links
            p.POSITION_CONTROL, 
            targetPositions=[0.3, 0.3],
            forces=[50] * 2,
            positionGains=[0.001] * 2, # kp
            velocityGains=[0.1] * 2)   # kd

# Moverse a cajon


#Dejar cubo


while True:
    p.stepSimulation()
    time.sleep(0.005)




