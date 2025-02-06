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
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane_transparent.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,-3.15])

robotId = p.loadURDF(urdf_path,startPos, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))


# Leer friccion y torque del usuario
top_rot_id = p.addUserDebugParameter("top_link_torque", -20, 20, 0)
frictionId = p.addUserDebugParameter("jointFriction", 0, 100, 10)

try:
    while True:
        frictionForce = p.readUserDebugParameter(frictionId)
        join_torque = p.readUserDebugParameter(top_rot_id)

        p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
        p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=join_torque)
        p.stepSimulation()
        time.sleep(1./240.)
            
except KeyboardInterrupt:
      pass
	
p.disconnect()