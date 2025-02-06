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

right_vel_id = p.addUserDebugParameter("right_velocities", -50, 50, 0)
left_vel_id = p.addUserDebugParameter("left_velocities", -50, 50, 0)
force_gripper_id = p.addUserDebugParameter("force_gripper", -1, 1, 0)

# 2 - right_front_wheel_joint   3 - right_back_wheel_joint
# 6 - left_front_wheel_joint    7 - left_back_wheel_joint
wheels = [2, 3, 6, 7]

#p.setRealTimeSimulation(1)

try:
    while True:
        right_vel = p.readUserDebugParameter(right_vel_id)
        left_vel = p.readUserDebugParameter(left_vel_id)
        force_gripper = p.readUserDebugParameter(force_gripper_id)

        p.setJointMotorControlArray(robotId, wheels, p.VELOCITY_CONTROL, targetVelocities=[right_vel, right_vel, left_vel, left_vel])
        p.setJointMotorControlArray(robotId, [9, 11], p.VELOCITY_CONTROL, targetVelocities=[force_gripper, force_gripper])
        p.stepSimulation()
        time.sleep(1./240.)
            
except KeyboardInterrupt:
      pass
	
p.disconnect()    
