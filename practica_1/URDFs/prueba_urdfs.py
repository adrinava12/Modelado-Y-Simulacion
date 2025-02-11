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
huskyId = p.loadURDF("husky/husky.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,1.57])

robotId = p.loadURDF(urdf_path,startPos, startOrientation)

try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
            
except KeyboardInterrupt:
      pass
	
p.disconnect()