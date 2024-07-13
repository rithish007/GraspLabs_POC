import pybullet as p
import time
import pybullet_data

# Connect to PyBullet physics server
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally

# Set gravity
p.setGravity(0, 0, -10)

# Load the plane URDF
planeId = p.loadURDF("plane.urdf")

# Load your delta robot URDF
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("delta_robot.urdf", startPos, startOrientation)

# Set the center of mass frame (loadURDF sets base link frame)
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)

# Simulation loop
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

# Get the position and orientation of the robot
robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
print(robotPos, robotOrn)

# Disconnect from the physics server
p.disconnect()