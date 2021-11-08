import pybullet as p
import time
import pybullet_data

import numpy as np
import matplotlib.pyplot as plt

t = 0
dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
#planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

time_list = []
position_list = []

#while True:
#    p.stepSimulation()
#    time.sleep(dt)

while t < 5:
    p.stepSimulation()
    time.sleep(dt)

    time_list.append(t)
    t+=dt

    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    position_list.append(jointPosition)


plt.grid(True)
plt.xlabel('t', fontsize=12)
plt.ylabel('x(t)', fontsize=12)
plt.plot(time_list, position_list)
plt.show()

p.disconnect()