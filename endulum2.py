import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import time

t = 0
dt = 1/240  # simulation step
q0 = 0   # starting position
p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

position_list = []
time_list = []

jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
q0 = jointPosition  # exact starting position
#position_list.insert(0, q0)

q1_desirable = 5
K_p = 0.9

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

while t < 5:
    q1_current, *_ = p.getJointState(boxId, jointIndex=1)
    position_list.append(q1_current)
    error = q1_desirable - q1_current
    torque = K_p * error
    p.setJointMotorControl2(bodyIndex=boxId,
                            jointIndex=1,
                            targetVelocity=0,
                            controlMode=p.TORQUE_CONTROL,
                            force=torque)


    p.stepSimulation()
    time_list.append(t)
    t += dt

print(position_list)
print(time_list, '\n')


plt.figure()
plt.grid(True)
plt.xlabel('t', fontsize=12)
plt.ylabel('x(t)', fontsize=12)
plt.plot(time_list, position_list, color='c', label="При K_p = {} для желаемого угла q = {}".format(K_p, q1_desirable))
plt.legend(loc='best')
plt.show()

p.disconnect()
