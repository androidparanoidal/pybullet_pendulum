import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)


t = 0
dt = 1 / 240  # simulation step
q0 = 0        # starting position
q1_desirable = 1
K_p = [0.5, 2, 4, 20]
n = 4
# position_list = np.zeros((n, 1200))
position_list = []
time_list = []
int_error = 0

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
q0 = jointPosition  # exact starting position

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

while t <= 10:
    q1_current, *_ = p.getJointState(boxId, jointIndex=1)
    position_list.append(q1_current)
    error = q1_desirable - q1_current
    int_error += error * dt
    torque = K_p[0] * error + K_p[0] * int_error
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=torque)
    p.stepSimulation()
    time_list.append(t)
    t += dt
    #time.sleep(dt)

''' 2 '''
t = 0
int_error = 0

# go to the starting position 2
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
q0 = jointPosition  # exact starting position

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
position_list2 = []
time_list2 = []

while t <= 10:
    q1_current2, *_ = p.getJointState(boxId, jointIndex=1)
    position_list2.append(q1_current2)
    error = q1_desirable - q1_current2
    int_error += error * dt
    torque = K_p[1] * error + K_p[1] * int_error
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=torque)
    p.stepSimulation()
    time_list2.append(t)
    t += dt
    #time.sleep(dt)


''' 3 '''
t = 0
int_error = 0

# go to the starting position 3
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
q0 = jointPosition  # exact starting position

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
position_list3 = []
time_list3 = []

while t <= 10:
    q1_current3, *_ = p.getJointState(boxId, jointIndex=1)
    position_list3.append(q1_current3)
    error = q1_desirable - q1_current3
    int_error += error * dt
    torque = K_p[2] * error + K_p[2] * int_error
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=torque)
    p.stepSimulation()
    time_list3.append(t)
    t += dt
    #time.sleep(dt)


''' 4 '''
t = 0
int_error = 0

# go to the starting position 3
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
q0 = jointPosition  # exact starting position

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
position_list4 = []
time_list4 = []

while t <= 10:
    q1_current4, *_ = p.getJointState(boxId, jointIndex=1)
    position_list4.append(q1_current4)
    error = q1_desirable - q1_current4
    int_error += error * dt
    torque = K_p[3] * error + K_p[3] * int_error
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=torque)
    p.stepSimulation()
    time_list4.append(t)
    t += dt
    #time.sleep(dt)


print(position_list)
#print(np.size(position_list))
print(time_list, '\n')

print(position_list2)
print(time_list2, '\n')

t_q = np.linspace(0, 10)
q_d = np.full(50, 1)

plt.figure()
plt.grid(True)
plt.title('Решения для желаемого q = {}:'.format(q1_desirable))
plt.xlabel('t', fontsize=12)
plt.ylabel('x(t)', fontsize=12)
plt.plot(time_list, position_list, color='c', label="При K_p и К_i = 0.5")
plt.plot(time_list, position_list2, color='r', label="При K_p и К_i = 2")
plt.plot(time_list, position_list3, color='m', label="При K_p и К_i = 4")
plt.plot(time_list, position_list4, color='g', label="При K_p и К_i = 20")
plt.plot(t_q, q_d, color='k', label="Желаемое значение q", linewidth=2)
plt.legend(loc='best')
plt.show()

p.disconnect()
