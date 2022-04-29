import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import time

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

t = 0
dt = 1/240  # simulation step
q0 = 0        # starting position
q1_desirable = 0.5
kp = 10
ki = 10
kd = 8
position_list = []
time_list = []
int_error = 0
diff_error = 0

def func(t, int_error, diff_error, q1_desirable, q0, kp, ki, kd):
    # go to the starting position
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    q0 = jointPosition  # exact starting position

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

    prev_error = 0.5
    while t <= 10:
        q1_current, *_ = p.getJointState(boxId, jointIndex=1)
        position_list.append(q1_current)

        error = q1_desirable - q1_current
        diff_error = (error - prev_error)/dt
        prev_error = error

        int_error += error * dt
        torque = kp * error + ki * int_error + kd * diff_error
        p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=torque)
        p.stepSimulation()
        time_list.append(t)
        t += dt

    return position_list, time_list

sol = func(t, int_error, diff_error, q1_desirable, q0, kp, ki, kd)

print(position_list)
#print(np.size(position_list))
print(time_list, '\n')

t_q = np.linspace(0, 10)
q_d = np.full(50, q1_desirable)

plt.figure()
plt.grid(True)
plt.title('Решения для желаемого q = {}:'.format(q1_desirable))
plt.xlabel('t', fontsize=12)
plt.ylabel('x(t)', fontsize=12)
plt.plot(time_list, position_list, color='c', label="При Kp = {}, Кi = {}, Kd = {}".format(kp, ki, kd), linewidth=2)
plt.plot(t_q, q_d, color='k', label="Желаемое значение q", linewidth=2)
plt.legend(loc='best')
plt.show()

p.disconnect()
