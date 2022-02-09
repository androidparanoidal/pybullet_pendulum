import pybullet as p
import pybullet_data
from scipy.integrate import odeint
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
dt = 1/240  # pybullet simulation step
q0 = 0.1  # starting position (radian) 1.58

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
q0 = jointPosition

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

# Симуляторное решение
time_list = []
position_list = []

while t < 5:
    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    position_list.append(jointPosition)
    p.stepSimulation()
    time_list.append(t)
    t += dt


# Численное решение с помощью odeint
b = 1
m = 1
length = 0.8
g = 9.81
koef1 = g/length
koef2 = b/(m * length**2)
q0_initial = [q0, 0]  # нач знач: x(t=0) = 1, x'(t=0) = 0

# d^2x/dt^2 +  b/(m*l*l) * dx/dt + g/l * x = 0, где пусть
def odesol(X, t, b, m, length, g):
    x, x_new = X
    dx_new = [x_new, -(b/(m * length**2)) * x_new - (g/length) * x]
    return dx_new

t = np.linspace(0, 5, 5*240)
solution = odeint(odesol, q0_initial, t, args=(b, m, length, g))
xs = solution[:, 0]

plt.figure()
plt.grid(True)
plt.xlabel('t', fontsize=12)
plt.ylabel('x(t)', fontsize=12)
plt.plot(t, solution[:, 0], color='k', label='Линейная система', linewidth=2)
plt.plot(time_list, position_list, color='c', label='Симуляторное')
plt.legend(loc='best')
plt.title('Сравнение симуляторного и решения линейной системы:')
plt.show()

p.disconnect()