import pybullet as p
import pybullet_data
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import pylab
import numpy as np
import math
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# q0 = 0  # starting position (radian)
time_list = []
position_list = []

# Симуляторное решение
def sim_solution(q0):
    t = 0
    dt = 1/240
    # go to the starting position
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    q0 = jointPosition

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=0.1)
    position_list = []
    while t < 5:
        jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
        position_list.append(jointPosition)
        p.stepSimulation()
        time_list.append(t)
        t += dt
    return position_list

#simsol1 = sim_solution(0.5)
#simsol2 = sim_solution((math.pi)/2)
simsol3 = sim_solution(0)


# Численное решение линейной системы d^2x/dt^2 +  b/(m*l*l) * dx/dt + g/l * x = 0
b = 1
m = 1
length = 0.8
g = 9.81
# q0_initial = [q0, 0]  # нач знач: x(t=0) = 0 = q0, x'(t=0) = 0

def odesol(X, t, b, m, length, g):
    x, x_new = X
    '''Если управление на первом такте: 
    dt = 1/240
    force = 0
    if (t < dt):
        upr = 0.1
        force = upr / (m * length**2)
    '''
    k1, k2 = 1, 4
    dx_new = [x_new, -(b / (m * length**2)) * x_new - (g / length) * x + (- k1 * x - k2 * x_new) / (m * length**2)]
    return dx_new


t = np.linspace(0, 5, 5*240)
#solution1 = odeint(odesol, [0.5, 0], t, args=(b, m, length, g))
#solution2 = odeint(odesol, [(math.pi)/2, 0], t, args=(b, m, length, g))
solution3 = odeint(odesol, [0.1, 0], t, args=(b, m, length, g))


pylab.figure(1)
pylab.grid()
pylab.title("Сравнение симуляторного и численного решения линейной системы:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
#pylab.plot(t, simsol1, color='m', label='Симуляторное при q0 = 0.1')
#pylab.plot(t, solution1[:, 0], color='k', label='Линейная система при q0 = 0.1', linestyle=':')
#pylab.plot(t, simsol2, color='g', label='Симуляторное при q0 = pi/2')
#pylab.plot(t, solution2[:, 0], color='k', label='Линейная система при q0 = pi/2', linestyle=':')
pylab.plot(t, simsol3, color='c', label='Симуляторное при q0 = 0')
pylab.plot(t, solution3[:, 0], color='k', label='Линейная система при q0 = 0', linestyle=':')
pylab.legend()

'''
pylab.figure(2)  # наверное можно циклом нарисовать..........
pylab.grid()
pylab.subplot(2, 3, 1)
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(t, simsol1, color='m', label='Симуляторное при q0 = 0.5')
pylab.plot(t, solution1[:, 0], color='k', label='Линейная система при q0 = 0.5', linestyle=':')
pylab.legend(loc='best')

pylab.subplot(2, 3, 2)
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(t, simsol2, color='m', label='Симуляторное при q0 = pi/2')
pylab.plot(t, solution2[:, 0], color='k', label='Линейная система при q0 = pi/2', linestyle=':')
pylab.legend(loc='best')

pylab.subplot(2, 3, 3)
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(t, simsol3, color='m', label='Симуляторное при q0 = 0')
pylab.plot(t, solution3[:, 0], color='k', label='Линейная система при q0 = 0', linestyle=':')
pylab.legend(loc='best')
'''
pylab.show()

p.disconnect()
