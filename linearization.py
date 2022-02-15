import pybullet as p
import pybullet_data
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import pylab
import numpy as np
import math
import time
from control.matlab import *
from scipy import signal
import numpy.linalg


p.connect(p.DIRECT)
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
w_list = []

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
    jointpos_prev = q0

    while t < 5:
        jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
        position_list.append(jointPosition)
        uglskor = (jointPosition - jointpos_prev)/dt
        w_list.append(uglskor)
        jointpos_prev = jointPosition
        p.stepSimulation()
        time_list.append(t)
        t += dt
    return position_list

#simsol1 = sim_solution(0.5)
#simsol2 = sim_solution((math.pi)/2)
simsol3 = sim_solution(0.1)


# Численное решение линейной системы d^2x/dt^2 +  b/(m*l*l) * dx/dt + g/l * x = 0
b = 1
m = 1
length = 0.8
g = 9.81
# q0_initial = [q0, 0]  # нач знач: x(t=0) = 0 = q0, x'(t=0) = 0


def model_1(X, t, b, m, length, g):
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
solution3 = odeint(model_1, [0.1, 0], t, args=(b, m, length, g))


c1 = b / (m * length**2)
c2 = g / length
c3 = m * length**2
A = np.array(([0.0, 1.0],
              [-c2, -c1]))
B = np.array(([0.0], [c3]))
p = np.array(([-0.2], [-0.5]))
K = place(A, B, p)
C = A - np.dot(B, K)
print('\n К: ', K)
print('Ранг матрицы А = ', np.linalg.matrix_rank(A))
print('Ранг матрицы В = ', np.linalg.matrix_rank(B))
print('Матрица преобразованной системы = ', C)
print('Собственные числа и собственные вектора этой матрицы: ', np.linalg.eig(C))


# dX/dt = A*X + B*u, X = [x, dx], управление u = -K*X => dX/dt = (A-B*K)*X
def model(X, t, b, m, length, g):
    c1 = b / (m * length ** 2)
    c2 = g / length
    c3 = m * length ** 2
    A = np.array(([0.0, 1.0],
                  [-c2, -c1]))
    B = np.array(([0.0], [c3]))
    p = np.array(([-0.3], [-0.4]))
    K = place(A, B, p)
    C = A - np.dot(B, K)
    X = np.array([X[0], X[1]])

    RHS = np.matmul(C, X)
    RHS = RHS.reshape(1, 2)
    dX_new = RHS.tolist()
    return dX_new[0]

m_solution = odeint(model, [0.1, 0] ,t, args=(b, m, length, g))
#print(m_solution[:, 0])


pylab.figure(1)
pylab.grid()
pylab.title("Сравнение симуляторного и численного решения линейной системы:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
#pylab.plot(t, simsol1, color='m', label='Симуляторное при q0 = 0.1')
#pylab.plot(t, solution1[:, 0], color='k', label='Линейная система при q0 = 0.1', linestyle=':')
#pylab.plot(t, simsol2, color='g', label='Симуляторное при q0 = pi/2')
#pylab.plot(t, solution2[:, 0], color='k', label='Линейная система при q0 = pi/2', linestyle=':')
pylab.plot(t, simsol3, color='c', label='Симуляторное при q0 = 0.1')
pylab.plot(t, solution3[:, 0], color='k', label='Линейная система при q0 = 0.1', linestyle=':')
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

color1 = (0.1, 0.2, 1.0)
pylab.figure(3)
pylab.grid()
pylab.title("График угловой скорости:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('ω', fontsize=12)
pylab.plot(t, w_list, color=color1, label='Угловая скорость ω при q0 = 0.1')
pylab.legend()

color2 = (0.1, 1.0, 0.2)
pylab.figure(4)
pylab.grid()
pylab.title("Решение матричного уравнения:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(t, m_solution[:, 0], color=color2, label=' ')
pylab.legend()

pylab.show()

p.disconnect()
