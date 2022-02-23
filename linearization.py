import pybullet as p
import pybullet_data
from scipy.integrate import odeint
import pylab
import numpy as np
import math
import control
import control.matlab

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# q0 = 0  # starting position (radian)
b = 1
m = 2
length = 0.8
g = 9.81
c1 = b / (m * length ** 2)
c2 = g / length
c3 = 1 / (m * length ** 2)
time_list = []
position_list = []
w_list = []
upr_m_list = np.array([])
upr_s_list = np.array([])

A = np.array(([0.0, 1.0], [c2, -c1]))
B = np.array(([0.0], [c3]))
poles = np.array(([-5], [-2]))
K = control.matlab.place(A, B, poles)
C = A - np.dot(B, K)
sch, sv = np.linalg.eig(C)

print('\nК: ', K)
print('Ранг матрицы А = ', np.linalg.matrix_rank(A))
print('Ранг матрицы В = ', np.linalg.matrix_rank(B))
print('Матрица преобразованной системы = С = ', C)
print('Собственные числа этой матрицы: ', sch)
print('Собственные вектора этой матрицы: ', sv)
print('Число обусловленности = ', np.linalg.cond(sv))


# Симуляторное решение
def sim_solution(q0, K):
    t = 0
    dt = 1/240
    # go to the starting position
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    q0 = jointPosition

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=0.1)

    position_list = []
    jointpos_prev = q0
    K_m = (np.asarray(K)).flatten()

    while t < 5:
        jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
        position_list.append(jointPosition)

        uglskor = (jointPosition - jointpos_prev)/dt
        w_list.append(uglskor)
        jointpos_prev = jointPosition

        vec_0 = np.array(jointPosition-math.pi)
        vec_1 = np.array(uglskor)
        vec_s = np.vstack((vec_0, vec_1))
        torque = (-1) * (K_m @ vec_s)

        global upr_s_list
        upr_s_list = np.append(upr_s_list, torque)
        p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=torque)
        p.stepSimulation()
        time_list.append(t)
        t += dt
    return position_list

#simsol1 = sim_solution(0.5, c1, c2, c3)
#simsol2 = sim_solution((math.pi)/2, c1, c2, c3)
simsol3 = sim_solution(0.7, K)
# print('sim', simsol3)


# q0_initial = [q0, 0]  # нач знач: x(t=0) = 0 = q0, x'(t=0) = 0
# Численное решение линейной системы d^2x/dt^2 +  b/(m*l*l) * dx/dt + g/l * x - force /(m*l^2)= 0
def model_1(X, t, b, m, length, g):
    x, x_new = X
    '''#Если управление на первом такте: 
    dt = 1/240
    force = 0
    if (t < dt):
        upr = 0.1
        force = upr / (m * length**2)
    '''
    k1, k2 = 5, 4
    dx_new = [x_new, -(b / (m * length**2)) * x_new - (g / length) * (x - math.pi) + (- k1 * x - k2 * x_new) / (m * length**2)]
    return dx_new


t = np.linspace(0, 5, 5*240)
#solution1 = odeint(model_1, [0.5, 0], t, args=(b, m, length, g))
#solution2 = odeint(model_1, [(math.pi)/2, 0], t, args=(b, m, length, g))
solution3 = odeint(model_1, [math.pi, 0], t, args=(b, m, length, g))


# Решение матричного уравнения dX/dt = A*X + B*u, X = [x, dx], управление u = -K*X => dX/dt = (A-B*K)*X
def model_2(X, t, c1, c2, c3):
    A = np.array(([0.0, 1.0], [-c2, -c1]))
    B = np.array(([0.0], [c3]))
    poles = np.array(([-5], [-2]))
    K = control.matlab.place(A, B, poles)
    C = A - np.dot(B, K)
    X = np.array([X[0], X[1]])
    # print(X)
    U = (-1) * np.array(K @ X)
    global upr_m_list
    upr_m_list = np.append(upr_m_list, U)

    rhs = np.matmul(C, X)
    rhs = rhs.reshape(1, 2)
    dX_new = rhs.tolist()

    return dX_new[0]


m_solution = odeint(model_2, [(math.pi), 0], t, args=(c1, c2, c3))
# print(m_solution[:, 0])
print(len(m_solution[:, 0]))
print(len(upr_m_list))


# Решение матричного уравнения с заменой dy/dt = A*y + B*u, y = [dx, x-pi]
def model_3(Y, t, c1, c2, c3):
    A = np.array(([0.0, 1.0], [c2, -c1]))
    B = np.array(([0.0], [c3]))
    poles = np.array(([-5], [-2]))
    K = control.matlab.place(A, B, poles)
    C = A - np.dot(B, K)
    Y = np.array([Y[0]-math.pi, Y[1]])
    # print(Y)
    U = (-1) * np.array(K @ Y)
    global upr_m_list
    upr_m_list = np.append(upr_m_list, U)

    rhs = np.matmul(C, Y)
    rhs = rhs.reshape(1, 2)
    dX_new = rhs.tolist()

    return dX_new[0]


m2_solution = odeint(model_3, [0.7, 0], t, args=(c1, c2, c3))


color1 = (0.1, 0.2, 1.0)
pylab.figure(1)
pylab.grid()
pylab.title("Сравнение симуляторного и численного решения линейной системы:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
#pylab.plot(t, simsol1, color='m', label='Симуляторное при q0 = 0.1')
#pylab.plot(t, solution1[:, 0], color='k', label='Линейная система при q0 = 0.1', linestyle=':')
#pylab.plot(t, simsol2, color='g', label='Симуляторное при q0 = pi/2')
#pylab.plot(t, solution2[:, 0], color='k', label='Линейная система при q0 = pi/2', linestyle=':')
#pylab.plot(t, solution3[:, 0], color='r', label='Линейная система при q0 = pi', linestyle=':')
pylab.plot(t, np.array(simsol3), color=color1, label='Симуляторное при q0 = pi')
pylab.plot(t, m_solution[:, 0], color='k', label='Решение матричного уравнения')
pylab.plot(t, m2_solution[:, 0], color='c', label='Решение матричного уравнения 2')
pylab.legend()


pylab.figure(2)
pylab.grid()
pylab.title("Графики угловой скорости:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('ω', fontsize=12)
pylab.plot(t, w_list, color=color1, label='Угловая скорость симуляторного решения')
#pylab.plot(t, solution3[:, 1], color='r', label=' ')
pylab.plot(t, m_solution[:, 1], color='k', label='Угловая скорость для матричной системы')
pylab.plot(t, m2_solution[:, 1], color='c', label='Угловая скорость для матричной системы 2')
pylab.legend()


'''
t1 = np.linspace(0, 5, 243)
pylab.figure(3)
pylab.grid()
pylab.title("Графики управления:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('u', fontsize=12)
pylab.plot(t1, upr_m_list, color='k', label='Управление для матричной системы')
pylab.plot(t, upr_s_list, color=color1, label='Управление для симуляторного решения')
pylab.legend()
'''

'''
pylab.figure(4)  # наверное можно циклом нарисовать..........
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
