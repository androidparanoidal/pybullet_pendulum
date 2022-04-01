import pybullet as p
import pybullet_data
import pylab
import numpy as np
import math
import control.matlab
# import collections

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

dt = 1/240
b = 1
m = 2
length = 0.8
g = 9.81
c1 = b / (m * length ** 2)
c2 = g / length
c3 = 1 / (m * length ** 2)
T = int(5 / dt)

time_list = [0]*T
position_list = [0]*T
w_list = [0]*T
position_list2 = []
time_list2 = []
w_list2 = []

upr_s2_list = np.array([])
upr_s_list = np.array([])
upr_pen_list = np.array([])

# u_buff = collections.deque([0 for j in range(10)])
u_buff = []

'''
a_list = collections.deque([1, 2, 3, 4, 5])
a_list.rotate(2)
shifted_list = list(a_list)
print(shifted_list)
'''

A = np.array(([0.0, 1.0], [c2, -c1]))
B = np.array(([0.0], [c3]))
poles = np.array(([-10], [-4]))
K = control.matlab.place(A, B, poles)
C = A - np.dot(B, K)
sch, sv = np.linalg.eig(C)
K_m = (np.asarray(K)).flatten()

'''
print('\nК: ', K)
print('Ранг матрицы А = ', np.linalg.matrix_rank(A))
print('Ранг матрицы В = ', np.linalg.matrix_rank(B))
print('Матрица преобразованной системы = С = ', C)
print('Собственные числа этой матрицы: ', sch)
print('Собственные вектора этой матрицы: ', sv)
print('Число обусловленности = ', np.linalg.cond(sv))
'''


# Симуляторное решение с запаздыванием
def sim_sol_delay(q0, K_m):
    t = 0
    dt = 1/240
    # go to the starting position
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    q0 = jointPosition

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
    T = int(5 / dt)

    position_list = [0]*T
    jointpos_prev = q0
    u_buff = [0 for j in range(20)]

    for i in range(0, T):
        global upr_s_list
        jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
        position_list[i] = jointPosition

        jointVelocity = (jointPosition - jointpos_prev)/dt
        w_list[i] = jointVelocity
        jointpos_prev = jointPosition

        vec_0 = np.array(jointPosition-math.pi)
        vec_1 = np.array(jointVelocity)
        vec_s = np.vstack((vec_0, vec_1))

        torque = u_buff[0]
        upr_s_list = np.append(upr_s_list, torque)
        p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL,
                                force=torque)
        u_buff.pop(0)
        torque_prev = (-1) * (K_m @ vec_s)
        u_buff.append(torque_prev)

        p.stepSimulation()
        time_list[i] = t
        t += dt
    '''
    print('Симулятор delay...')
    for e in upr_s_list:
        print(e, end=" ")
    print('\n')
    print(position_list)
    print(w_list)
    '''
    return position_list


ssol_delay = sim_sol_delay(math.pi-0.1, K)


# Симуляторное решение
def sim_sol(q0, K_m):
    t = 0
    dt = 1 / 240
    # go to the starting position
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
    q0 = jointPosition
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
    position_list2 = []
    jointpos_prev = q0

    while t < 5:
        jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
        position_list2.append(jointPosition)

        uglskor = (jointPosition - jointpos_prev) / dt
        w_list2.append(uglskor)
        jointpos_prev = jointPosition

        vec_0 = np.array(jointPosition - math.pi)
        vec_1 = np.array(uglskor)
        vec_s = np.vstack((vec_0, vec_1))

        torque = (-1) * (K_m @ vec_s)
        global upr_s2_list
        upr_s2_list = np.append(upr_s2_list, torque)
        p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL,
                                force=torque)

        p.stepSimulation()
        time_list2.append(t)
        t += dt
    '''
    print('\nСимулятор...')
    print(position_list2)
    print(w_list2)
    for e in upr_s2_list:
        print(e, end=" ")
    '''
    return position_list2


ssol = sim_sol(math.pi-0.1, K)

print('test:')
vect = np.array(([2], [-1]))
print('исходный \n', vect)
vect = np.delete(vect, 0)
buff = [1, 2, 3, 4, 5]
chvec = np.insert(vect, 0, buff[4], axis=0)
print('измененный ', chvec)
print('result:')
newvec = np.reshape(chvec, (-1, 1))
print(newvec)


u_b = [0 for j in range(20)]

def pendulum_func(Y, t, A, B, K_m):
    Y = np.array(([Y[0]-math.pi],
                  [Y[1]]))
    global upr_pen_list
    Upr = u_b[0]
    upr_pen_list = np.append(upr_pen_list, Upr)
    dy = np.matmul(A, Y) + (B * Upr)
    u_b.pop(0)
    Upr_prev = (-1) * (K_m @ Y)
    u_b.append(Upr_prev)

    dy = dy.reshape(1, 2)
    dY_new = dy.tolist()
    return dY_new[0]


def euler(func, q0, t):
    n = t - 1
    h = 1 / 240
    p0 = q0
    v0 = 0
    pos = [p0, v0]
    pos_m = np.array(pos)
    for i in range(n):
        f = pendulum_func(pos, 0, A, B, K_m)[1]
        pos[1] = pos[1] + h * f
        pos[0] = pos[0] + h * pos[1]
        pos_m = np.vstack((pos_m, pos))
    # print(pos_m)
    return pos_m


el_sol = euler(pendulum_func, math.pi-0.1, T)
# print(el_sol)
t1 = np.linspace(0, 1200*1/240, 1200)

pylab.figure(1)
pylab.grid()
pylab.title("График симуляторного решения:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(time_list, np.array(ssol_delay), color='c', label='sim delay')
pylab.plot(t1, el_sol[:, 0], color='r', label='euler')
pylab.plot(time_list, np.array(ssol), color='k', linestyle=':', label='sim')
pylab.legend()
'''
pylab.figure(2)
pylab.grid()
pylab.title("График угловой скорости:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('ω', fontsize=12)
pylab.plot(time_list, w_list, color='c', label='sim delay')
pylab.plot(time_list, w_list2, color='k', linestyle=':', label='sim')
pylab.legend()

pylab.figure(3)
pylab.grid()
pylab.title("График управления:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('u', fontsize=12)
pylab.plot(time_list, upr_s_list, color='c', label='sim delay')
pylab.plot(time_list, upr_s2_list, color='k', linestyle=':', label='sim')
pylab.legend()
'''
pylab.show()

p.disconnect()
