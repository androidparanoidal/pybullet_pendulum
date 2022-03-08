import pybullet as p
import pybullet_data
import pylab
import numpy as np
import math
import control.matlab

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
T = int(5/dt)

time_list = [0]*T
position_list = [0]*T
w_list = [0]*T

upr_s_list = np.array([])
u_buffer = []


A = np.array(([0.0, 1.0], [c2, -c1]))
B = np.array(([0.0], [c3]))
poles = np.array(([-50], [-20]))
K = control.matlab.place(A, B, poles)
C = A - np.dot(B, K)
sch, sv = np.linalg.eig(C)

'''
print('\nК: ', K)
print('Ранг матрицы А = ', np.linalg.matrix_rank(A))
print('Ранг матрицы В = ', np.linalg.matrix_rank(B))
print('Матрица преобразованной системы = С = ', C)
print('Собственные числа этой матрицы: ', sch)
print('Собственные вектора этой матрицы: ', sv)
print('Число обусловленности = ', np.linalg.cond(sv))
'''

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
    T = int(5 / dt)

    position_list = [0]*T
    jointpos_prev = q0
    K_m = (np.asarray(K)).flatten()

    for i in range(0, T):
        jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
        position_list[i] = jointPosition

        jointVelocity = (jointPosition - jointpos_prev)/dt
        w_list[i] = jointVelocity
        jointpos_prev = jointPosition

        vec_0 = np.array(jointPosition-math.pi)
        vec_1 = np.array(jointVelocity)
        vec_s = np.vstack((vec_0, vec_1))

        global upr_s_list

        if t < dt:
            torque_prev = (-1) * (K_m @ vec_s)
            u_buffer.append(torque_prev)
            torque = 0
            p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL,
                                    force=torque)
            upr_s_list = np.append(upr_s_list, torque)

        else:
            torque_prev = (-1) * (K_m @ vec_s)
            u_buffer[0] = torque_prev
            p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL,
                                    force=torque_prev)
            upr_s_list = np.append(upr_s_list, torque_prev)

        p.stepSimulation()
        time_list[i] = t
        t += dt
    print(position_list)
    print(w_list)
    print(upr_s_list)
    return position_list


simsol = sim_solution(math.pi-0.1, K)


pylab.figure(1)
pylab.grid()
pylab.title("График симуляторного решения:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(time_list, np.array(simsol), color='c', label='sim')
pylab.legend()

pylab.figure(2)
pylab.grid()
pylab.title("График угловой скорости:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('ω', fontsize=12)
pylab.plot(time_list, w_list, color='c', label='sim')
pylab.legend()

pylab.figure(3)
pylab.grid()
pylab.title("График управления:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('u', fontsize=12)
pylab.plot(time_list, upr_s_list, color='c', label='sim')
pylab.legend()

pylab.show()

p.disconnect()
