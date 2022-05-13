import pylab
import math
import numpy as np
import control.matlab
import pybullet as p
import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)
boxId2 = p.loadURDF("./simple.urdf", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId2, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId2, 2, linearDamping=0, angularDamping=0)

dt = 1/240
T = int(5 / dt)
u_buff = []
time_list = [0]*T
position_list = [0]*T
w_list = [0]*T
upr_s_list = np.array([])
upr_m_list = np.array([])

b = 1
mass = 2
length = 0.8
g = 9.81
c1 = b / (mass * length ** 2)
c2 = g / length
c3 = 1 / (mass * length ** 2)
TM = [0] * T
h = 1 / 240
m = 20
pi = math.pi

u_b = [0 for j in range(m)]

poles = np.array(([-7], [-4]))
A = np.array(([0.0, 1.0], [0.0, 0.0]))
B = np.array(([0.0], [1.0]))
K = control.matlab.place(A, B, poles)
K_m = (np.asarray(K)).flatten()

x_start = 0.0
x_d = pi/2


def rp_nonlin(x):
    x = np.array(([x[0]-x_d], [x[1]]))
    print(x[0])
    el = (np.sin(x[0])).tolist()
    tau = (-1) * ((K @ x) * mass * length**2) + b * x[1] + mass * g * length * el[0]
    dx = np.matmul(A, x) + (B * tau)
    dx = dx.reshape(1, 2)
    dx_n = dx.tolist()
    return dx_n[0]

def rp_nonlin_zap(x):
    x = np.array(([x[0]-x_d], [x[1]]))
    el = (np.sin(x[0])).tolist()
    tau = u_b[0]
    dx = np.matmul(A, x) + (B * tau)
    u_b.pop(0)
    tau_prev = (-1) * ((K @ x) * mass * length**2) + b * x[1] + mass * g * length * el[0]
    u_b.append(tau_prev)
    dx = dx.reshape(1, 2)
    dx_n = dx.tolist()
    return dx_n[0]

def euler(t, func, q0):
    N = np.size(t) - 1
    p0 = q0
    v0 = 0
    pos = [p0, v0]
    pos_m = np.array(pos)
    for i in range(N):
        df = func(pos)[1]
        pos[1] = pos[1] + h * df
        pos[0] = pos[0] + h * pos[1]
        pos_m = np.vstack((pos_m, pos))
    return pos_m
sol = euler(TM, rp_nonlin, x_start)
sol_zap = euler(TM, rp_nonlin_zap, x_start)


Bn = np.array(([0.0], [c3]))
def rp_nonlin2(x, tau):
    x = np.array(([x[0]], [x[1]]))
    el1 = (x[1]).tolist()
    el2 = (np.sin(x[0])).tolist()
    An = np.array(([el1[0]], [-c2 * el2[0] - c1 * el1[0]]))
    dx = An + (Bn * tau)
    dx = dx.reshape(1, 2)
    dx_n = dx.tolist()
    return dx_n[0]


def euler_step(x0, buff, func):
    p0 = x0[0]
    v0 = x0[1]
    pos = [p0, v0]
    df = func(pos, buff)[1]
    pos[1] = pos[1] + h * df
    pos[0] = pos[0] + h * pos[1]
    pos_m = [pos[0], pos[1]]
    return pos_m


def prediction(x, u_b, func):
    buf = u_b
    for i in range(len(buf)-1):
        x = euler_step(x, buf[i], func)
    return x

def euler_pred(t, func, x0):
    N = np.size(t) - 1
    p0 = x0
    v0 = 0
    x = [p0, v0]
    x_m = np.array([x])
    global upr_m_list
    for i in range(N):
        tau = u_b[0]
        upr_m_list = np.append(upr_m_list, tau)
        dx = func(x, tau)[1]
        u_b.pop(0)
        x[1] = x[1] + h * dx
        x[0] = x[0] + h * x[1]
        x_m = np.vstack((x_m, x))
        x_mm = x_m[-1]
        c = prediction(x_mm, u_b, func)
        tau_prev = (-1) * ((K_m[0] * (c[0] - x_d) + K_m[1] * c[1]) * mass * length**2) + b * c[1] + mass * g * length * np.sin(c[0])
        u_b.append(tau_prev)

    return x_m
sol_pred = euler_pred(TM, rp_nonlin2, x_start)
l = upr_m_list[-1]
upr_m_list = np.append(upr_m_list, l)


def sim(q0, func):
    t = 0
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
    position_list = [0] * T
    u_buff = [0 for j in range(m)]

    for i in range(0, T):
        global upr_s_list
        jointPosition, *_ = p.getJointState(boxId, jointIndex=1)
        jointVelocity = p.getJointState(boxId, jointIndex=1)[1]
        position_list[i] = jointPosition
        w_list[i] = jointVelocity
        vec_0 = np.array(jointPosition)
        vec_1 = np.array(jointVelocity)
        vec_s = np.vstack((vec_0, vec_1))
        vec_ss = vec_s.reshape(1, 2)
        x_n = vec_ss[-1]
        f = prediction(x_n, u_buff, func)

        torque = u_buff[0]
        upr_s_list = np.append(upr_s_list, torque)
        p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.TORQUE_CONTROL, force=torque)
        u_buff.pop(0)

        torque_prev = (-1) * ((K_m[0] * (f[0]-x_d) + K_m[1] * f[1]) * mass * length ** 2) + b * f[1] + mass * g * length * np.sin(f[0])
        u_buff.append(torque_prev)
        p.stepSimulation()
        t += dt

    '''for e in upr_s_list:
        print(e, end="\n")'''
    return position_list

sim_sol = sim(x_start, rp_nonlin2)
p.disconnect()

t1 = np.linspace(0, 1200*1/240, 1200)
t3 = np.linspace(m*1/240, 1200*1/240+(m*1/240), 1200)
t2 = np.linspace(0, 5)
xD = np.full(50, x_d)


pylab.figure(1)
pylab.grid()
pylab.title('График решения для x_d = {}'.format(x_d))
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(t2, xD, color='k', linestyle=':', label="Желаемое значение")
#pylab.plot(t1, sol[:, 0], color='b', label='Нелинейная система без всего')
#pylab.plot(t1, sol_zap[:, 0], color='c', label='Нелинейная система с запаздыванием')
pylab.plot(t3, sol_pred[:, 0], color='g', label='Нелинейная система с запаздыванием и прогнозом')
pylab.plot(t3, np.array(sim_sol), color='c', label='Симулятор')
pylab.legend()
pylab.show()


pylab.figure(2)
pylab.grid()
pylab.title("График управления:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('u', fontsize=12)
pylab.plot(t1, upr_m_list, color='c', label='model')
pylab.plot(t1, upr_s_list, color='k', linestyle=':', label='sim')
pylab.legend()
pylab.show()
