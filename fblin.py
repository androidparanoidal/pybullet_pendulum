import pylab
import math
import numpy as np
import control.matlab

dt = 1/240
b = 1
mass = 2
length = 0.8
g = 9.81
c1 = b / (mass * length ** 2)
c2 = g / length
c3 = 1 / (mass * length ** 2)
T = int(5 / dt)
TM = [0] * T
h = 1 / 240
m = 20
pi = math.pi

u_b = [0 for j in range(m)]

poles = np.array(([-8], [-4]))
A = np.array(([0.0, 1.0], [0.0, 0.0]))
B = np.array(([0.0], [1.0]))
K = control.matlab.place(A, B, poles)
K_m = (np.asarray(K)).flatten()

x_start = 1.7
x_d = 2.9


def rp_nonlin(x):
    x = np.array(([x[0]-x_d], [x[1]]))
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


def rp_nonlin_pred(x, tau):
    x = np.array(([x[0]], [x[1]]))
    dx = np.matmul(A, x) + (B * tau)
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
    for i in range(N):
        tau = u_b[0]
        dx = func(x, tau)[1]
        u_b.pop(0)
        x[1] = x[1] + h * dx
        x[0] = x[0] + h * x[1]
        x_m = np.vstack((x_m, x))

        x_mm = x_m[-1]
        c = prediction(x_mm, u_b, func)
        tau_prev = (-1) * ((K_m[0] * (c[0] - x_d) + K_m[1] * c[1]) * mass * length**2) + b * c[1] + mass * g * length * np.sin(c[0]-x_d)
        u_b.append(tau_prev)
    return x_m
sol_pred = euler_pred(TM, rp_nonlin_pred, x_start)


t1 = np.linspace(0, 1200*1/240, 1200)
t3 = np.linspace(m*1/240, 1200*1/240+(m*1/250), 1200)
t2 = np.linspace(0, 5)
xD = np.full(50, x_d)


pylab.figure(1)
pylab.grid()
pylab.title('График решения для x_d = {}'.format(x_d))
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(t2, xD, color='k', linestyle=':', label="Желаемое значение")
pylab.plot(t1, sol[:, 0], color='b', label='Нелинейная система без всего')
pylab.plot(t1, sol_zap[:, 0], color='c', label='Нелинейная система с запаздыванием')
pylab.plot(t3, sol_pred[:, 0], color='g', label='Нелинейная система с запаздыванием и прогнозом')
pylab.legend()
pylab.show()
