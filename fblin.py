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

A = np.array(([0.0, 1.0], [c2, -c1]))
B = np.array(([0.0], [c3]))
poles = np.array(([-10], [-4]))
K = control.matlab.place(A, B, poles)
K_m = (np.asarray(K)).flatten()

An = np.array(([0.0, 1.0], [0.0, 0.0]))
Bn = np.array(([0.0], [1.0]))
Kn = control.matlab.place(An, Bn, poles)
K_mn = (np.asarray(Kn)).flatten()

x_start = 0.2
x_d = 1.5

def rp_nonlin(x, tau):
    x = np.array(([x[0]], [x[1]]))
    el1 = (x[1]).tolist()
    el2 = (np.sin(x[0])).tolist()
    Ax = np.array(([el1[0]], [-c2 * el2[0] - c1 * el1[0]]))
    dx = Ax + (B * tau)
    dx = dx.reshape(1, 2)
    dx_n = dx.tolist()
    return dx_n[0]

def euler(t, func, x0):   # tau = Kx
    N = np.size(t) - 1
    p0 = x0
    v0 = 0
    x = [p0, v0]
    x_m = np.array([x])
    for i in range(N):
        x_b = x_m[-1]
        tau = (-1) * (K_m[0] * (x_b[0] - x_d) + K_m[1] * x_b[1])
        dx = func(x, tau)[1]
        x[1] = x[1] + h * dx
        x[0] = x[0] + h * x[1]
        x_m = np.vstack((x_m, x))
    return x_m
sol = euler(TM, rp_nonlin, x_start)


# tau = m*g*L*math.sin(q) + b*dq + m*L*L*u  при u = K*x
def rp_nonlin2(x):
    x = np.array(([x[0]-x_d], [x[1]]))
    el = (np.sin(x[0])).tolist()
    tau = (-1) * ((Kn @ x) * mass * length**2) + b * x[1] + mass * g * length * el[0]
    dx = np.matmul(An, x) + (Bn * tau)
    dx = dx.reshape(1, 2)
    dx_n = dx.tolist()
    return dx_n[0]


def euler2(t, func, q0):
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
sol2 = euler2(TM, rp_nonlin2, x_start)


t1 = np.linspace(0, 1200*1/240, 1200)
t2 = np.linspace(0, 5)
xD = np.full(50, x_d)


pylab.figure(1)
pylab.grid()
pylab.title('График решения для x_d = {}'.format(x_d))
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(t2, xD, color='k', linestyle=':', label="Желаемое значение")
pylab.plot(t1, sol[:, 0], color='b', label='Нелинейная система с u = KX')
pylab.plot(t1, sol2[:, 0], color='c', label='Нелинейная система с новым упр')
pylab.legend()
pylab.show()
