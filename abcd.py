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
m = 20
pi = math.pi

A = np.array(([0.0, 1.0], [c2, -c1]))
B = np.array(([0.0], [c3]))
poles = np.array(([-10], [-4]))
K = control.matlab.place(A, B, poles)
K_m = (np.asarray(K)).flatten()

u_b = [0 for j in range(m)]

def rp_nonlin(x, tau):
    x = np.array(([x[0]], [x[1]]))
    el1 = (x[1]).tolist()
    el2 = (np.sin(x[0])).tolist()
    Ax = np.array(([el1[0]], [-c2 * el2[0] - c1 * el1[0]]))
    dx = Ax + (B * tau)
    dx = dx.reshape(1, 2)
    dx_n = dx.tolist()
    return dx_n[0]


def prediction(x, u_b):
    y = rp_nonlin(x, u_b)
    # print(y)
    return y[0]


def euler(t, func, x0):
    N = np.size(t) - 1
    h = 1 / 240
    p0 = x0
    v0 = 0
    x = [p0, v0]
    x_m = np.array([x])
    # tau_0 = (-1) * K_m @ x_m

    for i in range(N):
        x_b = x_m[-1]
        tau = u_b[0]
        dx = func(x, tau)[1]
        u_b.pop(0)
        x[1] = x[1] + h * dx
        x[0] = x[0] + h * x[1]
        x_m = np.vstack((x_m, x))
        #c = prediction(x_m, u_b)[-1]
        #print(c)
        tau_prev = (-1) * (K_m[0] * (x_b[0] - pi) + K_m[1] * x_b[1])   #  #(K_m @ c)
        u_b.append(tau_prev)
    return x_m


x_start = math.pi-0.1
solution = euler(TM, rp_nonlin, x_start)
print(solution)

t1 = np.linspace(0, 1200*1/240, 1200)
t3 = np.linspace(m*1/240, 1200*1/240+(m*1/250), 1200)
xx0 = 0.1

pylab.figure(1)
pylab.grid()
pylab.title("График решения:")
pylab.xlabel('t', fontsize=12)
pylab.ylabel('x(t)', fontsize=12)
pylab.plot(t1, solution[:, 0], color='b', label='Нелинейная система для x0 = π - {}'.format(xx0))
pylab.legend()
pylab.show()
