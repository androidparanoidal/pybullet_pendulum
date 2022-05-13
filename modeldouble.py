import numpy as np
import math
import matplotlib.pyplot as plt


dt = 1 / 240
h = dt
L1, L2 = 0.8, 0.7
M1, M2 = 2, 1.5
g = 9.81
pi = math.pi

T = int(5 / dt)
TM = [0] * T

q_0 = np.array([0.1, 0.1, 0, 0])  # initial conditions


def model(x):
    x = np.array(([x[0]], [x[1]], [x[2]], [x[3]]))
    q1 = x[0]
    q2 = x[1]
    dq1 = x[2]
    dq2 = x[3]
    w = np.array(([dq1[0]], [dq2[0]]))
    tau = 0#np.array(([u[0]], [u[1]]))

    mm1 = (M1 * L1**2 + M2 * (L1**2 + 2*L1*L2 * np.cos(q2) + L2**2)).tolist()
    mm2 = (M2 * (L2 * L2 * np.cos(q2) + L2**2)).tolist()  # mm3 = mm2
    M = np.array([[mm1[0], mm2[0]],
                  [mm2[0], M2 * L2**2]])
    M_inv = np.linalg.inv(M)

    cc1 = (-M2 * L1 * L2 * np.sin(q2) * (2 * dq1 * dq2 + dq2**2)).tolist()
    cc2 = (M2 * L1 * L2 * np.sin(q2) * dq1**2).tolist()
    C = np.array(([cc1[0]], [cc2[0]]))

    gg1 = ((M1 + M2) * L1 * g * np.cos(q1) + M2 * g * L2 * np.cos(q1 + q2)).tolist()
    gg2 = (M2 * g * L2 * np.cos(q1 + q2)).tolist()
    G = np.array(([gg1[0]], [gg2[0]]))

    expr = tau - C - G - w
    ddq = np.dot(M_inv, expr)
    dx = np.concatenate((w, ddq))
    dx = dx.reshape(1, 4)
    return dx[0]


def euler(t, func, q_start):
    N = np.size(t) - 1
    pos = q_start
    pos_m = np.array(pos)
    for i in range(N):
        df1 = func(pos)[2]
        df2 = func(pos)[3]
        pos[2] = pos[2] + h * df1
        pos[0] = pos[0] + h * pos[2]
        pos[3] = pos[3] + h * df2
        pos[1] = pos[1] + h * pos[3]
        pos_m = np.vstack((pos_m, pos))
    return pos_m

el_sol = euler(TM, model, q_0)


t1 = np.linspace(0, 1200*1/240, 1200)
fig1 = plt.figure("Решение модели")
ax1 = fig1.add_subplot(321)
ax1.set_xlabel('t')
ax1.set_ylabel('q')
ax1.plot(t1, el_sol[:, 0])
ax1.grid()
ax2 = fig1.add_subplot(322)
ax2.set_xlabel('t')
ax2.set_ylabel('q')
ax2.plot(t1, el_sol[:, 1])
ax2.grid()
ax1.title.set_text('1 звено:')
ax2.title.set_text('2 звено:')

ax3 = fig1.add_subplot(323)
ax3.set_xlabel('t')
ax3.set_ylabel("q'")
ax3.plot(t1, el_sol[:, 2])
ax3.grid()
ax4 = fig1.add_subplot(324)
ax4.set_xlabel('t')
ax4.set_ylabel("q'")
ax4.plot(t1, el_sol[:, 3])
ax4.grid()
plt.show()
