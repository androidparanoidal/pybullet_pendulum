import pybullet as p
import pybullet_data
import numpy as np
import math
import matplotlib.pyplot as plt
import control.matlab

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./double_pendulum.urdf", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 3, linearDamping=0, angularDamping=0)

'''for _id in range(p.getNumJoints(boxId)):
    print(f'{_id} {p.getJointInfo(boxId, _id)[1]}')'''

dt = 1 / 240
h = dt
L1, L2 = 0.8, 0.7
M1, M2 = 2, 1.5
g = 9.81
pi = math.pi

the_0 = [0.0, 0.0]
q_0 = np.array([0.0, 0.0, 0, 0])
q_d1 = pi/2
q_d2 = pi/2

T = int(8 / dt)
TM = [0] * T
upr_m1_list = np.array([])
upr_m2_list = np.array([])
upr_s1_list = np.array([])
upr_s2_list = np.array([])

m = 20
u_b = [[0 for col in range(m)] for row in range(2)]

poles = np.array(([-2], [-1], [-3], [-4]))
B = np.array(([0, 0], [0, 0], [1, 0], [0, 1]))
A = np.array(([0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0],
              [0, 0, 0, 0]))
Km = control.matlab.place(A, B, poles)
K = (np.asarray(Km)).flatten()
K1 = K[:4]
K2 = K[4:]


def model(x, TAU):
    x = np.array(([x[0]], [x[1]], [x[2]], [x[3]]))
    q1 = x[0]
    q2 = x[1]
    dq1 = x[2]
    dq2 = x[3]
    w = np.array(([dq1[0]], [dq2[0]]))
    damp = np.array([1, 1])
    dem_w1 = damp[0] * w[0]
    dem_w2 = damp[1] * w[1]
    dem_w = np.array(([dem_w1[0]], [dem_w2[0]]))

    UPR = np.array(([TAU[0]], [TAU[1]]))

    mm1 = (M1 * L1**2 + M2 * (L1**2 + 2*L1*L2 * np.cos(q2) + L2**2)).tolist()
    mm2 = (M2 * (L2 * L2 * np.cos(q2) + L2**2)).tolist()  # mm3 = mm2
    M = np.array([[mm1[0], mm2[0]], [mm2[0], M2 * L2**2]])
    M_inv = np.linalg.inv(M)

    cc1 = (-M2 * L1 * L2 * np.sin(q2) * (2 * dq1 * dq2 + dq2**2)).tolist()
    cc2 = (M2 * L1 * L2 * np.sin(q2) * dq1**2).tolist()
    C = np.array(([cc1[0]], [cc2[0]]))

    gg1 = ((M1 + M2) * L1 * g * np.cos(q1) + M2 * g * L2 * np.cos(q1 + q2)).tolist()
    gg2 = (M2 * g * L2 * np.cos(q1 + q2)).tolist()
    G = np.array(([gg1[0]], [gg2[0]]))

    expr = UPR - C - G - dem_w
    ddq = np.dot(M_inv, expr)
    dx = np.concatenate((w, ddq))
    dx = dx.reshape(1, 4)
    return dx[0]


def euler_step(x0, b1, b2, func):
    p1 = x0[0]
    p2 = x0[1]
    v1 = x0[2]
    v2 = x0[3]
    pos = [p1, p2, v1, v2]
    buf = [b1, b2]
    df = func(pos, buf)
    pos[2] = pos[2] + h * df[2]
    pos[0] = pos[0] + h * pos[2]
    pos[3] = pos[3] + h * df[3]
    pos[1] = pos[1] + h * pos[3]
    pos_m = [pos[0], pos[1], pos[2], pos[3]]
    return pos_m

def prediction(x, buffer, func):
    buf = buffer
    for i in range(m-1):
        x = euler_step(x, buf[0][i], buf[1][i], func)
    return x


def euler_pred(t, func, q_start):
    N = np.size(t) - 1
    pos = q_start
    pos_m = np.array(pos)
    global upr_m1_list, upr_m2_list
    for i in range(N):
        u1 = u_b[0][0]
        u2 = u_b[1][0]
        uv = np.array([u1, u2])
        upr_m1_list = np.append(upr_m1_list, u1)
        upr_m2_list = np.append(upr_m2_list, u2)
        df = func(pos, uv)
        u_b[0].pop(0)
        u_b[1].pop(0)
        pos[2] = pos[2] + h * df[2]
        pos[0] = pos[0] + h * pos[2]
        pos[3] = pos[3] + h * df[3]
        pos[1] = pos[1] + h * pos[3]
        pos_m = np.vstack((pos_m, pos))
        pos_mm = pos_m[-1]
        c = prediction(pos_mm, u_b, func)
        q1 = c[0]
        q2 = c[1]
        dq1 = c[2]
        dq2 = c[3]
        w = np.array(([dq1], [dq2]))

        mm1 = (M1 * L1 ** 2 + M2 * (L1 ** 2 + 2 * L1 * L2 * np.cos(q2) + L2 ** 2)).tolist()
        mm2 = (M2 * (L2 * L2 * np.cos(q2) + L2 ** 2)).tolist()
        M = np.array([[mm1, mm2], [mm2, M2 * L2 ** 2]])

        cc1 = (-M2 * L1 * L2 * np.sin(q2) * (2 * dq1 * dq2 + dq2 ** 2)).tolist()
        cc2 = (M2 * L1 * L2 * np.sin(q2) * dq1 ** 2).tolist()
        C = np.array(([cc1], [cc2]))

        gg1 = ((M1 + M2) * L1 * g * np.cos(q1) + M2 * g * L2 * np.cos(q1 + q2)).tolist()
        gg2 = (M2 * g * L2 * np.cos(q1 + q2)).tolist()
        G = np.array(([gg1], [gg2]))

        k1c = K1[0] * (c[0] - q_d1) + K1[1] * (c[1] - q_d2) + K1[2] * c[2] + K1[3] * c[3]
        k2c = K2[0] * (c[0] - q_d1) + K2[1] * (c[1] - q_d2) + K2[2] * c[2] + K2[3] * c[3]
        U = (-1) * np.array(([k1c], [k2c]))
        u_prev = (M @ U) + C + G + w
        u_b[0].append(u_prev[0][0])
        u_b[1].append(u_prev[1][0])
    return pos_m

el_sol = euler_pred(TM, model, q_0)
em1 = upr_m1_list[-1]
em2 = upr_m2_list[-1]
upr_m1_list = np.append(upr_m1_list, em1)
upr_m2_list = np.append(upr_m2_list, em2)


joint_id = [1, 3]

def pendulum_sim(the0, func):
    t = 0
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetPositions=the0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.VELOCITY_CONTROL, forces=[0.0, 0.0])
    PLIST = []
    tau_b = [[0 for _ in range(m)] for _ in range(2)]
    global upr_s1_list, upr_s2_list

    for i in range(0, T):
        j1 = p.getJointStates(boxId, jointIndices=joint_id)[0]
        j2 = p.getJointStates(boxId, jointIndices=joint_id)[1]
        j_pos1, j_pos2 = j1[0], j2[0]
        j_vel1, j_vel2 = j1[1], j2[1]
        vec = [j_pos1, j_pos2, j_vel1, j_vel2]
        PLIST.append(vec)

        d = prediction(vec, tau_b, func)
        q1, q2, dq1, dq2 = d[0], d[1], d[2], d[3]

        w = np.array(([dq1], [dq2]))
        tr1 = tau_b[0][0]
        tr2 = tau_b[1][0]
        upr_s1_list = np.append(upr_s1_list, tr1)
        upr_s2_list = np.append(upr_s2_list, tr2)
        torques = [tr1, tr2]
        p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.TORQUE_CONTROL, forces=torques)
        tau_b[0].pop(0)
        tau_b[1].pop(0)

        mm1 = (M1 * L1 ** 2 + M2 * (L1 ** 2 + 2 * L1 * L2 * np.cos(q2) + L2 ** 2)).tolist()
        mm2 = (M2 * (L2 * L2 * np.cos(q2) + L2 ** 2)).tolist()
        M = np.array([[mm1, mm2], [mm2, M2 * L2 ** 2]])
        cc1 = (-M2 * L1 * L2 * np.sin(q2) * (2 * dq1 * dq2 + dq2 ** 2)).tolist()
        cc2 = (M2 * L1 * L2 * np.sin(q2) * dq1 ** 2).tolist()
        C = np.array(([cc1], [cc2]))
        gg1 = ((M1 + M2) * L1 * g * np.cos(q1) + M2 * g * L2 * np.cos(q1 + q2)).tolist()
        gg2 = (M2 * g * L2 * np.cos(q1 + q2)).tolist()
        G = np.array(([gg1], [gg2]))

        k1d = K1[0] * (q1 - q_d1) + K1[1] * (q2 - q_d2) + K1[2] * dq1 + K1[3] * dq2
        k2d = K2[0] * (q1 - q_d1) + K2[1] * (q2 - q_d2) + K2[2] * dq1 + K2[3] * dq2
        U = (-1) * np.array(([k1d], [k2d]))
        trq_prev = (M @ U) + C + G + w
        tau_b[0].append(trq_prev[0][0])
        tau_b[1].append(trq_prev[1][0])
        p.stepSimulation()
        t += dt

    pos_list = np.stack(PLIST, axis=0)
    return pos_list
sol_sim = pendulum_sim(the_0, model)



p.disconnect()
t1 = np.linspace(0, 1920*1/240, 1920)
t2 = np.linspace(0, 8)
qd1 = np.full(50, q_d1)
qd2 = np.full(50, q_d2)

fig1 = plt.figure("Графики решений")
ax1 = fig1.add_subplot(321)
ax1.set_ylabel('q')
ax1.plot(t2, qd1, color='k', linestyle=':')
ax1.plot(t1, el_sol[:, 0], label='model')
ax1.plot(t1, sol_sim[:, 0], label='sim')
ax1.legend()
ax1.grid()
ax2 = fig1.add_subplot(322)
ax2.set_ylabel('q')
ax2.plot(t2, qd2, color='k', linestyle=':')
ax2.plot(t1, el_sol[:, 1])
ax2.plot(t1, sol_sim[:, 1])
ax2.grid()
ax1.title.set_text('1 звено:')
ax2.title.set_text('2 звено:')
ax3 = fig1.add_subplot(323)
ax3.set_ylabel("q'")
ax3.plot(t1, el_sol[:, 2])
ax3.plot(t1, sol_sim[:, 2])
ax3.grid()
ax4 = fig1.add_subplot(324)
ax4.set_ylabel("q'")
ax4.plot(t1, el_sol[:, 3])
ax4.plot(t1, sol_sim[:, 3])
ax4.grid()
ax5 = fig1.add_subplot(325)
ax5.set_xlabel('t')
ax5.set_ylabel('u')
ax5.plot(t1, upr_m1_list)
ax5.plot(t1, upr_s1_list)
ax5.grid()
ax6 = fig1.add_subplot(326)
ax6.set_xlabel('t')
ax6.set_ylabel('u')
ax6.plot(t1, upr_m2_list)
ax6.plot(t1, upr_s2_list)
ax6.grid()
plt.suptitle('Желаемые значения {} для первого и {} для второго звена'.format(round(q_d1, 5), round(q_d2, 5)))
plt.show()
