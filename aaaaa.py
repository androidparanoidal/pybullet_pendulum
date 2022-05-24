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

dt = 1 / 240
h = dt
L1, L2 = 0.8, 0.7
M1, M2 = 2, 1.5
g = 9.81
pi = math.pi

the_0 = [pi / 2, pi / 2]
q_0 = np.array([pi / 2, pi / 2, 0, 0])
q_d1 = 0
q_d2 = 0

T = int(5 / dt)
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


def Mass_Matrix(x):
    q1, q2, dq1, dq2 = x[0], x[1], x[2], x[3]
    mm1 = M1 * L1 ** 2 + M2 * (L1 ** 2 + 2 * L1 * L2 * math.cos(q2) + L2 ** 2)
    mm2 = M2 * (L1 * L2 * math.cos(q2) + L2 ** 2)
    M = np.array([[mm1, mm2], [mm2, M2 * L2 ** 2]])
    return M


def Cor_Matrix(x):
    q1, q2, dq1, dq2 = x[0], x[1], x[2], x[3]
    cc1 = -M2 * L1 * L2 * math.sin(q2) * (2 * dq1 * dq2 + dq2 ** 2)

    cc2 = M2 * L1 * L2 * math.sin(q2) * dq1 ** 2
    C = np.array(([cc1[0]], [cc2[0]]))
    return C


def Cor_Matrixforsim(x):
    q1, q2, dq1, dq2 = x[0], x[1], x[2], x[3]
    cc1 = -M2 * L1 * L2 * math.sin(q2) * (2 * dq1 * dq2 + dq2 ** 2)
    cc2 = M2 * L1 * L2 * math.sin(q2) * dq1 ** 2
    C = np.array(([cc1], [cc2]))
    return C


def Grav_Matrix(x):
    q1, q2, dq1, dq2 = x[0], x[1], x[2], x[3]
    gg1 = (M1 + M2) * L1 * g * math.cos(q1) + M2 * g * L2 * math.cos(q1 + q2)
    gg2 = M2 * g * L2 * math.cos(q1 + q2)
    G = np.array(([gg1], [gg2]))
    return G


def model_simple(x):
    global upr_m1_list, upr_m2_list
    x = np.array(([x[0]], [x[1]], [x[2]], [x[3]]))
    q1, q2, dq1, dq2 = x[0], x[1], x[2], x[3]
    w = np.array(([dq1[0]], [dq2[0]]))
    C = Cor_Matrix(x)
    G = Grav_Matrix(x)
    M = Mass_Matrix(x)
    M_inv = np.linalg.inv(M)

    k1c = K1[0] * (q1 - q_d1) + K1[1] * (q2 - q_d2) + K1[2] * dq1 + K1[3] * dq2
    k2c = K2[0] * (q1 - q_d1) + K2[1] * (q2 - q_d2) + K2[2] * dq1 + K2[3] * dq2
    U = (-1) * np.array(([k1c[0]], [k2c[0]]))
    UPR = (M @ U) + C + G + w
    upr_m1_list = np.append(upr_m1_list, UPR[0])
    upr_m2_list = np.append(upr_m2_list, UPR[1])

    expr = UPR - C - G - w

    ddq = np.dot(M_inv, expr)
    dx = np.concatenate((w, ddq))
    dx = dx.reshape(1, 4)
    return dx[0]


def euler(t, func, q_start):
    N = np.size(t) - 1
    pos = q_start
    pos_m = np.array(pos)
    for i in range(N):
        df = func(pos)
        pos[2] = pos[2] + h * df[2]
        pos[0] = pos[0] + h * pos[2]
        pos[3] = pos[3] + h * df[3]
        pos[1] = pos[1] + h * pos[3]
        pos_m = np.vstack((pos_m, pos))
    return pos_m


el_sol = euler(TM, model_simple, q_0)
em1 = upr_m1_list[-1]
em2 = upr_m2_list[-1]
upr_m1_list = np.append(upr_m1_list, em1)
upr_m2_list = np.append(upr_m2_list, em2)

joint_id = [1, 3]


def pendulum_sim(the0):
    t = 0
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetPositions=the0,
                                controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0],
                                controlMode=p.VELOCITY_CONTROL, forces=[0.0, 0.0])
    PLIST = []
    global upr_s1_list, upr_s2_list

    for i in range(0, T):
        j1 = p.getJointStates(boxId, jointIndices=joint_id)[0]
        j2 = p.getJointStates(boxId, jointIndices=joint_id)[1]
        j_pos1, j_pos2 = j1[0], j2[0]
        j_vel1, j_vel2 = j1[1], j2[1]
        vec = [j_pos1, j_pos2, j_vel1, j_vel2]
        PLIST.append(vec)
        C = Cor_Matrixforsim(vec)
        G = Grav_Matrix(vec)
        M = Mass_Matrix(vec)
        w = np.array(([j_vel1], [j_vel2]))

        k1d = K1[0] * (j_pos1 - q_d1) + K1[1] * (j_pos2 - q_d2) + K1[2] * j_vel1 + K1[3] * j_vel2
        k2d = K2[0] * (j_pos1 - q_d1) + K2[1] * (j_pos2 - q_d2) + K2[2] * j_vel1 + K2[3] * j_vel2
        U = (-1) * np.array(([k1d], [k2d]))
        trq = (M @ U) + C + G + w

        tr1 = trq[0][0]
        tr2 = trq[1][0]
        torques = [tr1, tr2]
        p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0],
                                    controlMode=p.TORQUE_CONTROL, forces=torques)
        upr_s1_list = np.append(upr_s1_list, tr1)
        upr_s2_list = np.append(upr_s2_list, tr2)

        p.stepSimulation()
        t += dt

    pos_list = np.stack(PLIST, axis=0)
    return pos_list


sol_sim = pendulum_sim(the_0)

p.disconnect()
t1 = np.linspace(0, 1200 * 1 / 240, 1200)
t2 = np.linspace(0, 5)
qd1 = np.full(50, q_d1)
qd2 = np.full(50, q_d2)

fig1 = plt.figure("Графики решений")
ax1 = fig1.add_subplot(321)
ax1.set_ylabel('θ')
ax1.plot(t2, qd1, color='k', linestyle=':', label='θ_d1')
ax1.plot(t1, sol_sim[:, 0], label='simulator')
ax1.plot(t1, el_sol[:, 0], label='model', color='k', linestyle=':', linewidth=2)
ax1.legend()
ax1.grid()
ax2 = fig1.add_subplot(322)
ax2.set_ylabel('θ')
ax2.plot(t2, qd2, color='k', linestyle=':', label='θ_d2')
ax2.plot(t1, sol_sim[:, 1], label='simulator')
ax2.plot(t1, el_sol[:, 1], label='model', color='k', linestyle=':', linewidth=2)
ax2.legend()
ax2.grid()
ax1.title.set_text('1 звено:')
ax2.title.set_text('2 звено:')
ax3 = fig1.add_subplot(323)
ax3.set_ylabel("θ'")
ax3.plot(t1, sol_sim[:, 2])
ax3.plot(t1, el_sol[:, 2], color='k', linestyle=':', linewidth=2)
ax3.grid()
ax4 = fig1.add_subplot(324)
ax4.set_ylabel("θ'")
ax4.plot(t1, sol_sim[:, 3])
ax4.plot(t1, el_sol[:, 3], color='k', linestyle=':', linewidth=2)
ax4.grid()
ax5 = fig1.add_subplot(325)
ax5.set_xlabel('t')
ax5.set_ylabel('u')
ax5.plot(t1, upr_s1_list)
ax5.plot(t1, upr_m1_list, color='k', linestyle=':', linewidth=2)
ax5.grid()
ax6 = fig1.add_subplot(326)
ax6.set_xlabel('t')
ax6.set_ylabel('u')
ax6.plot(t1, upr_s2_list)
ax6.plot(t1, upr_m2_list, color='k', linestyle=':', linewidth=2)
ax6.grid()
plt.suptitle('Желаемые значения {} для первого и {} для второго звена'.format(round(q_d1, 5), round(q_d2, 5)))
plt.show()
