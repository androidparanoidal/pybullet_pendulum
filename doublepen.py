import pybullet as p
import pybullet_data
from scipy.integrate import odeint
import numpy as np
import math
import matplotlib.pyplot as plt

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./double_pendulum.urdf", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

L1, L2 = 0.8, 0.7
M1, M2 = 2, 1.5
g = 9.81

mm1 = M1 * L1**2 + M2 * (L1**2 + 2*L1*L2 + L2**2)
mm2 = M2 * (L1*L2 + L2**2)
mm3 = mm2
mm4 = M2 * L2**2
Mass_Matrix = np.array(([mm1, mm2], [mm3, mm4]))

cv1 = -M2*L1*L2  # !
cv2 = M2*L1*L2  # !
Coriolis_vec = np.array(([cv1], [cv2]))

gv1 = (M1 + M2) * L1 * g + M2 * L2 * g
gv2 = M2 * L2 * g
G_vec = np.array(([gv1], [gv2]))

dt = 1 / 240
pi = math.pi
joint_id = [1, 3]
the0_1 = 0.1
the0_2 = 0.1
the_0 = [the0_1, the0_2]

t_list = []
positions_list = []
w_list = []


def double_pendulum_sim(the_0):
    t = 0
    # go to the starting position
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetPositions=the_0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    '''joint_states = p.getJointStates(boxId, jointIndices=joint_id)
    joint_positions = np.array([i[0] for i in joint_states])
    the_0 = joint_positions'''

    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.VELOCITY_CONTROL, forces=[0.0, 0.0])
    positions_list = []

    while t < 5:
        joint_states = p.getJointStates(boxId, jointIndices=joint_id)
        joint_positions = np.array([i[0] for i in joint_states])
        joint_velocities = np.array([j[1] for j in joint_states])
        positions_list.append(joint_positions)
        w_list.append(joint_velocities)

        torque = [2.0, 1.5]
        p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.TORQUE_CONTROL,
                                    forces=torque)

        p.stepSimulation()
        t_list.append(t)
        t += dt

    pos_list = np.stack(positions_list, axis=0)
    return pos_list

sol_sim_p = double_pendulum_sim(the_0)
vel_list = np.stack(w_list, axis=0)

p.disconnect()



i_c = [0.1, 0, 0.1, 0]  # initial conditions

T = int(5 / dt)
TM = [0] * T

def model(x, t, L1, L2, M1, M2):
    theta1, w1, theta2, w2 = x

    c, s = np.cos(theta1 - theta2), np.sin(theta1 - theta2)

    theta1dot = w1
    w1dot = (M2 * g * np.sin(theta2) * c - M2 * s * (L1 * w1**2 * c + L2 * w2**2) - (M1+M2) * g * np.sin(theta1)) / (L1 * (M1 + M2 * s**2))
    theta2dot = w2
    w2dot = ((M1+M2) * (L1 * w1**2 * s - g * np.sin(theta2) + g * np.sin(theta1) * c) + M2 * L2 * w2**2 * s * c) / (L2 * (M1 + M2 * s**2))
    return theta1dot, w1dot, theta2dot, w2dot

def energy(x):
    th1, th1d, th2, th2d = x.T
    PE = -(M1 + M2) * L1 * g * np.cos(th1) - M2 * L2 * g * np.cos(th2)
    KE = 0.5 * M1 * (L1 * th1d) ** 2 + 0.5 * M2 * ((L1 * th1d) ** 2 + (L2 * th2d) ** 2 + 2 * L1 * L2 * th1d * th2d * np.cos(th1 - th2))
    return KE + PE

t1 = np.linspace(0, 1200*1/240, 1200)
a = odeint(model, i_c, t1, args=(L1, L2, M1, M2))

an1 = a[:, 0]
vel1 = a[:, 1]
an2 = a[:, 2]
vel2 = a[:, 3]

p1 = L1 * np.sin(an1)
v1 = -L1 * np.cos(an1)
p2 = p1 + L2 * np.sin(vel1)
v2 = v1 - L2 * np.cos(vel1)


fig1 = plt.figure("Решение модели")
ax1 = fig1.add_subplot(321)
ax1.set_xlabel('t')
ax1.set_ylabel('θ(t)')
ax1.plot(t1, p1)
ax2 = fig1.add_subplot(322)
ax2.set_xlabel('t')
ax2.set_ylabel('θ(t)')
ax2.plot(t1, p2)
ax1.title.set_text('1 звено:')
ax2.title.set_text('2 звено:')
ax1.grid()
ax2.grid()
plt.show()

fig2 = plt.figure("Симуляторное решение")
ax1 = fig2.add_subplot(321)
ax1.set_xlabel('t')
ax1.set_ylabel('θ(t)')
ax1.plot(t_list, sol_sim_p[:, 0])
ax2 = fig2.add_subplot(322)
ax2.set_xlabel('t')
ax2.set_ylabel('θ(t)')
ax2.plot(t_list, sol_sim_p[:, 1])
ax3 = fig2.add_subplot(323)
ax3.set_xlabel('t')
ax3.set_ylabel("θ'(t)")
ax3.plot(t_list, vel_list[:, 0])
ax4 = fig2.add_subplot(324)
ax4.set_xlabel('t')
ax4.set_ylabel("θ'(t)")
ax4.plot(t_list, vel_list[:, 1])
ax5 = fig2.add_subplot(325)
ax5.set_xlabel('t')
ax5.set_ylabel('u')
ax6 = fig2.add_subplot(326)
ax6.set_xlabel('t')
ax6.set_ylabel('u')
ax1.title.set_text('1 звено:')
ax2.title.set_text('2 звено:')
ax1.grid()
ax2.grid()
ax3.grid()
ax4.grid()
plt.show()
