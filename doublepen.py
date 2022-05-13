import pybullet as p
import pybullet_data
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

dt = 1 / 240
pi = math.pi
joint_id = [1, 3]
the0_1 = 0.1
the0_2 = 0.1
the_0 = [the0_1, the0_2]
T = int(5 / dt)
TM = [0] * T
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

        #torque = [2.0, 1.5]
        #p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.TORQUE_CONTROL, forces=torque)

        p.stepSimulation()
        t_list.append(t)
        t += dt

    pos_list = np.stack(positions_list, axis=0)
    return pos_list

sol_sim_p = double_pendulum_sim(the_0)
print(sol_sim_p)
vel_list = np.stack(w_list, axis=0)

p.disconnect()




t1 = np.linspace(0, 1200*1/240, 1200)


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
ax5.grid()
ax6.grid()
plt.show()
