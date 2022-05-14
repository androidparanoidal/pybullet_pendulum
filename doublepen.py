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
T = int(10 / dt)
TM = [0] * T


def double_pendulum_sim(the0):
    t = 0
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetPositions=the0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.VELOCITY_CONTROL, forces=[0.0, 0.0])
    PLIST = []

    for i in range(0, T):
        joint_states = p.getJointStates(boxId, jointIndices=joint_id)
        joint_positions = np.array([i[0] for i in joint_states])
        joint_velocities = np.array([j[1] for j in joint_states])
        tl = np.concatenate((joint_positions, joint_velocities))
        print('tl:', tl)
        jp = p.getJointStates(boxId, jointIndices=joint_id)[0]
        jv = p.getJointStates(boxId, jointIndices=joint_id)[1]
        print(jp)
        print(jp[0]) #позиция первого звена
        print(jp[1]) #скорость первого
        print(jv)
        print(jv[0]) #позиция второго
        print(jv[1], '\n')

        #print(i)
        #print(tl)
        PLIST.append(tl)


        #torque = [2.0, 1.5]
        #p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.TORQUE_CONTROL, forces=torque)

        p.stepSimulation()
        t += dt

    pos_list = np.stack(PLIST, axis=0)
    return pos_list


sol_sim = double_pendulum_sim(the_0)
#print(sol_sim)



p.disconnect()

t1 = np.linspace(0, 2400*1/240, 2400)
fig2 = plt.figure("Симуляторное решение")
ax1 = fig2.add_subplot(321)
ax1.set_ylabel('θ(t)')
ax1.plot(t1, sol_sim[:, 0])
ax2 = fig2.add_subplot(322)
ax2.set_ylabel('θ(t)')
ax2.plot(t1, sol_sim[:, 1])
ax3 = fig2.add_subplot(323)
ax3.set_ylabel("θ'(t)")
ax3.plot(t1, sol_sim[:, 2])
ax4 = fig2.add_subplot(324)
ax4.set_ylabel("θ'(t)")
ax4.plot(t1, sol_sim[:, 3])
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
