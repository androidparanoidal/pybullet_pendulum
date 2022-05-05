import pybullet as p
import pybullet_data
import numpy as np
import math
'''import pylab
import time '''

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
boxId = p.loadURDF("./double_pendulum.urdf", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

dt = 1 / 240
pi = math.pi
dof = 2
joint_id = range(dof)
the0_1 = 0.1
the0_2 = 0.1
the_0 = [the0_1, the0_2]

time_list = []
positions_list = []

def double_pendulum(the_0):
    t = 0
    # go to the starting position
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetPositions=the_0, controlMode=p.POSITION_CONTROL)
    for _ in range(1000):
        p.stepSimulation()

    joint_states = p.getJointStates(boxId, jointIndices=joint_id)
    joint_positions = np.array([i[0] for i in joint_states])
    the_0 = joint_positions

    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=joint_id, targetVelocities=[0.0, 0.0], controlMode=p.VELOCITY_CONTROL, forces=[0.0, 0.0])
    positions_list = []

    while t < 5:
        joint_states = p.getJointStates(boxId, jointIndices=joint_id)
        joint_positions = np.array([i[0] for i in joint_states])
        positions_list.append(joint_positions)
        p.stepSimulation()
        time_list.append(t)
        t += dt
        # time.sleep(dt)

    # print(positions_list)
    return positions_list

sol = double_pendulum(the_0)

t = np.linspace(0, 5, 5*240)



p.disconnect()

'''L1, L2 = 0.8, 0.7
M1, M2 = 2, 1.5
g = 9.81
pi = math.pi

i_c = [0.1, 0, 0.1, 0]  # initial conditions


T = int(5 / dt)
TM = [0] * T


def double_pendulum_sim():


    return 0
'''
