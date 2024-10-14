import pybullet as p
import time
import pybullet_data
import numpy as np






def spwan():
    p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    planeId = p.loadURDF("plane.urdf")
    p.setGravity(0,0,-9.8)
    iiwaId = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
    initial_jt = 2*np.ones(7)

    for joint_idx in range(p.getNumJoints(iiwaId)):
        p.resetJointState(iiwaId, joint_idx, initial_jt[joint_idx])

    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()



def joint_position_control(target_jt):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    p.setGravity(0,0,-9.8)
    iiwaId = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
    
    for jt_idx in range(p.getNumJoints(iiwaId)):
        p.setJointMotorControl2(
            bodyIndex = iiwaId,
            jointIndex = jt_idx,
            controlMode = p.POSITION_CONTROL,
            targetPosition = target_jt[jt_idx],
            force = 10
        )

    for i in range(100000):
        p.stepSimulation()
        time.sleep(1./60.)

    p.disconnect()





def torque_control():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    p.setGravity(0,0,-9.8)
    iiwaId = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

    for joint_idx in range(p.getNumJoints(iiwaId)):
        p.setJointMotorControl2(
            bodyIndex = iiwaId,
            jointIndex = joint_idx,
            controlMode = p.TORQUE_CONTROL,
            force = 10.0
        )

    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)




if __name__ == '__main__':
    final_jt = 2*np.ones(7)
    torque_control()
    # joint_position_control(final_jt)