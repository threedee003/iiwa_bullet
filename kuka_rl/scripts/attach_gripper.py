import pybullet as p
import time
import pybullet_data
import numpy as np


def spawn():
    
    p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    planeId = p.loadURDF("plane.urdf")
    p.setGravity(0,0,-9.8)
    iiwaId = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
    gripperId = p.loadURDF("wsg50-ros-pkg-master/wsg_50_simulation/urdf/wsg_50.urdf", [0,0,0])
    iiwa_eef_idx = 6
    eef_position, eef_orientation = p.getLinkState(iiwaId, iiwa_eef_idx)[:2]
    p.resetBasePositionAndOrientation(gripperId, eef_position, eef_orientation)
    gripper_constraint = p.createConstraint(
        parentBodyUniqueId = iiwaId,
        parentLinkIndex = iiwa_eef_idx,
        childBodyUniqueId = gripperId,
        childLinkIndex = -1,
        jointType = p.JOINT_FIXED,
        jointAxis = [0,0,0],
        parentFramePosition = [0,0,0],
        childFramePosition = [0,0,0]
    )
    initial_jt = 0*np.ones(7)

    for joint_idx in range(p.getNumJoints(iiwaId)):
        p.resetJointState(iiwaId, joint_idx, initial_jt[joint_idx])

    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()




def spawnSDF():
    p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0,0,-9.8)
    loaded_objects = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")
    robot_id = loaded_objects[0]

    for i in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()





if __name__ == '__main__':
    spawn()
