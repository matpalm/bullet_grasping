import numpy as np
import pybullet as p

EE_IDX = 6
EE_TIP_OFFSET = np.array([0, 0.02, 0.27])

class Kuka(object):
  def __init__(self):
    self.id = p.loadSDF("data/kuka_iiwa/kuka_with_gripper2.sdf")[0]
    # add frame axis for middle of end effector
    if False:
      p.addUserDebugLine([0,0.02,0.26], [0.1,0.02,0.26], [1,0,0],
                          parentObjectUniqueId=self.id, parentLinkIndex=EE_IDX)
      p.addUserDebugLine([0,0.02,0.26], [0,0.12,0.26], [0,1,0],
                          parentObjectUniqueId=self.id, parentLinkIndex=EE_IDX)
      p.addUserDebugLine([0,0.02,0.26], [0,0.02,0.36], [0,0,1],
                          parentObjectUniqueId=self.id, parentLinkIndex=EE_IDX)
    self._set_commanded_gripper_aperture(-0.1)

  # --- getters

  def joint_positions(self):
    return [p.getJointState(self.id, i)[0] for i in range(7)]

  def tip_pos_orientation(self, additional_offset=[0,0,0]):
    # return position, orientation of tip of gripper which is offset
    # from the position of the end effector (link 6). tip orientation
    # is the same as the ee orientation. add an additional offset if
    # required e.g. [0, 0, 0.1] to represent a point slightly past gripper
    ee_state = p.getLinkState(self.id, 6)
    ee_pos, ee_orient = ee_state[0], ee_state[1]
    rotation_matrix = np.array(p.getMatrixFromQuaternion(ee_orient)).reshape((3, 3))
    world_space_offset = rotation_matrix.dot(EE_TIP_OFFSET + np.array(additional_offset))
    tip_pos = ee_pos + world_space_offset
    return tip_pos, ee_orient

  # --- setters

  def joint_space_move(self, joint_positions, steps=100, delay=0):
    self._set_commanded_joint_positions(joint_positions)
    self._step(steps, delay)

  def cartesian_move(self, ee_position, ee_orientation, steps=1000, delay=0):
    ee_joint_idx = 6
    # we want to strongly favor end effector rotations over joint moves
    joint_damping = [1,1,1,1,1,1,0.001,0.001,0.1,0.1,0.1,0.1,0.1,0.1]
    for _ in range(steps):
      positions = p.calculateInverseKinematics(self.id, ee_joint_idx, ee_position, ee_orientation)
      self._set_commanded_joint_positions(positions[:7])
      p.stepSimulation()
      if delay != 0: time.sleep(delay)

  def open_gripper(self, steps=100, delay=0):
    self._set_commanded_gripper_aperture(0.1)
    self._step(steps, delay)

  def close_gripper(self, steps=100, delay=0):
    self._set_commanded_gripper_aperture(-0.1)
    self._step(steps, delay)

  def reset_joint_positions(self, positions):
    for i, position in enumerate(positions):
      p.resetJointState(self.id, i, position)

  # --- internals

  def _step(self, steps, delay):
    # TODO: introduce fn() so can use elsewhere
    for _ in range(steps):
      p.stepSimulation()
      if delay != 0: time.sleep(delay)


  def _set_commanded_joint_positions(self, positions):
    for i, position in enumerate(positions):
      p.setJointMotorControl2(self.id, i, p.POSITION_CONTROL, targetPosition=positions[i],
                              targetVelocity=0.0, force=200, positionGain=0.03, velocityGain=1.0)

  def _set_commanded_gripper_aperture(self, aperture):
    p.setJointMotorControl2(self.id, 8, p.POSITION_CONTROL, targetPosition=-aperture, force=6.0)
    p.setJointMotorControl2(self.id, 11, p.POSITION_CONTROL, targetPosition=aperture, force=6.5)
    p.setJointMotorControl2(self.id, 10, p.POSITION_CONTROL, targetPosition=0, force=8.0)
    p.setJointMotorControl2(self.id, 13, p.POSITION_CONTROL, targetPosition=0, force=8.0)
