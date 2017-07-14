import kuka
import gym
from gym import spaces
import numpy as np
import pybullet as p
import util as u

TARGET_RANDOM_OFFSET_MU = 0.3
ACTION_SCALE = 0.2
SUCCESS_DISTANCE = 0.15
STEPS_PER_COMMAND = 10

class GraspEnv(gym.Env):

  def __init__(self, gui=False, max_steps=50):
    self.max_steps = max_steps
    p.connect(p.GUI if gui else p.DIRECT)
    def load_urdf(urdf, position, orientation=(0,0,0,1)):
      return p.loadURDF(urdf, *(position+orientation))
    self.table = load_urdf("data/table/table.urdf", position=(0.5, 0, -0.82))
    self.target = load_urdf("target.urdf", position=(0.5, 0, 0))
    self.kuka = kuka.Kuka()
    self.action_space = spaces.Box(-1, 1, shape=(7,))

  def reset_target(self, pos, orient):
    self.target_pos = pos
    self.target_orient = orient
    p.resetBasePositionAndOrientation(self.target, pos, orient)

  def reset_joint_positions(self, positions):
    self.arm_positions = positions
    self.kuka.reset_joint_positions(positions)

  def _reset(self):
    self.steps = 0
    # reset arm / gripper above table
    self.arm_positions = [ 0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539 ]
    self.kuka.reset_joint_positions(self.arm_positions)
    # reset target to random pose
    self.reset_target(pos=np.array([0.52, 0.0, 0.29]) + np.random.normal(scale=TARGET_RANDOM_OFFSET_MU, size=(3,)),
                      orient=np.random.uniform(size=(4,)))
    return self.state()

  def _reset_arm_positions_to_actual_positions(self):
    self.arm_positions = self.kuka.joint_positions()

  def state(self):
    return u.render_two_cameras()

  def tip_to_target_distance(self):
    tip_pos, _ = self.kuka.tip_pos_orientation()
    distance = np.linalg.norm(tip_pos - self.target_pos)
    return distance

  def info(self):
    return {
      'distance': self.tip_to_target_distance(),
      'target': {'pos': list(self.target_pos), 'orient': list(self.target_orient)},
      'joints': list(self.arm_positions)
    }

  # TODO: make last action value the gripper; 0.0->closed, 1.0->fully open
  def _step(self, actions):

    assert len(actions) == 7  # 7d values (-1, 1)
    if self.steps > self.max_steps:
      return self.state(), -1, True, {}

    # update commanded arm joint positions
    for i, action in enumerate(actions):
      self.arm_positions[i] += action * ACTION_SCALE
    self.kuka._set_commanded_joint_positions(self.arm_positions)
    # step simulation once
    # TODO: n times?
    for _ in range(STEPS_PER_COMMAND):
      p.stepSimulation()
    # reset arm_positions based on where we actually got to (e.g. because of joint constraints etc)
    self._reset_arm_positions_to_actual_positions()

    # reward is 0.0 except on last step where it's either -1.0 or 1.0
    distance = self.tip_to_target_distance()
    target_reached = distance <= SUCCESS_DISTANCE
    if (self.steps == self.max_steps) or target_reached:
      done = True
      reward = 1 if target_reached else -1
    else:
      done = False
      reward = 0

    self.steps += 1

    return self.state(), reward, done, self.info()
