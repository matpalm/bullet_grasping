#!/usr/bin/env python

import argparse
import json
import grasp_env
import numpy as np
import random
import sys
import util as u

# kuka joints
# 0 - kuka joints j0
# ...
# 6 - kuka join j6, end effector rotation
# 7 - also (?) end effector rotation
# 8 - right finger upper; 0 to shut, -0.2 to open
# 9 - ?
# 10 - right finger lower; +0.1 to shut, -0.2 to open
# 11 - left finger upper; 0 to shut, +0.2 to open
# 12 - ?
# 13 - left finger lower; -0.1 to shut, -0.2 to open

class RandomAgent(object):
  def __init__(self, _env):
    pass

  def rollout(self, env, e):
    state = env.reset()
    step = 0
    done = False
    while not done:
      state, reward, done, info = env.step(env.action_space.sample())
      u.dump_state_as_img(state, "random", e, step)
      print "\t".join(map(str, ["random", e, step, reward, json.dumps(info)]))
      step += 1

class GreedyAgent(object):
  def __init__(self, env):
    self.a_idx = 0  # which action idx last changed
    self.actions = np.zeros(7)

  def pick_new_random_action(self):
    self.actions[self.a_idx] = 0
    self.a_idx = random.randrange(0, 7)
    self.actions[self.a_idx] = -1 if random.random() <= 0.5 else 1

  def rollout(self, env, e):
    env.reset()
    # do nothing once just to bootstrap value for "best distance"
    # note: reward is -1 / 1 and we ignore it and pull distance from info
    _state, _reward, done, info = env.step(np.zeros(7))
    # TODO: cornerish case of already being "done" at reset....
    best_distance = info['distance']
    # pick a random action idx to start with
    self.pick_new_random_action()

    step = 0
    while True:
      # try action
      state, reward, done, info = env.step(self.actions)
      distance = info['distance']
      u.dump_state_as_img(state, "greedy", e, step)
      print "\t".join(map(str, ["greedy", e, step, reward, json.dumps(info)]))
      if done: return
      step += 1

      if distance < best_distance - 1e-3:
        # if distance is better than best_distance we keep this action
        # ( add margin of 1e-2 since joint limits may mean we make
        # very small progress but are effectively stuck )
        best_distance = distance
      else:
        # rollback latest step by taking a step back in opposite
        # direction before deciding new action for next time
        self.actions[self.a_idx] *= -1
        state, reward, done, info = env.step(self.actions)
        distance = info['distance']
        u.dump_state_as_img(state, "greedy", e, step)
        # sanity check reversing actually _did_ get us back to best
        assert np.isclose(distance, best_distance, atol=0.1)
        print "\t".join(map(str, ["greedy", e, step, reward, json.dumps(info)]))
        if done: return
        step += 1
        # try something else next time
        self.pick_new_random_action()


if __name__ == "__main__":
  parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--agent', type=str, default="random", help="agent to run")
  parser.add_argument('--gui', action='store_true')
  parser.add_argument('--max_steps', type=int, default=50, help="env max steps")
  opts = parser.parse_args()
  print >>sys.stderr, "OPTS", opts

  env = grasp_env.GraspEnv(gui=opts.gui, max_steps=opts.max_steps)

  if opts.agent == "random":
    agent = RandomAgent(env)
  elif opts.agent == "greedy":
    agent = GreedyAgent(env)
  else:
    raise Exception("unknown agent type [%s]" % opts.agent)

  for e in range(10):
    agent.rollout(env, e)
