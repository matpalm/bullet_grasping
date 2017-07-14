#!/usr/bin/env python

# replay rollouts. using last state as "goal" state and regenerating entire episode

from collections import defaultdict
import argparse
import grasp_env
import json
import os
import sys
import time
import util as u
import numpy as np

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--run_in', type=str, default=None, help="run to read from")
parser.add_argument('--run_out', type=str, default=None, help="where to write replayed run to")
opts = parser.parse_args()
print >>sys.stderr, "OPTS", opts

# iterate over all episodes in run_in
env = grasp_env.GraspEnv(gui=False)
logger = u.Log(opts.run_out)

for episode in os.listdir("logs/"+opts.run_in):
  log = open("logs/%s/%s/log.json" % (opts.run_in, episode)).readlines()

  # should we ignore this episode in terms of replay since it was successful?
  last_log_entry = json.loads(log[-1])
  if last_log_entry['reward'] == 1:
    continue

  log = map(json.loads, log)
  episode_num = int(episode.replace("e", ""))  # TODO: hack difference in episode id vs 'e0000'

  # based on final arm position derive a target position that would have meant
  # the episode would be successful.
  env.reset_joint_positions(log[-1]['info']['joints'])
  fake_goal_pos, fake_goal_orientation = env.kuka.tip_pos_orientation(additional_offset=[0, 0, 0.03])

  # reset env (arm & target) and then explicitly _re_ reset target position.
  env.reset()
  env.reset_target(fake_goal_pos, fake_goal_orientation)

  # replay actions
  # note: with the goal in a different place there are cases where the replay won't be
  #  exactly the same..
  # 1) if the goal is placed in a position where the arm went to _multiple_ times during
  #    the original trajectory then the replayed episode will be shorter (since we'll
  #    satisfy the distance criteria on the first pass
  # 2) if the goal places the block in a place that stops the arm from moving the
  #    trajectories will diverge and it might not actually successfully satisfy the
  #    distance criteria.
  for step_num, record in enumerate(log):
    action = record['action']
    state, reward, done, info = env.step(action)
    logger.append(episode_num, step_num, state, action, reward, info)
    joint_diff = np.linalg.norm(np.array(record['info']['joints']) - np.array(info['joints']))
    print "episode", episode_num, "step", step_num, "joint_diff", joint_diff
    if done: break


