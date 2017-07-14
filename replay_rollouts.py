#!/usr/bin/env python

# replay rollouts. using last state as "goal" state and regenerating entire episode

from collections import defaultdict
import argparse
import grasp_env
import json
import sys
import time
import util as u

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--run_in', type=str, default=None, help="run to read from")
parser.add_argument('--run_out', type=str, default=None, help="where to write replayed run to")
opts = parser.parse_args()
print >>sys.stderr, "OPTS", opts

# slurp entire run log in
# TODO: port to protos later
episodes = defaultdict(list)  # {e_id: [steps, steps, ...], ... }
final_reward = {}             # {e_id: final_reward, ... }
for line in open("logs/%s/log.json" % opts.run_in):
  record = json.loads(line)
  e = record['e']
  episodes[e].append(record)
  final_reward[e] = record['reward']
  print "line", line

# replay each episode where we had -1 reward but replace target
# with block in position based on what we actually acheived.
env = grasp_env.GraspEnv(gui=False)
log = u.Log(opts.run_out)
for episode in final_reward.keys():
  # ignore episode if it was a good one.
  if final_reward[episode] == 1:
    continue
 
  # based on final arm position derive a target position that would have meant
  # the episode would be successful.
  env.reset_joint_positions(episodes[episode][-1]['info']['joints'])
  fake_goal_pos, fake_goal_orientation = env.kuka.tip_pos_orientation(additional_offset=[0, 0, 0.03])

  # reset env (arm & target) and then explicitly _re_ reset target position.
  env.reset()
  env.reset_target(fake_goal_pos, fake_goal_orientation)
  
  # replay episode
  for step, replay_info in enumerate(episodes[episode]):
    action = replay_info['action']
    state, reward, done, info = env.step(action)
    log.append(episode, step, state, action, reward, info)
    print "e", episode, "s", step
    print "target_joints", replay_info['info']['joints']
    print "actual_joints", info['joints']
    if done: break
  #assert reward == 1





