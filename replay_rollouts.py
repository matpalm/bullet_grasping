#!/usr/bin/env python

# replay rollouts. using last state as "goal" state and regenerating entire episode

from collections import defaultdict
import grasp_env
import json
import time
import sys
import util as u

if __name__ == "__main__":
  # slurp entire thing in
  # TODO: port to protos later
  episodes = defaultdict(list)  # {e_id: [steps, steps, ...], ... }
  final_reward = {}             # {e_id: final_reward, ... }
  for line in sys.stdin:
    cols = line.strip().split("\t")
    assert len(cols) == 5
    _, e, s, reward, info = cols
    # assume in 's' order per 'e'
    e, reward, info = int(e), float(reward), json.loads(info)
    episodes[e].append(info)
    final_reward[e] = reward

  # replay each episode where we had -1 reward but replace target
  # with block in position based on what we actually acheived.
  env = grasp_env.GraspEnv(gui=False)
  for e in final_reward.keys():
    # ignore episode if it was a good one.
    if final_reward[e] == 1: continue
    # look at final arm position to decide what "fake" goal we use.
    env.reset_joint_positions(episodes[e][-1]['joints'])
    fake_goal_pos, fake_goal_orientation = env.kuka.tip_pos_orientation(additional_offset=[0, 0, 0.03])
    env.reset_target(fake_goal_pos, fake_goal_orientation)
    # replay episode
    for step, replay_info in enumerate(episodes[e]):
      # reset arm based on replay info
      env.kuka.reset_joint_positions(replay_info['joints'])
      # emit new record with newly calculated info
      info = env.info()
      done = info['distance'] < 0.10  # note: during normal env this is 0.15
      reward = 1 if done else 0
      print "\t".join(map(str, ["replay", e, step, reward, json.dumps(info)]))
      u.dump_state_as_img(env.state(), "replay", e, step)
      if done: break
    assert reward == 1





