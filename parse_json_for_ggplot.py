#!/usr/bin/env python
import json
import sys

for line in sys.stdin:
  if line.startswith("pybullet"): continue
  cols = line.split("\t")
  assert len(cols) == 5
  run, episode, step, _reward, info = cols
  info = json.loads(info)
  print "\t".join(map(str, [run, episode, step, info['distance']]))
