#!/usr/bin/env python
import glob
import json
import sys

for f in sorted(glob.glob("logs/*/*")):
  for log in open(f+"/log.json").readlines():
    joints = json.loads(log)['info']['joints']
    print "\t".join(map(str, joints))

