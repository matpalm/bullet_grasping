#!/usr/bin/env python
import glob
import json
import sys

for f in sorted(glob.glob("logs/*/*")):
  log = open(f+"/log.json").readlines()
  last_entry = json.loads(log[-1])
  print "\t".join(map(str, [f, len(log), last_entry['info']['distance'], last_entry['reward']]))
