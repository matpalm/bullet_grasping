#!/usr/bin/env bash
set -ex
./replay_rollouts.py --run_in=r1 --run_out=r1_r > r1r.out 2>&1 &
./replay_rollouts.py --run_in=r2 --run_out=r2_r > r2r.out 2>&1 &
./replay_rollouts.py --run_in=r3 --run_out=r3_r > r3r.out 2>&1 &
./replay_rollouts.py --run_in=r4 --run_out=r4_r > r4r.out 2>&1 &
./replay_rollouts.py --run_in=r5 --run_out=r5_r > r5r.out 2>&1 &
time wait