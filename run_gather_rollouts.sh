#!/usr/bin/env bash
set -ex
./gather_rollouts.py --run r1 --agent greedy --max_steps=50 --num_episodes=10 >r1.out 2>&1 &
./gather_rollouts.py --run r2 --agent greedy --max_steps=50 --num_episodes=10 >r2.out 2>&1 &
./gather_rollouts.py --run r3 --agent greedy --max_steps=50 --num_episodes=10 >r3.out 2>&1 &
./gather_rollouts.py --run r4 --agent greedy --max_steps=50 --num_episodes=10 >r4.out 2>&1 &
./gather_rollouts.py --run r5 --agent greedy --max_steps=50 --num_episodes=10 >r5.out 2>&1 &
time wait