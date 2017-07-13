import pybullet as p
import t
import time
import numpy as np

p.connect(p.GUI)

def load_urdf(urdf, position, orientation=(0,0,0,1)):
  return p.loadURDF(urdf, *(position+orientation))
table = load_urdf("data/table/table.urdf", position=(0.5, 0, -0.82))
target = load_urdf("target.urdf", position=(0.5, 0, 0))
kuka = t.Kuka()

p.setGravity(0,0,-9.81)
def step(n, delay=0.0):
  for _ in range(n):
    p.stepSimulation()
    if delay != 0:
      time.sleep(delay)
step(100)

import math
kuka.joint_space_move([0,0.5,0,-1.5,0,0.8,0])
kuka.cartesian_move((0.5, 0., 0.3), p.getQuaternionFromEuler([0,-math.pi,0]), steps=200, delay=0.01)
#time.sleep(2)

#kuka.cartesian_move((0.5, 0., 0.3), p.getQuaternionFromEuler([0,-math.pi,math.pi/2]), steps=200, delay=0.01)
#kuka.cartesian_move((0.5, 0., 0.3), p.getQuaternionFromEuler([0,-math.pi,0]), steps=200, delay=0.01)
#time.sleep(2)

#def axis_at(x, y, z):
#  p.removeAllUserDebugItems()
#  p.addUserDebugLine((-10,y,z), (10,y,z), (1,1,1))
#  p.addUserDebugLine((x,-10,z), (x,10,z), (1,1,1))
#  p.addUserDebugLine((x,y,-10), (x,y,10), (1,1,1))



