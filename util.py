from PIL import Image
import numpy as np
import pybullet as p
import os

W = 160
H = 120

def axis_at(x, y, z):
  # draw x/y/z grid lines crossing at x,y,z
  p.removeAllUserDebugItems()
  p.addUserDebugLine((-10,y,z), (10,y,z), (1,1,1))
  p.addUserDebugLine((x,-10,z), (x,10,z), (1,1,1))
  p.addUserDebugLine((x,y,-10), (x,y,10), (1,1,1))

def render_camera(yaw, pitch):
  projMat = p.computeProjectionMatrixFOV(fov=70, aspect=(float(W)/H),
                                         nearVal=0.01, farVal=100)
  viewMat = p.computeViewMatrixFromYawPitchRoll(
    cameraTargetPosition=(0.5, 0.0, 0.1),
    distance=1.25, yaw=yaw, pitch=pitch, roll=0, upAxisIndex=2)
  render = p.getCameraImage(width=W, height=H,
                            viewMatrix=viewMat, projectionMatrix=projMat,
                            shadow=True)
  rgba = np.array(render[2], dtype=np.uint8).reshape((H, W, 4))
  rgb = rgba[:,:,:3]
  return rgb

def render_two_cameras():
  # (2, H, W, 3)
  return np.stack([render_camera(yaw=149, pitch=-53),
                   render_camera(yaw=30, pitch=-42)])

def dump_state_as_img(state, run, episode, step, base_dir="logs"):
  N, H, W, C = state.shape
  assert C == 3
  composite = Image.new('RGB', (N*W, H), (0,0,0))
  for i in range(N):
    composite.paste(Image.fromarray(state[i]), (i*W, 0))
  directory = "%s/%s/%03d/" % (base_dir, run, episode)
  if not os.path.exists(directory):
    os.makedirs(directory)
  composite.save("%s/%03d.png" % (directory, step))
