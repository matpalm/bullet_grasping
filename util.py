from PIL import Image
import json
import numpy as np
import os
import pybullet as p

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

def dump_state_as_img(state, base_dir):
  # TODO: fold into log_step? not used elsewhere...
  N, H, W, C = state.shape
  assert C == 3
  composite = Image.new('RGB', (N*W, H), (0,0,0))
  for i in range(N):
    composite.paste(Image.fromarray(state[i]), (i*W, 0))
  directory = "%s/%s/%03d/" % (base_dir, run, episode)
  composite.save("%s/%03d.png" % (directory, step))

class Log(object):
  def __init__(self, directory):
    self.directory = "logs/" + directory
    if not os.path.exists(self.directory):
      os.makedirs(self.directory)
    self.episode_log = open(self.directory + "/log.json", "w")
    
  def append(self, episode, step, state, action, reward, info):
    episode_directory = "%s/e%04d/" % (self.directory, episode)
    if not os.path.exists(episode_directory):
      os.makedirs(episode_directory)
    Image.fromarray(state[0]).save("%s/0_%03d.png" % (episode_directory, step))
    Image.fromarray(state[1]).save("%s/1_%03d.png" % (episode_directory, step))
    record = json.dumps({"e": episode, "s": step, "action": list(action),
                         "reward": reward, "info": info})
#    print record
    self.episode_log.write("%s\n" % record)
    self.episode_log.flush()
