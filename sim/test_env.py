#!/usr/bin/env python3

#import basics
import os
import sys
import pathlib
import time
import numpy as np
import cv2

#while not installing, just add to path
filepath = pathlib.Path(__file__).parent.absolute()
append_path = os.path.join(filepath,"../environments/")
sys.path.append(append_path)

#import environment
from environments.xarm_table import Environment

#import visualizer
from utilities.open3d_visualizer import Open3DVisualizer

#object detector

#create environment
env = Environment(mode=2, hz=100)  #1=effort, 2=vel, 3=pos
obs, reward, done, info = env.reset()

#create pcd visualizer
viz = Open3DVisualizer(width=env.camera_sens.width, height=env.camera_sens.height,
                        fx=env.camera_sens.fx, fy=env.camera_sens.fy,
                        cx=env.camera_sens.cx, cy=env.camera_sens.cy)
viz.start()


sim_time = 0.0
sim_step = 0
rgb = np.zeros((env.camera_sens.height, env.camera_sens.width))
bgr = np.zeros((env.camera_sens.height, env.camera_sens.width))
predictions_img = np.zeros((env.camera_sens.height, env.camera_sens.width))
predictions = {}
while True:
    
    sim_time += env.sim_dt
    
    action = [np.sin(sim_time), 0, -np.sin(sim_time), 0, 0, 0]
    # action = [10, 0, 0, 0, 0, 0]
    obs, reward, done, info = env.step(action)

    if(sim_step % 10 == 0):
        m, z, _ = env.render_camera(0.1, sim_time)
        rgb = cv2.cvtColor(m, cv2.COLOR_RGBA2RGB)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)  # for viz and inference
 
        vmin = 0.85
        vmax = 1.02
        img_z = np.clip((z.copy() - vmin)/(vmax-vmin), 0, 1)
        img_z = cv2.applyColorMap((img_z*255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)

        viz.reset()
        viz.add_pcd_img(rgb,z)



    cv2.imshow("img",bgr)
    cv2.imshow("depth",img_z)
    viz.step()
    if cv2.waitKey(1) in [27, 113]:  # esc or q
        # cv2.imwrite("test_img1.jpg", rgb)
        # cv2.imwrite("test_img.jpg", bgr)
        break  # esc to quit

    time.sleep(env.sim_dt/2)
    sim_step +=1

cv2.destroyAllWindows()
viz.stop()