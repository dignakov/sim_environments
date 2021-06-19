#!/usr/bin/env python3

import pybullet as p
import pybullet_data as pd
import pkgutil
egl = pkgutil.get_loader('eglRenderer')

import time
import math
import numpy as np
import cv2

import open3d as o3d
import open3d.visualization as vis


#parameters for later
flags = p.URDF_INITIALIZE_SAT_FEATURES
useFixedBase = True


# p.connect(p.GUI)
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pd.getDataPath())


# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/eglRenderTest.py
plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
print("plugin=", plugin)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# exit(0)

#init function (re-use?)
sim_dt = 0.01
p.resetSimulation()
p.setTimeStep(sim_dt)
p.setGravity(0, 0, -9.81)

plane_pos = [0, 0, -0.625]
table_pos = plane_pos
plane = p.loadURDF("plane.urdf", plane_pos, flags = flags, useFixedBase=useFixedBase)
table = p.loadURDF("table/table.urdf", table_pos, flags = flags, useFixedBase=useFixedBase)
robot = p.loadURDF("xarm/xarm6_robot.urdf", flags = flags, useFixedBase=useFixedBase)



numJoints = p.getNumJoints(robot)
print("robot joints: ", numJoints)


#3d visualization ##############################
vis = o3d.visualization.Visualizer()
vis.create_window()


x = np.linspace(-3, 3, 401)
mesh_x, mesh_y = np.meshgrid(x, x)
z = np.sinc((np.power(mesh_x, 2) + np.power(mesh_y, 2)))
z_norm = (z - z.min()) / (z.max() - z.min())
xyz = np.zeros((np.size(mesh_x), 3))
xyz[:, 0] = np.reshape(mesh_x, -1)
xyz[:, 1] = np.reshape(mesh_y, -1)
xyz[:, 2] = np.reshape(z_norm, -1)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)

vis.add_geometry(pcd)

vis.run()
exit(0)
