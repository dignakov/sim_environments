#!/usr/bin/env python3

import open3d as o3d
import open3d.visualization as vis
import numpy as np

class Open3DVisualizer():

    vis = o3d.visualization.Visualizer()
    pcd = o3d.geometry.PointCloud()
    pcd_idx = 10*False
    geom_added = False

    def __init__(self, width, height, fx, fy, cx, cy, flip_pcd=True, name="pcd"):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.flip_pcd = flip_pcd
        self.name = name

    def start(self):
        self.vis.create_window(self.name)

    def stop(self):
        self.vis.destroy_window()

    def reset(self):
        self.pcd.clear()

    def add_pcd_img(self, image, depth):

        depth_scale = 1.0
        o3d_img = o3d.geometry.Image(image)
        o3d_dep = o3d.geometry.Image(depth)
        intrinsics = o3d.camera.PinholeCameraIntrinsic(width=self.width, height=self.height, fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_img, o3d_dep, depth_scale=depth_scale, convert_rgb_to_intensity=False)
        pcd_tmp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsics)

        if(self.flip_pcd):
            flip_transform = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            # camera_pose_transform = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            # transform = camera_pose_transform@flip_transform
            transform = flip_transform
            pcd_tmp.transform(transform)

        self.pcd += pcd_tmp

        if not self.geom_added:
            self.vis.add_geometry(self.pcd)
            self.geom_added = True

        self.vis.update_geometry(self.pcd)

    def step(self):
        self.vis.poll_events()
        self.vis.update_renderer()