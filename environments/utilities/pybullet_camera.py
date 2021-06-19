#!/usr/bin/env python3

import numpy as np
import pybullet as p

class PybulletCamera:

    def __init__(self,
                width, height,
                cx=None, cy=None,
                fx=None, fy=None,
                fovy=None,
                near_plane=0.01, far_plane=100):

        self.ready = False
        self.width = width
        self.height = height
        self.near_plane = near_plane
        self.far_plane = far_plane

        if cx is not None and cy is not None:
            self.cx = cx
            self.cy = cy
        else:
            self.cx = width/2
            self.cy = height/2

        if fx is not None and fy is not None:
            self.fx = fx
            self.fy = fy
            self.fovy = 2*np.arctan2(height/2, fy)*180/np.pi
        elif fovy is not None:
            #fov takes presedence
            self.fovy = fovy
            self.fy = height/2/np.tan(fovy*np.pi/180/2)
            self.fx = self.fy  #assuming open3d does their config like acamera and not opengl... otherwise fix
            pass
        else:
            print("[ERROR] camera needs either (fx, fy) or fov")
            self.ready = False
            return

        #for pybullet rendering
        self.aspect = width/height
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fovy, 
                                                                self.aspect, 
                                                                self.near_plane, 
                                                                self.far_plane)

        print("[CAMERA] projection (GL):")
        print(self.projection_matrix)

        self.ready = True

    def compute_metric_z(self, dep):
        if not self.ready:
            return False, np.array([])
        metric_z = self.far_plane*self.near_plane/(self.far_plane - (self.far_plane-self.near_plane)*dep)
        return True, metric_z

        # img_z = (dep/np.amax(real_z)*254).astype(np.uint8)
        # img_z = dep - np.amin(dep)
        # img_z = metric_z - np.amin(metric_z)
        # img_z = (img_z/np.amax(img_z)*254).astype(np.uint8)