#!/usr/bin/env python3

import os
import tempfile
import time
import pathlib

import numpy as np

import gym
import pybullet as p
import pybullet_data as pd
import pkgutil

from utilities.pybullet_camera import PybulletCamera as Camera

class EnvironmentBase(gym.Env):

    def __init__(self, viz_type, hz, mode):
        self.viz = viz_type
        self.hz = hz
        self.sim_dt = 1.0/self.hz

        self.camera_viz = Camera(800, 600, 400, 400, fovy=60)
        self.camera_sens = Camera(800, 800, 400, 400, fovy=60)

        filepath = pathlib.Path(__file__).parent.absolute()
        self.assets_path = os.path.join(filepath, "assets")

        if(self.viz=="GUI"):
            print("GUI MODE")
            self.phys_id = p.connect(p.GUI)
        else:
            print("DIRECT/EGL MODE")
            self.phys_id = p.connect(p.DIRECT)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        file_io = p.loadPlugin('fileIOPlugin', physicsClientId=self.phys_id)
        if file_io < 0:
            raise RuntimeError('pybullet: cannot load FileIO plugin')
        if file_io >= 0:
            p.executePluginCommand(
                file_io,
                textArgument=self.assets_path,
                intArgs=[p.AddFileIOAction],
                physicsClientId=self.phys_id
            )
        p.setAdditionalSearchPath(pd.getDataPath())

        print("package path: ", pd.getDataPath())
        print("assets path: ", self.assets_path)
        print("file path: ", pathlib.Path(__file__).parent.absolute())
        # print("temp path: ", tempfile.gettempdir())

        if(self.viz=="EGL"):
            print("DIRECT --> EGL")
            egl = pkgutil.get_loader('eglRenderer')
            self.plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
            print("plugin=", self.plugin)

        

        self.robot = None
        self.joints = []
        self.num_joints = 0
        self.mode = mode # 3==position, 2==velocity, 1==effort

        print("Base Init Finished")

    def populate_world(self):  # FILL THIS IN IN  A SPECIFIC SIMULATION
        print("UNIMPLEMENTED")
        return False

    def configure_robot(self):
        if self.mode == 1:
            p.setJointMotorControlArray(
                bodyIndex=self.robot,
                jointIndices=self.joints,
                controlMode=p.VELOCITY_CONTROL,
                forces=0.1*np.ones(self.num_joints)
            )
        elif self.mode == 2:
            pass
        elif self.mode == 3:
            pass
        else:
            pass


    def command_joints(self, cmd):
        if self.mode == 1:  # effort
            p.setJointMotorControlArray(
                bodyIndex=self.robot,
                jointIndices=self.joints,
                controlMode=p.TORQUE_CONTROL,
                forces=cmd
            )
        elif self.mode == 2:  # velocity
            gains = np.ones(len(self.joints))
            p.setJointMotorControlArray(
                bodyIndex=self.robot,
                jointIndices=self.joints,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=cmd,
                velocityGains=gains
            )
        elif self.mode == 3:  # position
            gains = np.ones(len(self.joints))
            p.setJointMotorControlArray(
                bodyIndex=self.robot,
                jointIndices=self.joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=cmd,
                positionGains=gains
            )
            return True
        else:
            return False

    def get_joint_states(self):
        states = p.getJointStates(self.robot, self.joints)
        pos = np.zeros(self.num_joints)
        vel = np.zeros(self.num_joints)
        for i in range(self.num_joints):
            pos[i] = states[i][0]
            vel[i] = states[i][1]
        return pos, vel, np.zeros(self.num_joints) 

    def reset(self):
        p.resetSimulation()
        p.setTimeStep(self.sim_dt)
        p.setGravity(0, 0, -9.81)
        if not self.populate_world():
            return False
        self.configure_robot()
        return self.step()

    def step(self, action=None):
        p.stepSimulation()
        done = False
        reward = 0
        info = "base"
        obs = {'pos': [],
               'vel': [],
               'acc': [],
               'color': [],
               'depth': []}
        return obs, reward, done, info

    def render(self):
        print("UNIMPLEMENTED")

    def close(self):
        if(self.viz=="EGL"):
            p.unloadPlugin(self.plugin)