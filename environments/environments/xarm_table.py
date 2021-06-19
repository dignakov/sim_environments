import os
import sys
import pathlib

from environments.environment_base import EnvironmentBase, p, gym
import numpy as np

class Environment(EnvironmentBase):
    def __init__(self, viz_type="GUI", hz=300, mode=3):
        super().__init__(viz_type, hz, mode)
        self.robot = None
        self.table = None
        self.plane = None
        self.targets = []


    def populate_world(self):
        print("Populating XARM world")

        flags = p.URDF_INITIALIZE_SAT_FEATURES

        table_offset = [0, 0, -0.625]
        tray_offset = [0.37, 0, 0]
        arm_orientation = [0, 0, 0.7071068, -0.7071068]

        self.tray = p.loadURDF("tray/traybox.urdf", tray_offset, flags=flags, useFixedBase=True)
        self.plane = p.loadURDF("plane.urdf", table_offset, flags = flags, useFixedBase=True)
        self.table = p.loadURDF("table/table.urdf", table_offset, flags = flags, useFixedBase=True)
        self.robot = p.loadURDF("xarm/xarm6_robot.urdf", baseOrientation=arm_orientation, flags = flags, useFixedBase=True)

        # add objects
        filepath = os.path.join(pathlib.Path(__file__).parent.absolute(),"assets/diet_coke_can/diet_coke_can.urdf")
        print("loading target: ", filepath)
        self.targets = []
        for k in np.arange(0.3,0.5,0.1):
            for i in np.arange(0.3, 0.6, 0.1):  # 0.3 -- 0.6
                for j in np.arange(-0.2,0.2,0.1):  # -0.2 -- 0.2
                    orientation = 2*np.random.random_sample(4)-1
                    orientation = orientation/np.linalg.norm(orientation)
                    print(np.linalg.norm(orientation))
                    p.loadURDF(filepath, [i, j, k], baseOrientation=orientation, globalScaling=0.1)
                # k = k+0.1
            
        print("initializing sim")
        init_sim = 1*self.hz
        for t in range(init_sim):
            p.stepSimulation()
        print("sim ready")

        num_all_joints = p.getNumJoints(self.robot)  # all joints, not just active ones
        all_joints = [p.getJointInfo(self.robot, i) for i in range(num_all_joints)]
        self.joints = [j[0] for j in all_joints if j[2] == p.JOINT_REVOLUTE]
        self.num_joints = len(self.joints)

        # link_states = p.getLinkStates(self.robot)
        # print(link_states)

        print("active robot joints: ", self.joints)
        print("all joints:")
        for j in all_joints:
            print(j)

        # TODO: fix this once a gripper is added
        self.ee_flange = 6
        self.ee_tool = 6

        if self.mode == 1:
            pass
        elif self.mode == 2:
            pass
        elif self.mode == 3:
            pass
        else:
            pass

        return True
        
    def step(self, action=None):

        # command joints
        if action is not None:
            self.command_joints(action)

        p.stepSimulation()

        pos, vel, acc = self.get_joint_states()

        done = False
        reward = 0
        info = "derived"
        obs = {'pos': pos,
               'vel': vel,
               'acc': acc,
               'color': [],
               'depth': []}

        return obs, reward, done, info

    def render(self):
        print("xarm env render()")

    def render_camera(self, omega, time_now):

        cam_target = [0.5,0,0]
        cam_pos = [0.5,0,1]
        cam_up = [1,0,0]
        view_matrix = p.computeViewMatrix(cam_pos, cam_target, cam_up)
        
        img_arr = p.getCameraImage(self.camera_sens.width, self.camera_sens.height,
                                    view_matrix,
                                    self.camera_sens.projection_matrix,
                                    renderer=p.ER_BULLET_HARDWARE_OPENGL)
        w = img_arr[0]  #width of the image, in pixels
        h = img_arr[1]  #height of the image, in pixels
        rgba = img_arr[2]  #color data RGBA
        ogl_z = img_arr[3]  #depth data

        ret, metric_z = self.camera_sens.compute_metric_z(ogl_z)

        return rgba, metric_z, ogl_z