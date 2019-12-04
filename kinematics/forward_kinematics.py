'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

# i don't use matrix because of future issus
from numpy.matlib import matrix, identity
import numpy as np

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       # YOUR CODE HERE
                       }
        self.link = {'HeadYaw': [0, 0, 0.12650], 'HeadPitch': [0, 0, 0],
                     'LShoulderPitch': [0, 0.098, 0.100], 'LShoulderRoll': [0, 0, 0], 'LElbowYaw': [0.105, 0.015, 0],
                     'LElbowRoll': [0, 0, 0], 'LWristYaw': [0.05595, 0, 0],
                     'LHipYawPitch': [0, 0.050, -0.085], 'LHipRoll': [0, 0, 0], 'LHipPitch': [0, 0, 0],
                     'LKneePitch': [0, 0, -0.100], 'LAnklePitch': [0, 0, 0.10290], 'LAnkleRoll': [0, 0, 0],
                     'RShoulderPitch': [0, -0.098, 0.100], 'RShoulderRoll': [0, 0, 0], 'RElbowYaw': [0.105, -0.015, 0],
                     'RElbowRoll': [0, 0, 0], 'RWristYaw': [0.05595, 0, 0],
                     'RHipYawPitch': [0, -0.050, -0.085], 'RHipRoll': [0, 0, 0], 'RHipPitch': [0, 0, 0],
                     'RKneePitch': [0, 0, -0.100], 'RAnklePitch': [0, 0, 0.10290], 'RAnkleRoll': [0, 0, 0],
                     'LHand': [0.05775, 0, 0.01231], 'RHand': [0.05775, 0, 0.01231]
                     }

        # in mm

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = np.identity(4)
        # YOUR CODE HERE
        # implementation from http://www.oemg.ac.at/Mathe-Brief/fba2015/VWA_Prutsch.pdf chapter 4.4
        s = np.sin(joint_angle)
        c = np.cos(joint_angle)
        # x - axes
        if 'Roll' in joint_name:
            x = np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
            T = np.dot(T, x)
        # y - axes
        if 'Pitch' in joint_name:
            y = np.array([[c, 0, -s, 0], [0, 1, 0, 0], [s, 0, c, 0], [0, 0, 0, 1]])
            T = np.dot(T, y)
        # z - axes
        if 'Yaw' in joint_name:
            z = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            T = np.dot(T, z)

        T[:, -1][:-1] = self.link[joint_name]


        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = np.identity(4)
            for joint in chain_joints:
                # this two are not inside joints
                if joint == 'RWristYaw' or joint == 'LWristYaw' or joint == 'LHand' or joint == 'RHand':
                    angle = 0
                else:
                    angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T, Tl)
                # print T

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
