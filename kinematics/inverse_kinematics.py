'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from scipy.optimize import fmin_cg


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def error_func(self, angle_start, effector, transform):
        T = np.identity(4)
        for i, current_joint in enumerate(self.chains[effector]):
            angle = angle_start[i]
            Tl = self.local_trans(current_joint, angle)
            T = np.dot(T, Tl)
        rotation_error = np.sum(np.power((T[0:3, 0:3] * transform.T[0:3, 0:3]) - identity(3), 2))
        trans_error = np.linalg.norm(T[0:3, 3:4] - transform[0:3, 3:4])

        return rotation_error * 12 + trans_error

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        joint_angles = []
        # YOUR CODE
        for joint in self.chains[effector_name]:
            joint_angles.append(self.perception.joint[joint])
        joint_angles = fmin_cg(self.error_func, joint_angles, args=(effector_name, transform))
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        angle = self.inverse_kinematics(effector_name, transform)
        names = list()
        time = list()
        key = list()

        for i, joint in enumerate(self.chains[effector_name]):
            names.append(joint)
            time.append([0, 1])
            key.append([[0, [3, -0.00001, 0.00000], [3, 0.00001, 0.00001]],
                        [angle[i], [3, -0.00001, 0.00001], [3, 0.00001, 0.00001]]])
        self.set_time(0)
        self.keyframes = (names, time, key)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics

    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
    """
    T = identity(4)
    T[0, 0] = 2
    T[0, 1] = 2
    T[0, 2] = 2

    agent.set_transforms('RLeg', T)
    agent.run()
    """
