'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''
# using Pyro4 need to install with pip

# add PYTHONPATH
import numpy as np
import os
import sys
import time
import Pyro4
import subprocess
import thread
import pickle
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


@Pyro4.expose
class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ServerAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('robot_pose.pkl'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(ServerAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        feature_list = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch',
                        'RKneePitch', 'AngleX', 'AngleY']
        data = np.zeros(len(feature_list))

        for i in range(len(feature_list) - 2):
            data[i] = perception.joint[feature_list[i]]

        data[-2] = perception.imu[0]
        data[-1] = perception.imu[1]

        post_data = []
        post_data.append(data)
        post_data = np.asarray(post_data)

        posture = self.posture_classifier.predict(post_data)
        self.posture = posture

        return posture

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        print 'get_angle'
        while self.rpc_block:
            print 'Get_Angle in Blocked Wait 1s'
            time.sleep(1)
        print self.perception.joint[joint_name]
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print 'set_angle'
        while self.rpc_block:
            print 'set_angle in Blocked Wait 1s'
            time.sleep(1)
        self.rpc_block = True
        self.target_joints[joint_name] = angle
        self.rpc_block = False
        time.sleep(0.3)
        return 1

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print 'get_posture'
        while self.rpc_block:
            print 'get_posture in Blocked Wait 1s'
            time.sleep(1)
        return np.array2string(self.posture)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print 'execute_keyframes'
        while self.rpc_block:
            print 'execute_keyframes in Blocked Wait 1s'
            time.sleep(1)
        self.rpc_block = True
        self.set_time(0)
        self.keyframes = keyframes
        self.rpc_block = False
        time_to_wait = keyframes[1][0][-1] + 1      # wait one second more then the last time in the first time slot
        time.sleep(time_to_wait)
        return 1

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print 'get_transform'
        while self.rpc_block:
            print 'get_transform in Blocked Wait 1s'
            time.sleep(1)
        return self.transforms[name].tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print 'set_transform'
        while self.rpc_block:
            print 'set_transform in Blocked Wait 1s'
            time.sleep(1)
        self.rpc_block = True
        self.set_transforms(effector_name, np.matrix(transform))
        time.sleep(2) # sleep to 1.2 time to in set_transform is everytime 1s
        self.rpc_block = False
        return 1

if __name__ == '__main__':
    agent = ServerAgent()

    # start a Pyro4 dns server on the localhost
    dns_server = subprocess.Popen("pyro4-ns")
    # make the rpc server withhelp of Pyro4
    daemon = Pyro4.Daemon()
    ns = Pyro4.locateNS()
    uri = daemon.register(agent)
    ns.register("nao.robot", uri)
    rpc_server = thread.start_new_thread(daemon.requestLoop, ())
    print 'RPC Server start!'

    agent.run()

