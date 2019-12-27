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
import os
import sys
import time
import Pyro4
import subprocess
import thread
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


@Pyro4.expose
class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        print 'get_angle'
        while self.rpc_block:
            print 'Get_Angle in Blocked Wait 1s'
            time.sleep(1)
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print 'set_angle'
        while self.rpc_block:
            print 'set_angle in Blocked Wait 1s'
            time.sleep(1)
        self.target_joints[joint_name] = angle
        return 1

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

if __name__ == '__main__':
    agent = ServerAgent()

    # start a Pyro4 dns server on the localhost
    dns_server = subprocess.Popen("pyro4-ns")
    # make the rpc server withhelp of Pyro4
    daemon = Pyro4.Daemon()
    print '1'
    ns = Pyro4.locateNS()
    print '2'
    uri = daemon.register(agent)
    print '3'
    ns.register("nao.robot", uri)
    print '4'
    rpc_server = thread.start_new_thread(daemon.requestLoop, ())
    print '5'
    # rpc_server.start()
    print 'RPC Server start!'

    agent.run()

