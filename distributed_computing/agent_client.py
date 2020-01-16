'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import Pyro4
import sys
import os
import threading
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from keyframes import hello, leftBackToStand


class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.execute_keyframes(keyframes))
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.proxy.set_transform(effector_name, transform))
        thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.rpc_client = Pyro4.Proxy("PYRONAME:nao.robot")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.rpc_client.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        temp = self.rpc_client.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.rpc_client.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        temp = self.rpc_client.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.rpc_client.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        temp = self.rpc_client.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    print "start"
    print 'first get the angle of LShoulderPitch'
    print agent.get_angle('LShoulderPitch')
    print 'now set the angle of RShoulderPitch to 1'
    agent.set_angle('RShoulderPitch', 1)
    print 'now get the transform of LHipYwaPitch'
    print agent.get_transform('LHipYawPitch')
    from numpy.matlib import identity
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    T = T.tolist()
    print 'now execute the keyframe hello'
    agent.execute_keyframes(hello())
    print 'now set the transorm of LLeg to ', T
    agent.set_transform('LLeg', T)
    print 'now get the posture'
    print agent.get_posture()
    print 'now testing the Post Handler, the server side block the call so that the server only can do one task and ' \
          'not both togethere'
    agent.post.execute_keyframes(hello())
    agent.post.set_transform('LLeg', T)
    # TEST CODE HERE


