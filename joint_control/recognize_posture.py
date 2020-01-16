'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello, leftBellyToStand, leftBackToStand, rightBellyToStand, rightBackToStand, wipe_forehead
import pickle
import numpy as np


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('robot_pose.pkl'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

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

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    # agent.keyframes = leftBackToStand()
    # agent.keyframes = leftBellyToStand()
    agent.run()
