'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes  the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.time = 0
        self.keyframe_start = []

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        # NOTE needed to build in virtual time because time in perception.time is the simulation time and not
        # build own first keyframe little bit change that impact in the result
        rtime = perception.time
        if self.time == 0:
            self.time = rtime
            for j, n in enumerate(self.keyframes[0]):
                offset = self.keyframes[2][j][0][1][1]
                if n in self.perception.joint:
                    self.keyframe_start.append([self.perception.joint[n], [3, 0, 0], [3, -offset, 0]])
                else:
                    self.keyframe_start.append([0, [3, 0, 0], [3, -offset, 0]])
        vtime = rtime - self.time

        for i, name in enumerate(keyframes[0]):
            # getting simulations times
            stime = keyframes[1][i]
            if vtime > stime[-1]:
                break

            # getting points x is for time and y is for the angle
            index = len([x for x in stime if x < vtime])
            if index == 0:
                p0x = 0
                key_l = self.keyframe_start[i]
            else:
                p0x = stime[index - 1]
                key_l = keyframes[2][i][index - 1]

            p1x = p0x + key_l[2][1]
            key_r = keyframes[2][i][index]
            p3x = stime[index]
            p2x = p3x + key_r[1][1]
            p0y = key_l[0]
            p1y = p0y + key_l[2][2]
            p3y = key_r[0]
            p2y = p3y + key_r[1][2]

            bezierMatrix = np.array([[1, 0, 0, 0], [-3, 3, 0, 0], [3, -6, 3, 0], [-1, 3, -3, 1]])
            x = np.array([p0x, p1x, p2x, p3x])
            y = np.array([p0y, p1y, p2y, p3y])
            coefficientsX = np.dot(bezierMatrix, x)
            coefficientsX[0] -= vtime
            candidates = np.polynomial.polynomial.polyroots(coefficientsX)
            epsilon = 1e-6  # error margin for x to t conversion
            # finding correct candidate for t (t has to be in [0,1])
            t = [x.real for x in candidates if -(epsilon) <= x.real <= 1 + (epsilon) and x.imag == 0][0]


            # getting y values
            coefficientsY = np.dot(bezierMatrix, y)
            result = np.dot(np.array([1, t, t ** 2, t ** 3]), coefficientsY)
            target_joints[name] = result

        for key, value in target_joints.items():
            print key, ': ', value

        print '****************************************************************'

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = leftBackToStand()
    # agent.keyframes = leftBellyToStand()
    # agent.keyframes = rightBackToStand()
    # agent.keyframes = rightBellyToStand()
    # agent.keyframes = wipe_forehead()
    agent.run()
