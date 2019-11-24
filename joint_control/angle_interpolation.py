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


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.time = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        # NOTE needed to build in virtual time because time in perception.time is the simulation time and not
        rtime = perception.time
        if self.time == 0:
            self.time = rtime
        vtime = rtime - self.time

        for index in range(len(keyframes[0])):
            # get name
            name = keyframes[0][index]
            if name is 'LHand' or name is 'LWristYaw' or name is 'RHand' or name is 'RWristYaw':
                # Don't know what to todo with his values because they not inside perception.joints!!!!!
                None
            else:
                # check in wich time stamp we are
                for time_index in range(len(keyframes[1][index])):
                    time = keyframes[1][index][time_index]
                    if time > vtime:
                        p0 = perception.joint[name]
                        p3 = keyframes[2][index][time_index][0]
                        time_p1 = time + keyframes[2][index][time_index][1][1]
                        p1 = p3 + keyframes[2][index][time_index][1][2]
                        time_p2 = time + keyframes[2][index][time_index][2][1]
                        p2 = p3 + keyframes[2][index][time_index][2][2]
                        t = vtime / keyframes[1][index][-1]
                        point = (((1 - t) ** 3) * p0) + (((3 * ((1 - t) ** 2)) * p1) * t) + (
                                    ((3 * (1 - t)) * p2) * (t ** 2)) + (p3 * (t ** 3))
                        target_joints[name] = point

        # print target_joints

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    # agent.keyframes = leftBackToStand()
    # agent.keyframes = leftBellyToStand()
    # agent.keyframes = rightBackToStand()
    # agent.keyframes = rightBellyToStand()
    # agent.keyframes = wipe_forehead()
    agent.run()
