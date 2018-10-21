#from robots.robot import robot
from baxter_api.baxter_utils import BaxterUtils
import json
from future.utils import viewitems
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg
import rospy
import time
import os

from robots import Robot
from spaces import Point

default_config = {
    'baxter_utils_config': {
        'arm': "right",
        'env_json_path': os.path.join(os.getcwd(), "env_config.json") 
    }
}

class BaxterRobot(Robot):
    def __init__(self, name, init=None, cspace=None, wspace=None,
                 stepsize=0.1, config={}):
        Robot.__init__(self, name, initConf=init, cspace=cspace, wspace=wspace,
                       controlspace=stepsize)

        self.config = default_config
        self.config.update(config)

        json_filename = self.config.get('json-filename', 'env_config.json')
        self.config['baxter_utils_config']['env_json_path'] = json_filename
        
        self.baxter_utils = BaxterUtils(self.config['baxter_utils_config'])

        with open(json_filename) as f:
            self.env = json.loads(f.read())

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        time.sleep(3)

        self.max_cache_size = 25000
        self.cache = dict()
        self.fifo = [None] * self.max_cache_size
        self.cache_index = 0

        self.all_requests = [] # a list of all defined requests

    def sensor_update(self):
        '''#TODO: Remove/inactivate serviced requests'''
        pass

    def sensor_reset(self):
        '''#TODO: Utility function called at the end of a surveillance cycle.'''
        pass

    def get_interactive_object_position(self):
        for k, v in viewitems(self.env):
            if v['marker_type'] == "interactive":
                object_pose = self.tf_buffer.lookup_transform(
                    value['parent_frame_id'][1:], value['child_frame_id'][1:],
                    rospy.Time())
                object_pose = np.array([object_pose.transform.translation.x,
                                        object_pose.transform.translation.y,
                                        object_pose.transform.translation.z])

                return object_pose

        
    def fk(self, joint_angles):
        '''
        joint_angles is a list of length 7
        '''
        gripper_pose = self.baxter_utils.fk(joint_angles).pose_stamped[0]
        return [gripper_pose.pose.position.x,
                gripper_pose.pose.position.y,
                gripper_pose.pose.position.z,
                gripper_pose.pose.orientation.x,
                gripper_pose.pose.orientation.y,
                gripper_pose.pose.orientation.z,
                gripper_pose.pose.orientation.w]

    def steer(self, start, target, atol=0):
        '''Returns a position that the robot can move to from the start position
        such that it steers closer to the given target position using the
        robot's dynamics.

        Note: It simulates the movement.
        '''
        s = start.coords
        t = target.coords
        dist = self.cspace.metric(s, t)
        if dist <= self.controlspace + atol:
            return target
        return self.initConf.__class__(s + (t-s) * self.controlspace/dist)

    def move(self, joint_angles):
        joint_angles = list(joint_angles.coords) + [0]
        # to account for the order difference in fk and move_to_joint_position
        remapped_joint_angles = [joint_angles[i] for i in [2, 3, 0, 1, 4, 5, 6]]
        self.baxter_utils.move_to_joint_position(remapped_joint_angles)

    def reset(self):
        self.baxter_utils.reset()

    def getSymbols(self, position, local=False, bitmap=False):
        if position in self.cache:
            if bitmap:
                return self.cache[position][0]
            return self.cache[position][1]

        joint_angles = tuple(position.coords) + (0,)
        gripper_position = np.array(self.fk(joint_angles)[:3])

        symbols = {}
        for object_name, value in viewitems(self.env):
            object_pose = self.tf_buffer.lookup_transform(
                value['parent_frame_id'][1:], value['child_frame_id'][1:],
                rospy.Time())
            object_pose = np.array([object_pose.transform.translation.x,
                                    object_pose.transform.translation.y,
                                    object_pose.transform.translation.z])

            if object_name == "table":
                symbols[object_name] = gripper_position[2] > object_pose[2] + 0.03 and \
                                       gripper_position[2] < object_pose[2] + 0.7 and \
                                       abs(gripper_position[0] - object_pose[0]) < value['scale'][0] / 2 and \
                                       abs(gripper_position[1] - object_pose[1]) < value['scale'][1] / 2
            else:
                if value['marker_type'] == "cube":
#                     tmp = np.min([
#                         gripper_position[0] - (object_pose[0] - value['scale'][0]/2),
#                         (object_pose[0] + value['scale'][0]/2) - gripper_position[0],
#                         gripper_position[1] - (object_pose[1] - value['scale'][1]/2),
#                         (object_pose[1] + value['scale'][1]/2) - gripper_position[1],
#                     ])
#                     symbols[object_name] = (tmp >= 0
#                             and gripper_position[2] > object_pose[2] + 0.03
#                             and gripper_position[2] <= object_pose[2] + 0.3)
                    l, h = np.zeros((2, 3), dtype=np.float)
                    h[:2] = np.asarray(value['scale'][:2])/2
                    l[:2] = -h[:2]
                    l[3], h[3] = 0.03, 0.3
                    symbols[object_name] = np.all(
                                    l <= gripper_position - object_pose <= h)

                elif value['marker_type'] == "sphere":
                    dist = np.linalg.norm(gripper_position[:2] - object_pose[:2])
                    l, h = 0.06, 0.3
                    symbols[object_name] = (dist<= value['scale'][0]/2
                                            and l < gripper_position[2] - object_pose[2] <= h)
                elif value['marker_type'] == "interactive":
                    pass
                else:
                    raise ValueError("marker type not supported")

        # cache result
        b = [symbols[key] for key in symbols.keys()]
        s = set(key for key in symbols.keys() if symbols[key])
        # check for cache size
        if self.fifo[self.cache_index] is not None:
            del self.cache[self.fifo[self.cache_index]]
        # add to cache
        self.cache[position] = (b, s)
        self.fifo[self.cache_index] = position
        self.cache_index += 1
        self.cache_index %= self.max_cache_size
        assert(len(self.cache) <= self.max_cache_size)

        if bitmap:
            return b
        return s

    def isSimpleSegment(self, u, v):

        u_symbols = np.array(self.getSymbols(u, bitmap=True))
        v_symbols = np.array(self.getSymbols(v, bitmap=True))

        symbols = u_symbols

        ep = (list(u_symbols) and list(v_symbols)) \
              or (list(u_symbols) and not list(v_symbols)) \
              or (not list(u_symbols) and list(v_symbols))

        start_joints = np.array(u.coords)
        end_joints = np.array(v.coords)
        joints_diff = end_joints - start_joints
        joints_diff_sign = np.sign(joints_diff)

        # inc = 0.01 # get intermediate joints from start to end in increments of 0.05 radians
        # joints_inc = inc * joints_diff_sign

        interval = 100 # divide into 100 steps
        joints_inc = np.abs(joints_diff/interval) * joints_diff_sign
        
        joints = start_joints
        joint_heights_all = []

        #for _ in range(int(np.max(np.abs(joints_diff/inc)))):
        for _ in range(interval):
            s = np.array(self.getSymbols(Point(joints), bitmap=True))

            symbols = np.vstack([symbols, s])
            for i in range(len(joints)):
                if (joints[i] < end_joints[i] and joints_diff_sign[i] > 0) or \
                   (joints[i] > end_joints[i] and joints_diff_sign[i] < 0):
                    joints[i] += joints_inc[i]

        symbols = np.vstack([symbols, v_symbols])
        symbols = symbols[1:-1,:]

        res = []
        i = 0
        for r in ep:
            if r:
                res.append(r)
            else:
                res.append(np.all(symbols[:,i] == symbols[0,i]))
            i += 1

        return all(res)

if __name__ == "__main__":
    baxter = BaxterRobot(name="baxter")
    baxter.reset()
    #print(baxter.fk([0,0,0,0,0,0,0]))
    #print(baxter.getSymbols([0,0,0,0,0,0,0]))
    # print(baxter.isSimpleSegment(Point([0,0,0,0,0,0]),
    #                              Point([0,1,0,0,1,1])))
    while True:
        baxter.get_interactive_object_position()