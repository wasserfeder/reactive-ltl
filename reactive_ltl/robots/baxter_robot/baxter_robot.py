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

default_config = {
    'baxter_utils_config': {
        'arm': "right",
    }
}

class BaxterRobot(object):
    def __init__(self, name, init=None, wspace=None, stepsize=0.1, config={}):
        # Robot.__init__(self, name, initConf=init, cspace=wspace, wspace=wspace,
        #                controlspace=stepsize)

        self.config = default_config
        self.config.update(config)
        self.baxter_utils = BaxterUtils(self.config['baxter_utils_config'])
        
        with open("env_config.json") as f:
            self.env = json.loads(f.read())

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        time.sleep(3)
        
        
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

    def move(self, joint_angles):
        self.baxter_utils.move_to_joint_position(joint_angles)

    def reset(self):
        self.baxter_utils.reset()

    def getSymbols(self, position, local=False):
        joint_angles = position
        gripper_position = np.array(self.fk(joint_angles)[:3])

        symbols = {}
        for object_name, value in viewitems(self.env):
            object_pose = self.tf_buffer.lookup_transform(value['parent_frame_id'][1:], value['child_frame_id'][1:], rospy.Time())
            object_pose = np.array([object_pose.transform.translation.x,
                                    object_pose.transform.translation.y,
                                    object_pose.transform.translation.z])

            if object_name == "table":
                symbols[object_name] = gripper_position[2] > object_pose[2] + 0.03
            else:
                if value['marker_type'] == "cube":
                    tmp = np.min([
                        gripper_position[0] - (object_pose[0] - value['scale'][0]/2),
                        (object_pose[0] + value['scale'][0]/2) - gripper_position[0],
                        gripper_position[1] - (object_pose[1] - value['scale'][1]/2),
                        (object_pose[1] + value['scale'][1]/2) - gripper_position[1],
                    ])
                    symbols[object_name] = tmp >= 0 and gripper_position[2] > object_pose[2] + 0.3 and gripper_position[2] <= object_pose[2] + 0.6
                    
                elif value['marker_type'] == "sphere":
                    symbols[object_name] = np.linalg.norm(gripper_position[:2] - object_pose[:2]) <= value['scale'][0]/2 and gripper_position[2] > object_pose[2] + 0.3 and gripper_position[2] <= object_pose[2] + 0.6
                else:
                    raise ValueError("marker type not supported")

        return [symbols[key] for key in symbols.keys()]
                    
    def isSimpleSegment(self, u, v):

        u_symbols = np.array(self.getSymbols(u))
        v_symbols = np.array(self.getSymbols(v))

        symbols = u_symbols
        
        ep = (list(u_symbols) and list(v_symbols)) or (list(u_symbols) and not list(v_symbols)) or (not list(u_symbols) and list(v_symbols))
            
        start_joints = np.array(u)
        end_joints = np.array(v)
        joints_diff = end_joints - start_joints
        joints_diff_sign = np.sign(joints_diff)
            
        inc = 0.01 # get intermediate joints from start to end in increments of 0.05 radians
        joints_inc = inc * joints_diff_sign
        
        joints = start_joints
        joint_heights_all = []

        for _ in range(int(np.max(np.abs(joints_diff/inc)))):
            s = np.array(self.getSymbols(joints))
            
            symbols = np.vstack([symbols,s])
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

        return res
            
if __name__ == "__main__":
    baxter = BaxterRobot(name="baxter")
    #print(baxter.fk([0,0,0,0,0,0,0]))
    #print(baxter.getSymbols([0,0,0,0,0,0,0]))
    print(baxter.isSimpleSegment([0,0,0,0,0,0,0], [0,1,0,0,1,0,1]))