import rospy
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import tf

def get_pose_stamped_from_tf(trans, rot):
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp=rospy.Time.now()
    ps.pose.position.x=trans[0]
    ps.pose.position.y=trans[1]
    ps.pose.position.z=trans[2]
    ps.pose.orientation.x=rot[0]
    ps.pose.orientation.y=rot[1]
    ps.pose.orientation.z=rot[2]
    ps.pose.orientation.w=rot[3]
    return ps

def get_pose_from_tf(trans, rot):
    pose = Pose()
    pose.position.x=trans[0]
    pose.position.y=trans[1]
    pose.position.z=trans[2]
    pose.orientation.x=rot[0]
    pose.orientation.y=rot[1]
    pose.orientation.z=rot[2]
    pose.orientation.w=rot[3]
    return pose

def get_target_yaw(robot_position, robot_orientation, frontier_position, angle):
    """
    Returns the desired yaw angle for the robot to reach its goal
    robot_position: (x, y, z)
    robot_orientation: (roll, pitch, yaw)
    frontier_position: front["location"]
    angle: front["angle"] - angle is relative to the robots position 
    """
    # rf_x = frontier_position[0] - robot_position[0] # frontier pos relative to robot
    # rf_y = frontier_position[1] - robot_position[1]

    ret = None
    # print("({}, {})".format(rf_x, rf_y))
    
    if angle < np.pi/2:
        ret = (robot_orientation[0], robot_orientation[1], robot_orientation[2] + (-angle + np.pi/2))
    else:
        ret = (robot_orientation[0], robot_orientation[1], robot_orientation[2] - (angle-np.pi/2))
    return ret

def from_quaternion(msg):
    """
    Returns a numpy array from a ROS quaternion
    """
    return np.array([msg.x, msg.y, msg.z, msg.w])

def from_position(msg):
    """
    Returns robot position as a numpy array
    """
    return np.array([msg.x, msg.y, msg.z])

def inverse_homog(mat):
    """
    Returns the invers of a homogeneous transformation matrix
    """
    out = np.eye(4)
    R = mat[0:3, 0:3]
    p = mat[0:3, 3]
    out[0:3, 0:3] = R.T
    out[0:3, 3] = -np.transpose(R[0:3, 0:3]).dot(p)
    assert(np.allclose(np.eye(4), mat.dot(out)))
    return out

def homog_from_pose(pose):
    """
    Helper function to get the 4x4 homogenous represntation from a ROS Pose message
    """
    p = from_position(pose.position) # get the robots position as a numpy array
    g = tf.transformations.quaternion_matrix(from_quaternion(pose.orientation))
    g[0:3, 3] = p # set p of homogeneous transform
    return g

def dist(ps1, ps2):
    """
    Helper function to get the distance between two PoseStamped objects
    """
    pos1 = ps1.pose.position
    pos1 = np.array([pos1.x, pos1.y, pos1.z])
    pos2 = ps2.pose.position
    pos2 = np.array([pos2.x, pos2.y, pos2.z])
    return np.linalg.norm(pos2 - pos1)
