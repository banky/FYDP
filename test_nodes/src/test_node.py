import numpy as np
import math as m
import rospy
import tf
from controls.msg import TwoPointVector
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3, Point
from powertrain.msg import PowertrainParams
    
# Trajectory straight line on y-axis
traj_start = Point()
traj_start.x = 0
traj_start.y = 0

traj_end = Point()
traj_end.x = 0
traj_end.y = 10000

# Position
ps = PoseStamped()
ps.pose.position.x = 10
ps.pose.position.y = 50
ps.pose.position.z = 0

# Orientation
quaternion = tf.transformations.quaternion_from_euler(0, 0, m.pi/4)
ps.pose.orientation.x = quaternion[0]
ps.pose.orientation.y = quaternion[1]
ps.pose.orientation.z = quaternion[2]
ps.pose.orientation.w = quaternion[3]


def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down localization node...")


def main():
    rospy.init_node("test_nodes")
    rospy.loginfo("Starting Test Nodes")
    rospy.on_shutdown(shutdown_hook)

    #Subscribe to Pose node for position and orientation data
    pose_pub = rospy.Publisher('localization/pose', PoseStamped, queue_size=10)

    #Subscribe to trajectory node for path to follow
    traj_pub = rospy.Publisher('path_planning/trajectory', TwoPointVector, queue_size=10)

    while not rospy.is_shutdown():
        traj_pub.publish(TwoPointVector(traj_start, traj_end))
        pose_pub.publish(ps)


if __name__ == "__main__":
    main()
