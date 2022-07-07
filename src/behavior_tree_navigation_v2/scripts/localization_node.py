#! /usr/bin/python3
import base64

import behavior_tree_navigation_v2.msg as msgs
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped

import actionlib
import glob
from io import BytesIO
import rospy
import sys


class LocalizeActionServer:
    """"""

    def __init__(self, file):
        """"""
        self.initial_pose_publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.file = file
        self.pose_stamped_bytes = None
        self.init_pose = False
        self.server = actionlib.SimpleActionServer("localize", msgs.LocalizeAction, self.execute, False)
        self.server.start()
        rospy.loginfo(self.file)
        rospy.sleep(2)

    def __call__(self, *args, **kwargs):
        rospy.spin()

    def execute(self, action):
        """We don't use the action itself"""
        result = msgs.LocalizeResult()
        rate = rospy.Rate(10)
        if not self.init_pose:
            self.initialise_pose()
        while not self.init_pose:
            rate.sleep()
        result.pose = self.pose_stamped_bytes
        self.server.set_succeeded(result)

    def initialise_pose(self):
        """
        Creates an initial pose and publishes it if self.init_pose is True.

        :return:
        """
        rospy.loginfo("Getting initial pose")
        with open(self.file) as file_obj:
            poses = file_obj.readlines()
            pose_str = poses[-1]
            pose = self.to_pose_with_covariance(pose_str)
            self.serialize_pose_stamped(pose)
        if pose is not None:
            rospy.loginfo("Publishing initial pose")
            # Fixme, uncomment
            self.initial_pose_publisher.publish(pose)
            self.init_pose = True

    @staticmethod
    def to_pose(goal: tuple):
        assert len(goal) == 7

        pose = Pose()

        goal = tuple(map(float, goal))

        pose.position.x = goal[0]
        pose.position.y = goal[1]
        pose.position.z = goal[2]
        pose.orientation.x = goal[3]
        pose.orientation.y = goal[4]
        pose.orientation.z = goal[5]
        pose.orientation.w = goal[6]

        return pose

    @staticmethod
    def to_pose_with_covariance(goal: str):
        """
        Takes our string formatted goal and creates and returns a pose with covariance stamped message

        :param goal:
        :return:
        """
        goal = goal.split(";")
        assert len(goal) == 9

        covariance = goal.pop()

        pose = PoseWithCovarianceStamped()

        pose.header.frame_id = goal.pop(0)
        pose.header.stamp = rospy.Time.now()

        pose.pose.pose = LocalizeActionServer.to_pose(tuple(map(float, goal)))
        pose.pose.covariance = list(map(float, covariance.split(',')))

        return pose

    def serialize_pose_stamped(self, pose):
        """
        Serializes a PoseStamp according to our unique extra requirements for serialization,
        ie, using base64 and converting to string.

        :param pose:
        :return:
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = pose.header.frame_id
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose.pose.pose

        buff = BytesIO()
        pose_stamped.serialize(buff)
        serialized_bytes = buff.getvalue()
        enc = 'utf-8'
        base64_bytes = base64.b64encode(serialized_bytes)
        self.pose_stamped_bytes = str(base64_bytes, encoding=enc)


if __name__ == '__main__':
    rospy.init_node("localization_server")
    rospy.loginfo("Initialised LocalizeActionServer Node.")

    argv = rospy.myargv(argv=sys.argv)
    if len(argv) > 1:
        init_pose_file = argv[1]
    else:
        path = "/home/kahlflekzy/PycharmProjects/data/poses"
        init_pose_file = glob.glob(f"{path}/init_pose*.dat")[0]

    server = LocalizeActionServer(init_pose_file)
    server()
