#! /usr/bin/python3

from geometry_msgs.msg import Pose
from behavior_tree_navigation_v2.msg import PoseTime, PoseTimeArray

import random
import rospy


class Publisher:
    """
    * Gets some poses I had stored in a particular file,
    * Parses them into PoseTime(s),
    * Publishes PoseTimeArray messages based on some selections made by the user.
    """

    def __init__(self):
        self.pub = rospy.Publisher("/tasks_listener/tasks", PoseTimeArray, queue_size=5)
        self.poses = []  # Array of PoseTime(s)
        self.header_frame_id = None
        self.number_of_goals = 0
        self.get_poses()

    def get_poses(self):
        """
        Gets the poses from file, calls a function that does the parsing, then stores them in a list.

        :return:
        """
        file_path = f"/home/kahlflekzy/PycharmProjects/data/poses/goals_22_06_14_13_55_06.dat"
        with open(file_path) as file:
            goals = file.readlines()
            self.number_of_goals = len(goals)
            for goal in goals:
                pose = self.goal_to_pose(goal)
                pose_time = self.pose_to_pose_time(pose)
                self.poses.append(pose_time)

    def goal_to_pose(self, goal) -> Pose:
        """
        Converts a goal string into a geometry_msg/Pose.

        :param goal:
        :return:
        """
        goal = goal.split(";")
        pose = Pose()
        self.header_frame_id = goal.pop(0)

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
    def pose_to_pose_time(pose) -> PoseTime:
        """
        Convert a geometry_msg/Pose into a behavior_tree_navigation_v2/PoseTime msg.
        Assigns some random value between 2-5 seconds as the PoseTime's duration.

        :param pose:
        :return: pose_time
        """
        pose_time = PoseTime()
        pose_time.duration = random.randint(2, 5)
        pose_time.pose = pose
        return pose_time

    def __call__(self, *args, **kwargs):
        """
        Main loop

        :param args:
        :param kwargs:
        :return:
        """
        rospy.init_node("task_publisher")
        while not rospy.is_shutdown():
            self.prompt()
            try:
                message = self.get_selection()
                self.pub.publish(message)
            except IndexError:
                rospy.loginfo("Wrong Input.")

    def get_selection(self) -> PoseTimeArray:
        """
        Get user's selection and then choose selected poses for publishing.

        :return:
        """
        f = input()
        pose_array = PoseTimeArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = self.header_frame_id
        pose_array.poses = []
        if len(f) != 0 and f != '0':
            fs = map(int, list(f))
            poses = [self.poses[i-1] for i in fs]
            pose_array.poses = poses
        return pose_array

    def prompt(self):
        m = map(str, range(1, len(self.poses)+1))
        message = f"""
                Available for selection {', '.join(m)}\n
                For example, to select pose 1, 2, 3 and 4, type: 1234
                Type just '0' to create an empty PoseTimeArray: 0 
                Note that:
                    1. There is not space in the input.
                    2. Your selection should be within the specified ranges.
                    3. The order of your selection is maintained.\n
                """
        rospy.loginfo(message)


if __name__ == '__main__':
    node = Publisher()
    node()
