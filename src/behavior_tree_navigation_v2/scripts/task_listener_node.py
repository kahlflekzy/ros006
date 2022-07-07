#! /usr/bin/python3
import base64

import behavior_tree_navigation_v2.msg as msgs
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

import actionlib
from io import BytesIO, StringIO
import rospy
import typing


class TaskListenerActionServer:
    """"""

    def __init__(self):
        """"""
        self.tasks: typing.List[msgs.PoseTimeArray] = []

        self.init_subscribers()

        self.server = actionlib.SimpleActionServer("task_listener", msgs.TasksListenerAction, self.execute, False)
        self.server.start()

        rospy.sleep(2)

    def __call__(self, *args, **kwargs):
        rospy.spin()

    def execute(self, action):
        """"""
        rospy.loginfo(f"Got task listen request with flag: {action.start}")
        result = msgs.TasksListenerResult()
        rate = rospy.Rate(10)
        if action.start == 1:
            while len(self.tasks) == 0:
                rate.sleep()
            task = self.tasks.pop(0)
            result.tasks = self.serialize(task)
        else:
            result.tasks = '0'
        self.server.set_succeeded(result)

    def init_subscribers(self):
        """Define topics for subscription"""
        rospy.Subscriber("/tasks_listener/tasks", msgs.PoseTimeArray, self.add_task)
        rospy.loginfo("Initialised subscribers.")

    def add_task(self, data):
        """"""
        self.tasks.append(data)

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

        pose.pose.pose = TaskListenerActionServer.to_pose(tuple(map(float, goal)))
        pose.pose.covariance = list(map(float, covariance.split(',')))

        return pose

    @staticmethod
    def serialize(data) -> str:
        """
        Serialize data and use base64 to encode data into a a printable format.

        :param data:
        :return: serialized_string
        """
        buff = BytesIO()
        data.serialize(buff)
        serialized_bytes = buff.getvalue()
        enc = 'utf-8'
        base64_bytes = base64.b64encode(serialized_bytes)
        serialized_string = str(base64_bytes, encoding=enc)
        return serialized_string


if __name__ == '__main__':
    rospy.init_node("task_listener_server")
    rospy.loginfo("Initialised TaskListenerActionServer Node.")

    server = TaskListenerActionServer()
    server()
