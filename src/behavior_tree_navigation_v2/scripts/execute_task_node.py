#! /usr/bin/python3
import base64

import behavior_tree_navigation_v2.msg as msgs
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

import actionlib
import rospy


class TasksActionsServer:
    """
    Creates two servers

    - One for communicating tasks with the TaskListener client\
    - Another for communicating with the ExecuteGoals client\
    """

    def __init__(self, server_name="move_robot"):
        """"""
        self.tasks = None

        self.shutdown = False
        self.goal = GoalStatus()

        self.initial_pose_publisher = None
        self.goal_publisher = None

        self.init_subscribers()
        self.init_publishers()

        self.execute_goal = actionlib.SimpleActionServer(server_name, msgs.TasksAction, self.execute, False)

        self.execute_goal.start()

    def execute(self, action: msgs.TasksGoal):
        """"""
        self.deserialize(action.tasks)
        tasks_count = len(self.tasks.poses)
        task_id = 0
        feedback = msgs.TasksFeedback()
        result = msgs.TasksResult()

        self.goal = GoalStatus()  # might have been forced set by other callers.

        while task_id < tasks_count:
            pose_time = self.tasks.poses[task_id]
            pose_stamped = self.get_pose_stamped(pose_time)

            self.goal_publisher.publish(pose_stamped)

            while self.goal.status != GoalStatus.SUCCEEDED and not self.shutdown:
                feedback.status = f"{task_id}-{self.goal.status}"
                self.execute_goal.publish_feedback(feedback)
            self.goal.goal_id.id = ''
            self.goal.status = GoalStatus.PENDING
            task_id += 1
            rospy.loginfo(f"Got to Pose. Waiting for {pose_time.duration} seconds.")
            rospy.sleep(pose_time.duration)
        result.status = 0
        self.execute_goal.set_succeeded(result)

    def init_subscribers(self):
        """Define topics for subscription"""
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.worker)
        rospy.loginfo("Initialised subscribers.")

    def init_publishers(self):
        """Define publishers."""
        self.goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.sleep(2)

    def __call__(self, *args, **kwargs):
        rospy.on_shutdown(self.handle_shutdown)
        rospy.spin()

    def handle_shutdown(self):
        """"""
        rospy.loginfo("Cleaning Up.")
        self.shutdown = True

    def worker(self, goal_array_status):
        """"""
        if len(goal_array_status.status_list) != 0:
            goal = goal_array_status.status_list[-1]
            if self.goal.goal_id.id == '' and goal.status != GoalStatus.SUCCEEDED:
                self.goal.goal_id.id = goal.goal_id.id
                self.goal.status = goal.status
            elif self.goal.goal_id.id == goal.goal_id.id:
                self.goal.status = goal.status
                self.goal.text = goal.text

    def deserialize(self, data):
        """
        Deserialize data: str into a task.
        First converts string into a byte, then uses base64 to decode
        the byte into it's original format.

        :param data:
        :return:
        """
        self.tasks = msgs.PoseTimeArray()
        enc = 'utf-8'
        bytes_ = bytes(data, encoding=enc)
        base64_bytes = base64.b64decode(bytes_)
        self.tasks.deserialize(base64_bytes)

    def get_pose_stamped(self, pose_time):
        """
        Creates a PoseStamped msg from a PoseTime msg.

        :param pose_time:
        :return:
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.tasks.header.frame_id
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose_time.pose

        return pose_stamped


if __name__ == '__main__':
    rospy.init_node("tasks_actions_server")
    rospy.loginfo("Initialised TasksActionsServer Node.")

    server = TasksActionsServer()
    server()
