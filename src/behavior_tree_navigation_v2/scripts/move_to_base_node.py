#! /usr/bin/python3
import base64

import rospy
import behavior_tree_navigation_v2.msg as msgs

from actionlib_msgs.msg import GoalStatus
from execute_task_node import TasksActionsServer
from geometry_msgs.msg import PoseStamped


class MoveToBaseActionsServer(TasksActionsServer):
    def __init__(self):
        """"""
        self.pose_stamped = None
        TasksActionsServer.__init__(self, "move_to_base")

    def execute(self, action: msgs.TasksGoal):
        """"""
        self.deserialize(action.tasks)
        self.pose_stamped.header.stamp = rospy.Time.now()

        self.goal = GoalStatus()

        self.goal_publisher.publish(self.pose_stamped)

        feedback = msgs.TasksFeedback()
        result = msgs.TasksResult()
        while self.goal.status != GoalStatus.SUCCEEDED and not self.shutdown:
            feedback.status = f"{self.goal.status}"
            self.execute_goal.publish_feedback(feedback)
        self.goal.goal_id.id = ''
        self.goal.status = GoalStatus.PENDING
        result.status = 0
        rospy.loginfo(f"Got to base.")
        self.execute_goal.set_succeeded(result)

    def deserialize(self, data):
        """"""
        self.pose_stamped = PoseStamped()
        enc = 'utf-8'
        bytes_ = bytes(data, encoding=enc)
        base64_bytes = base64.b64decode(bytes_)
        self.pose_stamped.deserialize(base64_bytes)


if __name__ == '__main__':
    rospy.init_node("move_to_base_actions_server")
    rospy.loginfo("Initialised MoveToBaseActionsServer Node.")

    server = MoveToBaseActionsServer()
    server()
