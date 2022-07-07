#! /usr/bin/python3

import behavior_tree_navigation_v2.msg as msgs
import nav_msgs.msg as nav_msgs

import actionlib
import os
import rospy
import signal
import subprocess
import sys


class MapServer:
    """"""
    def __init__(self, map_file = None):
        """

        :param map_file:
        """
        self.map_file = map_file
        if self.map_file is None:
            self.get_map_file()
        rospy.loginfo(f"Got map file {self.map_file}")

        self.status = False
        self.publisher = 1
        rospy.Subscriber("/map_metadata", nav_msgs.MapMetaData, self.map_server_listener)
        self.server = actionlib.SimpleActionServer("serve_map", msgs.LoadMapAction, self.execute, False)
        self.server.start()
        command = ' '.join(['rosrun', 'map_server', 'map_server', self.map_file])
        self.map_process = subprocess.Popen(command, shell=True)

    def __call__(self, *args, **kwargs):
        rospy.on_shutdown(self.handle_shutdown)
        rospy.spin()

    def get_map_file(self):
        """
        If
        :return:
        """
        map_file = "/home/kahlflekzy/PycharmProjects/ros003/map.yaml"
        # FIXME: I added the two lines below as a hack to avoid waiting for user input. It affects the behaviortree
        self.map_file = map_file
        return
        prompt = f"""
        Available map file: 
            1. {map_file}
        Input number (e.g. 1) to select a file from the above listed.
        Input 0 to enter the path to a different map file.
        """
        while True:
            print(prompt)
            input_ = input()
            if input_ == '1':
                self.map_file = map_file
                break
            elif input_ == '0':
                input_ = input("Enter the absolute path to a map_file. ")
                if os.path.exists(input_):
                    self.map_file = input_
                    break
                else:
                    print(f"No file found at {input_}")
            else:
                print("Invalid Input.")

    def execute(self, action):
        result = msgs.LoadMapResult()
        rate = rospy.Rate(10)
        if not self.map_process.poll():
            while not self.status:
                rate.sleep()
            result.status = 1
        else:
            result.status = 0
        self.server.set_succeeded(result)

    def map_server_listener(self, data):
        rospy.loginfo("MapServer Up.")
        self.status = True

    def handle_shutdown(self):
        rospy.loginfo("Killing MapServer.")
        self.map_process.terminate()
        self.map_process.wait(2)

if __name__ == '__main__':
    rospy.init_node("map_action_server")
    rospy.loginfo("Initialised MapActionServer Node.")

    argv = rospy.myargv(argv=sys.argv)

    if len(argv) > 1:
        # roslaunch
        map_ = argv[1]
    else:
        # rosrun
        map_ = None
    print(argv)
    server = MapServer(map_file=map_)
    server()
