#!/usr/bin/env python
# Read a CSV file of all the poses
# Publish to Rviz as a path message

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import numpy as np
import sys
import csv
import time
from nav_msgs.msg import Path

from challenge_tools_lib import ReactPose
from challenge_tools_lib import ReactTime
from challenge_tools_lib import ChallengeToolsLib

class GTServer(Node):
    def __init__(self):
        super().__init__('groundtruth_server')
        self.pub_path = self.create_publisher(Path, '/gt_poses', 10)
        self.fixed_frame = "map"

        self.ctl = ChallengeToolsLib()
        self.ctl.fixed_frame = self.fixed_frame


    def do_work(self, run_name):

        try:
            path = get_package_share_directory("challenge_tools_ros")
            print("share dir:", path)
        except PackageNotFoundError:
            print(f"Package challenge_tools_ros not found in the ament index.")

        main_folder = path + "/groundtruth/"
        csv_filename = main_folder + "/" + run_name + "/" + "gt_poses.csv"
        print("Reading: ", csv_filename)

        gt_poses = self.ctl.load_csv_as_poses_list(csv_filename)
        gt_poses = list(gt_poses.items())

        while (1==1):
            self.get_logger().info(f"publish ground truth for: {run_name}")
            self.ctl.publish_trajectory(self.pub_path, gt_poses)
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    args = sys.argv[1:]
    num_args = len(args)
    print("Usage:")
    print("  groundtruth_server.py run_name")
    print(" ")
    if (num_args < 1):
        print("1 required argument.", num_args, "provided.")
        print("quitting")
        return
    
    print("Using commandline configuration:")
    run_name = args[0]
    print("  run_name:", run_name)
    print(" ")

    app = GTServer()
    app.do_work(run_name)
    app.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
