#!/usr/bin/env python
# Tool to convert a binary GT trajectory to a list of poses
# apply and offset and rescaling to bring into alignment
# with the floor plans. Writes out a CSV file

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import numpy as np
import sys
import csv
import time
from nav_msgs.msg import Path
import msgpack
import math

from challenge_tools_lib import ReactPose
from challenge_tools_lib import ReactTime
from challenge_tools_lib import ChallengeToolsLib


class GTPub(Node):
    def __init__(self):
        super().__init__('groundtruth_publisher')
        self.pub_path = self.create_publisher(Path, '/gt_poses', 10)
        self.fixed_frame = "map"

        self.ctl = ChallengeToolsLib()
        self.ctl.fixed_frame = self.fixed_frame

    def read_aligned_frame_trajectory_3d_2d_bin_file(self, path: str):
        """
        Load frame trajectory aligned data from a MessagePack binary file.
        :param path: Path to the file containing frame trajectory aligned data.
        :return: List of tuples (timestamp, x, y, z, qx, qy, qz, qw, xn, yn, angle).
        """
        aligned_frame_trajectory_3d_2d = []
        with open(path, "rb") as f:
            data = msgpack.unpackb(f.read(), raw=False)
            for item in data:
                aligned_frame_trajectory_3d_2d.append(tuple(item))

        # Convert to the ReactPose type
        poses = []
        for entry in aligned_frame_trajectory_3d_2d:
            this_pose = ReactPose()
            this_pose.pos = [entry[1], entry[2], entry[3]]
            this_pose.quat = [entry[4], entry[5], entry[6], entry[7]]

            ts = entry[0]
            #print(ts)
            sec = math.floor(ts)#index#line[0]
            nsec = math.floor((ts - sec)*1000000000.0)  #line[1]
            #print(sec, nanosec)
            this_time = ReactTime(sec, nsec)

            pose_t = (this_time, this_pose)
            poses.append(pose_t)

        return poses

    def do_work(self):
        this_run = "runA"

        # All floors except for UG
        # 7351 x 4206 = 73.51m x 42.06m
        pixel_size = [7351, 4206] # resolution of the occupancy map / floor plan from Samuele
        gt_pixel_size = [2097, 1200] # resolution of the GT file from Trimble

        if (this_run == "runA"):
            run_name = "floor_1_2025-05-05_run_1"
            # pix0 is the from the <<top>> left of the image
            # "translation" in Trimble json. Coordinates of first data point in the image
            pix0 = [0.4978540772532189, 0.5141666666666667] 
        elif (this_run == "runB"):
            run_name = "floor_2_2025-05-05_run_1"
            pix0 = [0.5007153075822603, 0.5016666666666667]
        elif (this_run == "runC"):
            run_name = "floor_2_2025-10-28_run_1"
            pix0 = [0.542203147353362, 0.705]
        elif (this_run == "runD"):
            run_name = "floor_2_2025-10-28_run_2"
            pix0 = [0.5755841678588459, 0.72]
        elif (this_run == "runE"):
            run_name = "floor_UG_2025-10-16_run_1"
            pix0 = [0.49351415094339623, 0.4225]
            # UG is different
            pixel_size = [11890, 8411]
            gt_pixel_size = [1696, 1200]

        try:
            path = get_package_share_directory("challenge_tools_ros")
            print("share dir:", path)
        except PackageNotFoundError:
            print(f"Package challenge_tools_ros not found in the ament index.")

        main_folder = path + "/groundtruth/"
        full_filename = main_folder + "/" + run_name + "/" + "aligned_frame_trajectory_3d_2d_gt.bin"
        full_filename_out = main_folder + "/" + run_name + "/" + "gt_poses.csv"

        gt_poses = self.read_aligned_frame_trajectory_3d_2d_bin_file(full_filename)
        print (gt_poses[0])
        print (gt_poses[1])
        print (len(gt_poses))


       

        pixel_to_metric_scale = 100. # 100. # 1 pixel is 1cm. This is common to all floors


        # Dimensions of Occupancy Map
        metric_size = [pixel_size[0]/pixel_to_metric_scale, pixel_size[1]/pixel_to_metric_scale]
        print(metric_size)

        # Multiplier to go from - 0.035050000000000005
        gt_to_metric_scaling_x = metric_size[0]/gt_pixel_size[0]
        print(gt_to_metric_scaling_x, "gt_to_metric_scaling_x")
        gt_to_metric_scaling_y = metric_size[1]/gt_pixel_size[1]
        print(gt_to_metric_scaling_y, "gt_to_metric_scaling_y")

        # Position of first point - wrt to <<bottom>> left of the metric map
        pt0= [pix0[0] * metric_size[0], (1-pix0[1]) * metric_size[1]]
        print (pt0 , "pt0 in metric space")

        # make the GT poses metric scaled (but still offset from the map)
        for index, pose in enumerate(gt_poses):
            pose_t = gt_poses[index]
            pose = pose_t[1]
            pose.pos[0] = pose.pos[0]*gt_to_metric_scaling_x
            pose.pos[1] = pose.pos[1]*gt_to_metric_scaling_x
            pose.pos[2] = pose.pos[2]*gt_to_metric_scaling_x # scaling z is a guess
            pose_t_out = [pose_t[0], pose]
            gt_poses[index] = pose_t_out

        # Determine the offset to apply to move all poses to correct offset
        offset_to_pt0 = [pt0[0] - gt_poses[0][1].pos[0], pt0[1] - gt_poses[0][1].pos[1]]
        print (offset_to_pt0, "offset_to_pt0")

        # Produce the correct offset
        for index, pose in enumerate(gt_poses):
            pose_t = gt_poses[index]
            pose = pose_t[1]
            pose.pos[0] = pose.pos[0] + offset_to_pt0[0]
            pose.pos[1] = pose.pos[1] + offset_to_pt0[1]
            pose_t_out = [pose_t[0], pose]
            gt_poses[index] = pose_t_out


        self.ctl.write_csv_poses(full_filename_out, gt_poses)

        while (1==1):
            print("pub")
            self.ctl.publish_trajectory(self.pub_path, gt_poses)
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    app = GTPub()
    app.do_work()
    app.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
