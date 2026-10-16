[Home](../README.md) · [Documentation guide](../README.md#documentation)

<a name="openvins-example"></a>
# OpenVINS Example

We have tested and configured [OpenVINS](https://github.com/rpng/open_vins), a leading open-source visual-inertial odometry system, to run on our dataset.

[Here](https://www.youtube.com/watch?v=GsbvdB7MyZE) is a video demonstrating OpenVINS alongside the ground truth trajectory. It illustrates the LiDAR-based GT in detail and shows how the inertial visual odometry (VO) from OpenVINS aligns with it.

Note that the OpenVINS trajectory has only been aligned to ground-truth pose at the start of the recording (approximately t = 10005 sec). The video is not explicitly localizing against the floor plan.

<div align="center" style="display: flex; gap: 12px; justify-content: center; flex-wrap: wrap;">
  <img src="../media/floor1_gt.png" alt="floor1" style="width: 48%; max-width: 520px;"/>
  <img src="../media/floor1_points.png" alt="floor1" style="width: 48%; max-width: 520px;"/>
</div>

## Before You Start

Complete [tool installation](challenge-tools.md#install-challenge-tools) and download `floor_1_2025-05-05_run_1`. The commands below use the workspace at `~/ros2_ws`.

<a name="install-openvins-with-ros2"></a>
## Installing OpenVINS with ROS2

We provide minimal instructions for installing OpenVINS with ROS2. More detailed information can be found in the [official documentation](https://docs.openvins.com/getting-started.html).

First, install the dependencies:

```bash
sudo apt update
sudo apt install libeigen3-dev libboost-all-dev libceres-dev
```

Then install OpenVINS:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Hilti-Research/open_vins.git
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

<a name="run-openvins"></a>
## Running OpenVINS

We provide ready-to-use scripts and a calibrated configuration file for our device. [Prepare each terminal](challenge-tools.md#prepare-each-terminal) after building OpenVINS. Replace `/path/to/data` below with your download directory.

Start terminals 1–3 before playing the bag in terminal 4.

In **Terminal 1**, start OpenVINS and Rviz:

```bash
ros2 launch challenge_tools_ros run_openvins.launch.py
```

In **Terminal 2**, publish the floor plan and subscribe to the OpenVINS output to align it with the provided initial ground-truth pose at t = 10005 sec (check [static_transform_publisher.py](../challenge_tools_ros/gt_helper/static_transform_publisher.py)):

```bash
ros2 launch challenge_tools_ros map_server.launch.py mask:=masks_with_windows run_name:=floor_1_2025-05-05_run_1
```

In **Terminal 3**, visualize the ground-truth trajectory if provided:

```bash
ros2 launch challenge_tools_ros groundtruth_server.launch.py run_name:=floor_1_2025-05-05_run_1
```

In **Terminal 4**, play the bag (you can press the `space` key to pause and resume):

```bash
ros2 bag play /path/to/data/floor_1/2025-05-05/run_1/rosbag -p
```

---

Next: [understand the reference poses](ground-truth.md) or [prepare a trajectory submission](challenge-evaluation.md#what-to-deliver).
