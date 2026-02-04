<div align="center">

<h1>Hilti × Trimble 360 Visual-Inertial SLAM Challenge 2026</h1>

<p>
  <a href="https://scholar.google.com/citations?hl=it&user=677n63kAAAAJ">Samuele Centanni</a><sup>*</sup>,
  <a href="https://scholar.google.com/citations?hl=en&user=8afWaIUAAAAJ">Yuhao Zhang</a><sup>*</sup>,
  <a href="https://scholar.google.com/citations?hl=en&user=T0M78kEAAAAJ">Yifu Tao</a>,
  <a href="https://scholar.google.com/citations?hl=en&user=lVIr3qcAAAAJ">Julien Kindle</a>,
  <a href="https://scholar.google.com/citations?hl=en&user=Vxwg0vcAAAAJ">Frank Neuhaus</a>,
  <a href="https://scholar.google.com/citations?hl=en&user=d1xlF6AAAAAJ">Tilman Koß</a>,
  <a href="https://scholar.google.com/citations?hl=en&user=8I_26uEAAAAJ">Aryaman Patel</a>,
  <br>
  <a href="https://scholar.google.com/citations?hl=en&user=ztmyFyUAAAAJ">Michael Helmberger</a>,
  <a href="https://scholar.google.com/citations?hl=en&user=vxENFQkAAAAJ">Emilia Szymańska</a>,
  <a href="https://scholar.google.com/citations?user=-h1gOMkAAAAJ&hl=en&oi=ao">Torben Gräber</a>,
  <a href="https://scholar.google.com/citations?hl=en&user=BqV8LaoAAAAJ">Maurice Fallon</a>
  <br>
  <em><sup>*</sup> Equal contribution</em>
</p>

[<img src="https://img.shields.io/badge/Home_Page-red" alt="Home Page">](https://hilti-trimble-challenge.com)
[<img src="https://img.shields.io/badge/Dataset-4285F4?logo=googledrive&logoColor=white" alt="Dataset">](https://drive.google.com/drive/u/1/folders/1BWFIfEL40Nvj-yeyre5O9dOiYCTWatv5)
[<img src="https://badges.aleen42.com/src/youtube.svg" alt="Video">](https://www.youtube.com/playlist?list=PLIbIX8ez6UWS0N2A-lTmwYnmhMPQs8IYU)

<p>
  <a href="https://hilti-trimble-challenge.com/">
    <img src="media/HTSC2026.png?raw=true" alt="HTSC2026" width="90%">
  </a>
</p>

</div>

If you find this dataset useful, please consider giving it a ⭐ and citing it in your work.
```bibtex
@online{slamchallenge2026,
   title = {{Hilti}-{Trimble}-{Oxford} Dataset: 360 Visual-Inertial Benchmark with Floor Plan Priors for SLAM and Localization},
   year = {2026},
   url = {https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026},
   author = {Centanni, Samuele and Zhang, Yuhao and Tao, Yifu and Kindle, Julien and Neuhaus, Frank and Koß, Tilman and Patel, Aryaman and Helmberger, Michael and Szymańska, Emilia and Gräber, Torben and Fallon, Maurice},
   urldate = {2026-02-01}
}
```

<!-- TOC start (generated with https://github.com/derlin/bitdowntoc) -->
# Table of Contents

- [Challenge Homepage & Evaluation Server](#challenge-homepage-evaluation-server)
- [Introduction: 360 Visual-Inertial SLAM Challenge with Additional Building Floorplans](#introduction-360-visual-inertial-slam-challenge-with-additional-building-floorplans)
- [Challenge & Evaluation](#challenge-evaluation)
   * [What to Deliver](#what-to-deliver)
   * [Metric](#metric)
      + [Notes](#notes)
      + [Final Score](#final-score)
   * [Evaluation Server](#evaluation-server)
- [Dataset](#dataset)
   * [Dataset Floorplans](#dataset-floorplans)
   * [Dataset Properties](#dataset-properties)
   * [Ground Truth](#ground-truth)
   * [Initial Localization in Floorplan](#initial_localization)
   * [Sensor Data & ROS Bags](#sensor-data-ros-bags)
      + [Topics Description](#topics-description)
      + [Constant Timestamp Shift](#constant-timestamp-shift)
   * [Devices](#devices)
      + [Camera (Visual-Inertial Sensor)](#camera-visual-inertial-sensor)
      + [LiDAR (Ground Truth Acquisition)](#lidar-ground-truth-acquisition)
      + [Camera and IMU Calibration](#camera-and-imu-calibration)
- [Challenge Tools ROS](#challenge-tools-ros)
   * [Install Challenge Tools](#install-challenge-tools)
   * [Image Decompression](#image-decompression-1)
   * [Image Inversion](#image-inversion)
   * [OpenVINS Example](#openvins-example)
      + [Install OpenVINS with ROS2](#install-openvins-with-ros2)
      + [Run OpenVINS](#run-openvins)
   * [Stella-VSLAM Example](#stella-vslam-example)
      + [Run Stella-VSLAM](#run-stella-vslam)

<!-- TOC end -->


<!-- TOC --><a name="challenge-homepage-evaluation-server"></a>
# Challenge Homepage & Evaluation Server

You can find the **official challenge homepage** for this challenge at https://hilti-trimble-challenge.com. It includes an _evaluation server_ and _prize money_ information.


<!-- TOC --><a name="introduction-360-visual-inertial-slam-challenge-with-additional-building-floorplans"></a>
# Introduction: 360 Visual-Inertial SLAM within a Building Floorplan

Accurate and reliable SLAM and localization is crucial in robotics, automation, and digital construction workflows. Yet real-world construction sites remain amoung the most challenging environments for modern Visual-Inertial SLAM: changing lighting conditions, repetitive texture and ongoing building works push state-of-the-art algorithms to their limits.

To advance the field and to establish a common benchmark for these demanding scenarios, [Hilti](http://hilti.com/), [Trimble](https://www.trimble.com/), and the [Dynamic Robot Systems Group](https://dynamic.robots.ox.ac.uk/) of University of Oxford have joined forces to launch the SLAM Challenge 2026.
Our intention is to provide an open and realistic dataset, captured directly on active construction sites, to evaluate and compare SLAM systems in real world conditions.

The winners of the challenge will be announced at [ICRA](https://2026.ieee-icra.org/) on **5th June 2026** in Vienna Austria, as part of the [OCRAIM workshop](https://ariamhub.com/ocraim/).

<table width="100%">
  <tr>
    <td align="center" width="33%">
      <img src="media/indoor_1.jpeg" alt="Indoor 1">
      <br>
      <em>Indoor 1</em>
    </td>
    <td align="center" width="33%">
      <img src="media/indoor_2.jpeg" alt="Indoor 2">
      <br>
      <em>Indoor 2</em>
    </td>
    <td align="center" width="33%">
      <img src="media/outdoor.jpeg" alt="Outdoor">
      <br>
      <em>Outdoor View</em>
    </td>
  </tr>
</table>


<!-- TOC --><a name="challenge-evaluation"></a>
# Challenge & Evaluation

We challenge participants with two different tasks. You are free to participate in one or both of these tasks.

The two tasks are the following:

1. **SLAM**: we ask you to estimate the trajectory of the Insta360 camera - in **any reference frame**,
2. **Localization**: we ask you to estimate the camera trajectory in the **floorplan reference frame** (map coordinate frame). We consider the bottom-left pixel of the floorplan as the $(0,0)$ coordinate.

<!-- TOC --><a name="what-to-deliver"></a>
## What to Deliver

For each run, we require your trajectory estimates to achieve **at least $99$**% coverage of the ground truth trajectory. **Otherwise**, the score for that run will be automatically set to 0.

The coverage is computed as $\dfrac{\text{|matched preditected poses|}}{\text{|ground truth poses|}} * 100$.

Predictions should be submitted using the **TUM** format:  

```
timestamp tx ty tz qx qy qz qw
```

Each file must be named as follows:

```
floor_X_YYYY-MM-DD_run_Z.txt
```

where:

- $X$ is the floor number, i.e. $1, 2, 3, 4, 5, 6, 7, \text{EG}, \text{UG1}, \text{UG2}$
- $\text{YYYY, MM, DD}$ are the year, month and day of the run respectively
- $Z$ is the run number on that day, i.e. $1, 2$

<!-- TOC --><a name="metric"></a>
## Scoring Metric

You will be evaluated using an exponential score --- where a higher score corresponds with higher accuracy.

The score is computed using this formula:

$$ \text{score} = \dfrac1N \sum_{t=1}^N a \cdot e^{-c \cdot \text{error}_t} $$

with the constants

$$a = 100.0$$

$$c = 0.46051701859880917$$

$N$ is the number of poses in that run and $t$ is the $t-th$ pose. The score is **capped at 100 for each run**.

The final score will be then the sum of each run score:

$$\text{finalScore} =  \sum_{i=1}^{25} \text{score}_i $$

<!-- TOC --><a name="notes"></a>
### Notes

1) The values $a$ and $c$ were obtained by fitting an exponential function such that:

- when the error is **0 m**, the score is **100**
- when the error is **10 m**, the score is **1**

2) Our LiDAR ground truth does not start at exactly the starting point of the camera recording, as a result we **will not evaluate the trajectory of the first 5 seconds of each run**.

3) When testing your solution with our evaluation server you will receive a report showing the error plots for your entry. We decided to hide details for six of the runs. For these six runs we will show you only the score **but we will not show the error plots for these six runs**.

4) In the **Localization task** we will not evaluate the _z-coordinate_.

5) **Important**: we will not adjust the trajectory scale (when using Evo), therefore you have to ensure that your trajectories are properly scaled.

<!-- TOC --><a name="final-score"></a>
### Final Score

We provide 30 recordings to test, but we will evaluate only those that **were not** part of the Early Release made in November. This is because we provided the Ground Truth for these 5 Early Release runs (see [Ground Truth](#ground-truth) section).

As a result, for the **SLAM** task we will evaluate 25 out of 30 runs, and the maximum achievable total score will be **2500 points**.

For the **Localization** task we also will exclude `floor_UG2-2025-12-02_run_1` as we do not provide a floorplan for this recording. As a result, the maximum achievable total score for localization task will be **2400 points** (24 runs out of 30).

<!-- TOC --><a name="evaluation-server"></a>
## Evaluation Server

You can submit your entry using our [evaluation server](https://submit.hilti-challenge.com/). Good Luck!


<!-- TOC --><a name="dataset"></a>
# The Challenge Dataset

You can find the public folder with all 30 samples [here](https://drive.google.com/drive/folders/19mRLGJYYAJAvC-qnPyimRLQAV8RONp9f?usp=drive_link).

In order to simplify the download, we provide two python scripts in the [download_data](challenge_tools_ros/download_data/) folder.



The structure of the dataset folder is as follows:

```
data/
└── floor_X/
    └── YYYY-MM-DD/
        ├── run_Z/
            ├── rosbag/
            │   └── rosbag.db3 # ROS 2 bag file
            │   └── metadata.yaml
```

In the table below we show the trajectories of 5 runs. If you want to have an easier look at all the 30 runs, check all the videos on the [youtube page](https://www.youtube.com/playlist?list=PLIbIX8ez6UWS0N2A-lTmwYnmhMPQs8IYU).

| Youtube Video | Rosbag Folder | Ground Truth Image |
| :---: | :---: | :---: |
| [floor_1_2025-05-05_run_1](https://www.youtube.com/watch?v=dGeTArx1ah0) | [rosbag](https://drive.google.com/drive/folders/1LR_82lGppMEIL1cqjKb9FDIisKgRsKif?usp=drive_link) | <img src="media/GT_plots_early_release/floor_1_gt.png" alt="GT Floor 1" width="400"/> |
| [floor_2_2025-05-05_run_1](https://www.youtube.com/watch?v=nW9fxv75HOs) | [rosbag](https://drive.google.com/drive/folders/1pswcr7wUIUVsr5Y13-BNre9vuABihHsT?usp=drive_link) | <img src="media/GT_plots_early_release/floor_2_gt.png" alt="GT Floor 1" width="400"/> |
| [floor_2_2025-10-28_run_1](https://www.youtube.com/watch?v=fEBxu02EnSs) | [rosbag](https://drive.google.com/drive/folders/1LebGR8riGU9QL_XfaYM9VJBrLIaI7q4u?usp=drive_link) | <img src="media/GT_plots_early_release/floor_2_gt_run1.png" alt="GT Floor 1" width="400"/> |
| [floor_2_2025-10-28_run_2](https://www.youtube.com/watch?v=EzEfg7PvjD4)| [rosbag](https://drive.google.com/drive/folders/1USazXLkD4uuwFw97VxbjFEr9WAn91kMW?usp=drive_link) | <img src="media/GT_plots_early_release/floor_2_gt_run2.png" alt="GT Floor 1" width="400"/> |
| [floor_UG1_2025-10-16_run_1](https://www.youtube.com/watch?v=BGmqwdSTStU) | [rosbag](https://drive.google.com/drive/folders/1pB3829XSki2ScQeuWYG-myMco31n594m?usp=drive_link) | <img src="media/GT_plots_early_release/floor_UG_gt.png" alt="GT Floor 1" width="400"/> |

> [!NOTE]
> Please, if you have any doubt or issue refer to the **[Issues section](https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026/issues)**.

<!-- TOC --><a name="dataset-floorplans"></a>
## Dataset Floorplans

We provide the building floorplans in as PNG images, which can be converted into ROS2 Occupancy Maps and viusalised in Rviz using the following [script](/challenge_tools_ros/challenge_tools_ros/runtime_helper/map_server.py).

The floorplans are stored in the repo [here](floorplans). We provide two types of floorplan - **with and without windows**. Both formats show the main columns and walls of the building site. 

<div align="center">
  <img src="floorplans/masks_no_windows/floor_1.png" alt="floor_1 floorplan - no_window" width="50%"/>
</div>

The floorplans show the intended state of the building at the end of construction. During construction, the walls and columns are successively built which means that not all elements are present in the earlier recordings.

This image shows a ground truth trajectory aligned to the floor 1 floorplan.

<div align="center">
  <img src="media/floorplan_vs_pcd.png" alt="floorplan vs pcd" width="50%"/>
</div>


<!-- TOC --><a name="dataset-properties"></a>
## Dataset Properties

The dataset captures several realistic real-world challenges. These include:

- _Dynamic Initialization_: some runs start with **intentional** motion at the beginning of the run. This is a realistic challenge for this application.
- _Imprecise floorplans_: construction sites always contain deviations in their as-built state versus the as-planned state.

Algorithms which are robust to these deviations and to dynamic initialization will be more useful in real-world construction sites.

An example of deviation is shown in the following - the real wall is shorter than the floorplan:

<div align="center">
  <img src="media/floorplan_pcd_devation.png" alt="floorplan vs pcd" width="50%"/>
</div>

In general, you can expect the floorplans to have an accuracy of 1cm relative to the real world.

<!-- TOC --><a name="ground-truth"></a>
## Ground Truth

Ground truth is provided in `cam0->map` convention (the transformation that expresses the pose of `cam0` in the `map` frame) for five early release runs. You can find them [here](https://drive.google.com/drive/folders/1i20VagJDaM7hefWAlGFZ4kRAoehxAQAK?usp=drive_link).

They were acquired using a [LiDAR](#lidar-ground-truth-acquisition) system, and transformed using LiDAR-camera extrinsics and IMU time alignment. Since the LiDAR starts slightly later than the camera, we will not evaluate the trajectory during the first 5 seconds of each run.

<!-- TOC --><a name="initial_localization"></a>
## Initial Localization in Floorplan
To provide an initial localization estimate (for the localization task) we provide the ground-truth camera pose at approximately t = 10005 sec at [here](groundtruth/init_gt_poses.csv) (the start camera timestamp is t = 10000 sec).

<!-- TOC --><a name="sensor-data-ros-bags"></a>
## Sensor Data & ROS Bags

<!-- TOC --><a name="topics-description"></a>
### Topic Descriptions

The provided recordings are ROS 2 bags. The dataset contains the following topics:
| Topic Name | Message Type | Frequency | Description |
| :--- | :--- | :--- | :--- |
| `/cam0/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 30 Hz | Front Camera data (RGB, 1472×1440 resolution). Compressed to minimize file size. |
| `/cam1/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 30 Hz | Back Camera data (RGB, 1472×1440 resolution). Compressed to minimize file size. |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1000 Hz | Raw IMU data. |

<!-- TOC --><a name="constant-timestamp-shift"></a>
### Constant Timestamp Shift
In Insta360, the IMU recordings slightly preceed the cameras. To avoid negative IMU timestamps, we applied a constant timestamp shift of `1e4` seconds to all runs. Therefore, all image message timestamps start at t = 10000 sec.

<!-- TOC --><a name="devices"></a>
## Devices


<!-- TOC --><a name="camera-visual-inertial-sensor"></a>
### Camera (Visual-Inertial Sensor)

Each run was captured using the **Insta360 One-RS 1-Inch Edition** ([Insta360 website](https://www.insta360.com/product/insta360-oners/1inch-360)).

- Sensors and Lenses: The camera is equipped with two 1-inch CMOS sensors (co-engineered with Leica). Each lens is a Leica Summicron-A 6.52 mm f/2.2 fisheye, providing an extremely wide field of view (approx. 200° per lens) for full 360° coverage.

- Shutter: The sensor uses an electronic (rolling) shutter.

- Integrated IMU: The camera includes a 6-axis Inertial Measurement Unit (3-axis gyroscope + 3-axis accelerometer). IMU data is internally recorded and embedded into the video metadata.


<!-- TOC --><a name="lidar-ground-truth-acquisition"></a>
### LiDAR (Ground Truth Acquisition)

A high-accuracy ground truth trajectory was acquired using LiDAR-Inertial SLAM. We use these trajectories to evaluate your SLAM algorithm.

A LiDAR-Inertial mapping device containing a **Hesai XT32 LiDAR** ([Hesai website](https://www.hesaitech.com/product/xt16-32-32m/)) was rigidly attached to the Insta360 during the runs. This sensor is used only to generate the reference trajectory and its data is not included in the provided ROS 2 bags.


<!-- TOC --><a name="camera-and-imu-calibration"></a>
### Camera and IMU Calibration

We provide camera and IMU calibration results for both the _pinhole equidistant_ camera model and the _enhanced unified_ camera model (EUCM) [here](https://drive.google.com/drive/folders/1kYxgaCAtsVLe1B1MGsc2kR6RnByloHUV?usp=sharing). The original calibration runs are also available in the [public drive](https://drive.google.com/drive/folders/1kYxgaCAtsVLe1B1MGsc2kR6RnByloHUV?usp=drive_link). 

Calibration for the EUCM was performed using the open-source [Kalibr](https://github.com/ethz-asl/kalibr) toolbox. For the pinhole equidistant camera model, we used an internal calibration tool, as Kalibr’s implementation was unable to successfully calibrate such large-FOV fisheye cameras using that model. In addition, we have added support for EUCM to [OpenVINS](https://github.com/Hilti-Research/open_vins) so as to demonstrate that OpenVINS can process all of these recordings.


<!-- TOC --><a name="challenge-tools-ros"></a>
# Challenge Tools ROS

We provide several tools to make it easier to work with SLAM Challenge 2026 data. These tools were developed using ROS2 Jazzy and Ubuntu 24.04.


<!-- TOC --><a name="install-challenge-tools"></a>
## Install Challenge Tools

First, create a ROS2 workspace (if you have not created one before), and then install the challenge tools:

```
mkdir ~/ros2_ws/src -p
cd ~/ros2_ws/src
git clone https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026.git
cd ~/ros2_ws
colcon build --symlink-install
```

<!-- TOC --><a name="image-decompression-1"></a>
## Image Decompression
Images are stored as the `CompressedImage` type in our ROS2 bags. We provide a ready-to-run node to perform decompression:
```
ros2 run challenge_tools_ros image_conversion_node.py /cam0/image_raw/compressed /cam1/image_raw/compressed /cam0/image_raw /cam1/image_raw
```

<!-- TOC --><a name="image-inversion"></a>
## Image Inversion

The image sensors in the cameras are inverted. We have not artificially inverted the recorded images (as that would interfere with rolling shutter correction). For easier visualization we provide a simple [tool](challenge_tools_ros/runtime_helper/rotate_image_180.py) to rotate the image by 180 degrees:

```
ros2 run challenge_tools_ros rotate_image_180.py /cam0/image_raw
```

The rotated topic will be published to `/cam0/image_raw/rotated`, which you can visualize in RViz.

Optional arguments:
- Add `compressed` as second argument if the input is a `CompressedImage` topic
- Add `gui` as third argument to show a preview window

<!-- TOC --><a name="openvins-example"></a>
## OpenVINS Example
We have tested and configured [OpenVINS](https://github.com/rpng/open_vins), a leading open-source visual-inertial odometry system, to run on our dataset.

[Here](https://www.youtube.com/watch?v=GsbvdB7MyZE) is a video demonstrating OpenVINS along side the ground truth trajectory. It illustrates the LiDAR-based GT in detail and shows how the inertial visual odometry (VO) from OpenVINS aligns with it. 

Note that the OpenVINS trajectory has only been aligned to ground-truth pose at the start of the recording (approximately t = 10005 sec). The video is not explicitly localizing against the floor plan.

<div align="center" style="display: flex; gap: 12px; justify-content: center; flex-wrap: wrap;">
  <img src="media/floor1_gt.png" alt="floor1" style="width: 48%; max-width: 520px;"/>
  <img src="media/floor1_points.png" alt="floor1" style="width: 48%; max-width: 520px;"/>
</div>

<!-- TOC --><a name="install-openvins-with-ros2"></a>
### Installing OpenVINS with ROS2
We provide minimal instructions for installing OpenVINS with ROS2. More detailed information can be found in the [official documentation](https://docs.openvins.com/getting-started.html).

First, install the dependencies:
```
sudo apt update
sudo apt install libeigen3-dev libboost-all-dev libceres-dev
```

Then install OpenVINS:
```
cd ~/ros2_ws/src
git clone https://github.com/Hilti-Research/open_vins.git
cd ~/ros2_ws
colcon build --symlink-install
```


<!-- TOC --><a name="run-openvins"></a>
### Running OpenVINS

We provide ready-to-use scripts and a calibrated configuration file for our device.

In **Terminal 1**, start OpenVINS and Rviz:

```
ros2 launch challenge_tools_ros run_openvins.launch.py
```

In **Terminal 2**, publish the floor plan and subscribe to the OpenVINS output to align it with the provided initial ground-truth pose at t = 10005 sec (check [static_transform_publisher.py](challenge_tools_ros/gt_helper/static_transform_publisher.py)):

```
ros2 launch challenge_tools_ros map_server.launch.py mask:=masks_with_windows run_name:=floor_1_2025-05-05_run_1
```

In **Terminal 3**, play the bag (you can press the `space` key to pause and resume):

```
ros2 bag play floor_1/2025-05-05/run_1/rosbag -p
```

In **Terminal 4**, visualize the ground-truth trajectory if provided:

```
ros2 launch challenge_tools_ros groundtruth_server.launch.py run_name:=floor_1_2025-05-05_run_1
```

<!-- TOC --><a name="stella-vslam-example"></a>
## Stella-VSLAM Example

Stella-VSLAM is another open-source vSLAM system. It directly processes 360° equirectangular images. To install it, follow the instructions [here](https://stella-cv.readthedocs.io/en/latest/ros2_package.html).


<!-- TOC --><a name="run-stella-vslam"></a>
### Run Stella-VSLAM

In **Terminal 1**, start Stella-VSLAM:

```
ros2 launch challenge_tools_ros run_stella_vslam.launch.py \
    image_conversion:=false \
    config_path:=/path/to/challenge_tools_ros/config/hilti_stella_vslam/fisheye.yaml
```

By default, the `config_path` is set to `fisheye.yaml`, which is the monocular configuration for the front camera. <br>

Since we only provide raw sensor data from the Insta360, which contains separate images from the front and back lenses, you would need to stitch the images from both lenses if you are interested in 360° equirectangular SLAM. You must also modify the `equirectangular.yaml` file according to the resolution of your stitched image and set the `config_path`.

An example of Stella-VSLAM running on stitched images is shown below:

<div align="center">
  <img src="media/floor1_stella_vslam_stitch.png" alt="floor1_stella_vslam_stitch" width="50%"/>
</div>

In **Terminal 2**, launch the `image_conversion_node` if the bag stores images as `CompressedImage`:

```
ros2 run challenge_tools_ros image_conversion_node.py /cam0/image_raw/compressed /camera/image_raw
```

Here, `/cam0/image_raw/compressed` is the compressed image topic in the bag, and `/camera/image_raw` is the image topic that Stella-VSLAM subscribes to. 

In **Terminal 3**, play the ros bag:

```
ros2 bag play /path/to/ros2/bag/folder -p
```

**Known Issue:** Sometimes the Stella-VSLAM ROS wrapper may fail to properly subscribe to image messages. If this happens, terminate the `image_conversion_node` in Terminal 2 using `Ctrl+C` and relaunch it.
