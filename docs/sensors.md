[Home](../README.md) · [Documentation guide](../README.md#documentation)

<a name="sensor-data-ros-bags"></a>
# Sensors, ROS Topics, and Calibration

Use this page to interpret recorded messages and select the calibration for your SLAM system. For downloads, see the [dataset guide](dataset.md). For setup and playback, see [ROS image tools](challenge-tools.md).

<a name="topics-description"></a>
## Topic Descriptions

The provided recordings are ROS 2 bags. The dataset contains the following topics:

| Topic Name | Message Type | Frequency | Description |
| :--- | :--- | :--- | :--- |
| `/cam0/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 30 Hz | Front Camera data (RGB, 1472×1440 resolution). Compressed to minimize file size. |
| `/cam1/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 30 Hz | Back Camera data (RGB, 1472×1440 resolution). Compressed to minimize file size. |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 1000 Hz | Raw IMU data. |

<a name="humble-compatibility"></a>
### Humble Compatibility

> [!TIP]
> **Humble + Ubuntu 22.04 Compatibility Fix**
>
> If you encounter playback errors, update your configuration of the `metadata.yaml` file of each rosbag :
>
> Replace:
>   ```
>   offered_qos_profiles:
>      []
>   ```
> With:
>   ```
>   offered_qos_profiles:
>      ""
>   ```

<a name="constant-timestamp-shift"></a>
## Constant Timestamp Shift

In Insta360, the IMU recordings slightly precede the cameras. To avoid negative IMU timestamps, we applied a constant timestamp shift of `1e4` seconds to all runs. Therefore, all image message timestamps start at t = 10000 sec.

<a name="devices"></a>
## Devices

<a name="camera-visual-inertial-sensor"></a>
### Camera (Visual-Inertial Sensor)

Each run was captured using the **Insta360 One-RS 1-Inch Edition** ([Insta360 website](https://www.insta360.com/product/insta360-oners/1inch-360)).

- Sensors and Lenses: The camera is equipped with two 1-inch CMOS sensors (co-engineered with Leica). Each lens is a Leica Summicron-A 6.52 mm f/2.2 fisheye, providing an extremely wide field of view (approx. 200° per lens) for full 360° coverage.

- Shutter: The sensor uses an electronic (rolling) shutter. The rolling shutters of the two cameras are well synchronized, including their exposure timings.

- Integrated IMU: The camera includes a 6-axis Inertial Measurement Unit (3-axis gyroscope + 3-axis accelerometer). IMU data is internally recorded and embedded into the video metadata.

<a name="lidar-ground-truth-acquisition"></a>
### LiDAR (Ground Truth Acquisition)

A high-accuracy ground truth trajectory was acquired using LiDAR-Inertial SLAM. We use these trajectories to evaluate your SLAM algorithm.

A LiDAR-Inertial mapping device containing a **Hesai XT32 LiDAR** ([Hesai website](https://www.hesaitech.com/product/xt16-32-32m/)) was rigidly attached to the Insta360 during the runs. This sensor is used only to generate the reference trajectory and its data is not included in the provided ROS 2 bags.

<a name="camera-and-imu-calibration"></a>
## Camera and IMU Calibration

We provide camera and IMU calibration results for both the _pinhole equidistant_ camera model and the _enhanced unified_ camera model (EUCM) [here](https://huggingface.co/datasets/Hilti-Research/hilti-trimble-slam-challenge-2026/tree/main/camera_calibration). The original calibration runs are also available in the [public dataset](https://huggingface.co/datasets/Hilti-Research/hilti-trimble-slam-challenge-2026/tree/main/camera_calibration).

Calibration for the EUCM was performed using the open-source [Kalibr](https://github.com/ethz-asl/kalibr) toolbox. Since pinhole–equidistant camera model is widely supported by most existing SLAM systems, we fit its parameters to the EUCM intrinsics, as Kalibr’s implementation was unable to successfully calibrate such large-FOV fisheye cameras using this model. In addition, we have added support for EUCM to [OpenVINS](https://github.com/Hilti-Research/open_vins) so as to demonstrate that OpenVINS can process all of these recordings.

Repository configurations: [OpenVINS](../config/hilti_openvins/) · [rolling shutter parameters](../config/hilti_openvins/rs_config.yaml) · [Stella-VSLAM](../config/hilti_stella_vslam/).

---

Related: [ground truth synchronization](ground-truth.md#sensors-synchronization) · [image tools](challenge-tools.md).
