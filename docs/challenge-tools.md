[Home](../README.md) · [Documentation guide](../README.md#documentation)

<a name="challenge-tools-ros"></a>
# ROS Image Tools

We provide several tools to make it easier to work with SLAM Challenge 2026 data. These tools were developed using ROS2 Jazzy and Ubuntu 24.04.

<a name="install-challenge-tools"></a>
## Install the Tools

The commands below assume ROS 2 Jazzy and `colcon` are already installed and use Bash.

Create a workspace, clone the repository, and build it:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026.git
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

If you already have a ROS workspace and checkout, use those paths instead. Required ROS dependencies are declared in [package.xml](../package.xml); the build also requires the libraries listed in [CMakeLists.txt](../CMakeLists.txt), including OpenCV, PCL, and Eigen3.

## Prepare Each Terminal

Run these commands in **every new terminal** used for the tools or examples:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

<a name="image-decompression-1"></a>
## Image Decompression

Images are stored as the `CompressedImage` type in our ROS2 bags. We provide a ready-to-run node to perform decompression:

```bash
ros2 run challenge_tools_ros image_conversion_node.py /cam0/image_raw/compressed /cam1/image_raw/compressed /cam0/image_raw /cam1/image_raw
```

<a name="image-inversion"></a>
## Image Inversion

The image sensors in the cameras are inverted. We have not artificially inverted the recorded images (as that would interfere with rolling shutter correction). For easier visualization we provide a simple [tool](../challenge_tools_ros/runtime_helper/rotate_image_180.py) to rotate the image by 180 degrees:

```bash
ros2 run challenge_tools_ros rotate_image_180.py /cam0/image_raw
```

The rotated topic will be published to `/cam0/image_raw/rotated`, which you can visualize in RViz.

Optional arguments:

- Add `compressed` as second argument if the input is a `CompressedImage` topic
- Add `gui` as third argument to show a preview window

<a name="image-stitching"></a>
## Image Stitching

We are providing the two raw fisheye images only for the challenge because stitching them into a 360° equirectangular image is not mathematically or geometrically exact and requires certain assumptions.

This is because a dual-fisheye camera consists of two lenses separated by a small but non-zero baseline, meaning they do not share the same optical center. However, generating a single equirectangular image implicitly assumes a single effective viewpoint (i.e., a central projection model). Without per-pixel depth information, it is impossible to correctly reconcile the parallax between the two lenses. Projecting both fisheye images onto a common unit sphere therefore introduces geometric inconsistencies, since the rays originate from different optical centers.

For people who are still interested in working with 360° equirectangular images, we provide a reference image stitching script. Run the command below from the repository checkout so the `config/` paths resolve, and replace the bag path with your downloaded recording. The output is written to `rosbag_pano`. This script assumes a large radius for the virtual projection sphere (it is not insta360 official stitching algorithm):

```bash
cd ~/ros2_ws/src/hilti-trimble-slam-challenge-2026
ros2 run challenge_tools_ros image_stitching.py \
    --bag   /path/to/data/floor_1/2025-05-05/run_1/rosbag \
    --yaml  config/hilti_openvins/kalibr_imucam_chain.yaml \
    --mask0 config/hilti_openvins/mask_cam0.png \
    --mask1 config/hilti_openvins/mask_cam1.png \
    --out   rosbag_pano \
    --use-torch
```

The `--use-torch` option requires PyTorch. Omit it to use the OpenCV implementation.

---

Next: [OpenVINS with fisheye images](openvins.md) · [Stella-VSLAM with stitched images](stella-vslam.md).
