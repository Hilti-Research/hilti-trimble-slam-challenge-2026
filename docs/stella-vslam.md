[Home](../README.md) · [Documentation guide](../README.md#documentation)

<a name="stella-vslam-example"></a>
# Stella-VSLAM Example

Stella-VSLAM is another open-source vSLAM system. It directly processes 360° equirectangular images. To install it, follow the instructions [here](https://stella-cv.readthedocs.io/en/latest/ros2_package.html).

## Before You Start

Complete [tool installation](challenge-tools.md#install-challenge-tools), install Stella-VSLAM using the instructions above, and [prepare each terminal](challenge-tools.md#prepare-each-terminal), including the workspace containing your Stella-VSLAM installation.

<a name="run-stella-vslam"></a>
## Run Stella-VSLAM

First, stitch the fisheye images into 360° equirectangular images using either our [image stitching guide](challenge-tools.md#image-stitching) or your own stitching algorithm.

In **Terminal 1**, start Stella-VSLAM:

```bash
ros2 launch challenge_tools_ros run_stella_vslam.launch.py
```

In **Terminal 2**, play the ros bag:

```bash
ros2 bag play /path/to/stitched/ros2/bag/folder -p --rate 0.5
```

An example of Stella-VSLAM running on stitched images is shown below:
<div align="center">
  <img src="../media/floor2_stella_vslam_stitch.png" alt="floor1_stella_vslam_stitch" width="50%"/>
</div>

## Troubleshooting

Sometimes the Stella-VSLAM ROS wrapper may fail to properly subscribe to image messages. If this happens, run the `image_conversion_node` in a separate terminal instead of including it in the launch file. Then repeatedly terminate and relaunch the node until the image is displayed properly.

---

Related: [stitching assumptions](challenge-tools.md#image-stitching) · [submission format](challenge-evaluation.md#what-to-deliver).
