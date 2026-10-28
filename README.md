<div align="center">

<h1>Hilti × Trimble SLAM Challenge 2026</h1>

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

[<img src="https://img.shields.io/badge/Home%20Page--red?style=social&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI%2BPHBhdGggZmlsbD0iIzQyODVGNCIgZD0iTTEyIDMgMiAxMmgzdjloNXYtNmg0djZoNXYtOWgzTDEyIDN6Ii8%2BPC9zdmc%2B" alt="Home Page">](https://hilti-challenge.com/dataset-2026)
[<img src="https://img.shields.io/badge/arXiv--b31b1b?style=social&logo=arxiv" alt="arXiv">](https://arxiv.org/abs/2607.06464)
[<img src="https://img.shields.io/badge/Hugging%20Face--yellow?style=social&logo=huggingface&logoColor=FFD21E" alt="Hugging Face Dataset">](https://huggingface.co/datasets/Hilti-Research/hilti-trimble-slam-challenge-2026/tree/main)
[<img src="https://img.shields.io/badge/YouTube--red?style=social&logo=youtube" alt="YouTube">](https://www.youtube.com/playlist?list=PLIbIX8ez6UWS0N2A-lTmwYnmhMPQs8IYU)

<p>
  <a href="https://hilti-trimble-challenge.com/">
    <img src="media/HTSC2026.png?raw=true" alt="HTSC2026" width="80%">
  </a>
</p>

</div>

The accompanying paper has been accepted for publication in **IEEE Robotics and Automation Letters**. If you find this dataset useful, please consider giving it a ⭐ and citing it in your work.

```bibtex
@ARTICLE{slamchallenge2026,
  author={Centanni, Samuele and Zhang, Yuhao and Tao, Yifu and Kindle, Julien and Neuhaus, Frank and Ko\ss, Tilman and Patel, Aryaman and Helmberger, Michael and Szyma\'nska, Emilia and Gr\"aber, Torben and Fallon, Maurice},
  title={Hilti-Trimble-Oxford Dataset: 360 Visual-Inertial Benchmark With Floor Plan Priors for SLAM and Localization},
  journal={IEEE Robotics and Automation Letters}, 
  year={2026},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/LRA.2026.3747660}
}

```

<a name="introduction-360-visual-inertial-slam-challenge-with-additional-building-floorplans"></a>
## Introduction

Accurate and reliable SLAM and localization is crucial in robotics, automation, and digital construction workflows. Yet real-world construction sites remain among the most challenging environments for modern Visual-Inertial SLAM: changing lighting conditions, repetitive texture and ongoing building works push state-of-the-art algorithms to their limits.

To advance the field and to establish a common benchmark for these demanding scenarios, [Hilti](http://hilti.com/), [Trimble](https://www.trimble.com/), and the [Dynamic Robot Systems Group](https://dynamic.robots.ox.ac.uk/) of University of Oxford have joined forces to launch the SLAM Challenge 2026.
Our intention is to provide an open and realistic dataset, captured directly on active construction sites, to evaluate and compare SLAM systems in real world conditions.

The winners of the challenge were announced at [ICRA](https://2026.ieee-icra.org/) on **5th June 2026** in Vienna, Austria, as part of the [OCRAIM workshop](https://ariamhub.com/ocraim/).

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

<a name="challenge-homepage-evaluation-server"></a>
The [official challenge homepage](https://hilti-trimble-challenge.com) provides evaluation server and prize information. See the [evaluation guide](docs/challenge-evaluation.md) for tasks, submission requirements, and scoring.

## Start Here

1. **Get a first run working:** [download a sequence](docs/dataset.md#download-a-recording), then [install the ROS tools](docs/challenge-tools.md#install-challenge-tools).
2. **Run a baseline:** use [OpenVINS](docs/openvins.md) with the original fisheye images, or [Stella-VSLAM](docs/stella-vslam.md) with stitched panoramas.
3. **Evaluate your trajectory:** check the [ground truth conventions](docs/ground-truth.md) and [submission requirements](docs/challenge-evaluation.md#what-to-deliver).

## Documentation

| I want to… | Read |
| --- | --- |
| Download data, compare sequence conditions, and preview trajectories | [Dataset and Sequence Guide](docs/dataset.md) |
| Use floorplans for localization | [Floorplans](docs/floorplans.md) |
| Understand ROS topics, timestamps, and calibration | [Sensors and calibration](docs/sensors.md) |
| Use reference trajectories and initial poses | [Ground truth](docs/ground-truth.md) |
| Install the tools and process images | [ROS image tools](docs/challenge-tools.md) |
| Run visual-inertial odometry | [OpenVINS example](docs/openvins.md) |
| Run SLAM on panoramic images | [Stella-VSLAM example](docs/stella-vslam.md) |
| Prepare a submission and understand scores | [Challenge and evaluation](docs/challenge-evaluation.md) |

For questions or problems, open an [issue](https://github.com/Hilti-Research/hilti-trimble-slam-challenge-2026/issues).

## News

- **[2026-09-21]** ⏱️ Added an explanation of [sensor synchronization](docs/ground-truth.md#sensors-synchronization) in the ground truth collection device.
- **[2026-08-06]** 🤗 Released the dataset on [Hugging Face](https://huggingface.co/datasets/Hilti-Research/hilti-trimble-slam-challenge-2026/tree/main).
- **[2026-06-29]** 🔎 Added the [detailed sequence categorization](docs/dataset.md#preview-and-choose-a-sequence).

<details>
<summary>Earlier updates</summary>

- **[2026-06-16]** ⚓ Released the [ground truth](groundtruth/) for all the sequences.
- **[2026-06-12]** 🏆 Published the Challenge's [leaderboard](https://hilti-challenge.com/leaderboard-2026).
- **[2026-03-05]** ℹ️ Added rolling shutter [specifications](config/hilti_openvins/rs_config.yaml).
- **[2026-03-03]** 📐 Added floor plans in `dxf` format.
  > **Note:** These floor plans are not officially registered.
- **[2026-02-27]** 📷 Updated camera parameters and intrinsic calibration.
- **[2026-02-16]** 🖼️ Introduced [image stitching script](challenge_tools_ros/bag_helper/image_stitching.py) for dataset post-processing.
- **[2026-02-16]** 🎨 Added custom image encoding options in `image_conversion_node`.
- **[2026-02-03]** 🗺️ Fixed `floor_UG1_2025-05-19_run_1` static transformation.

</details>
