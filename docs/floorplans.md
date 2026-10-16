[Home](../README.md) · [Documentation guide](../README.md#documentation)

<a name="dataset-floorplans"></a>
# Floorplans and Localization

We provide the building floorplans as PNG images, which can be converted into ROS2 Occupancy Maps and visualized in RViz using the following [script](../challenge_tools_ros/runtime_helper/map_server.py).

The floorplans are stored in the repo [here](../floorplans). We provide two types of floorplan - **with and without windows**. Both formats show the main columns and walls of the building site.

<div align="center">
  <img src="../floorplans/masks_no_windows/floor_1.png" alt="floor_1 floorplan - no_window" width="50%"/>
</div>

The floorplans show the intended state of the building at the end of construction. During construction, the walls and columns are successively built which means that not all elements are present in the earlier recordings.

This image shows a ground truth trajectory aligned to the floor 1 floorplan.

<div align="center">
  <img src="../media/floorplan_vs_pcd.png" alt="floorplan vs pcd" width="50%"/>
</div>

<a name="dataset-properties"></a>
## Construction-Site Conditions

The dataset captures several realistic real-world challenges. These include:

- _Dynamic Initialization_: some runs start with **intentional** motion at the beginning of the run. This is a realistic challenge for this application.
- _Imprecise floorplans_: construction sites always contain deviations in their as-built state versus the as-planned state.

Algorithms which are robust to these deviations and to dynamic initialization will be more useful in real-world construction sites.

An example of deviation is shown in the following - the real wall is shorter than the floorplan:

<div align="center">
  <img src="../media/floorplan_pcd_devation.png" alt="floorplan vs pcd" width="50%"/>
</div>

In general, you can expect the floorplans to have an accuracy of 1cm relative to the real world.

See [sequence categorization](dataset.md#preview-and-choose-a-sequence) for the acquisition conditions of every run.

## Coordinate Frame and Initial Pose

For the localization task, the bottom-left pixel of the floorplan is the $(0,0)$ coordinate. Use the [initial camera poses](ground-truth.md#initial_localization) to initialize a trajectory in the map frame.

No floorplan is provided for `floor_UG2_2025-12-02_run_1`; see the [evaluation guide](challenge-evaluation.md#final-score) for excluded runs.

---

Related: [ground truth](ground-truth.md) · [OpenVINS floorplan visualization](openvins.md#run-openvins).
