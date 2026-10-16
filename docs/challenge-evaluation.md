[Home](../README.md) · [Documentation guide](../README.md#documentation)

<a name="challenge-evaluation"></a>
# Challenge & Evaluation

We challenge participants with two different tasks. You are free to participate in one or both of these tasks.

The two tasks are the following:

1. **SLAM**: we ask you to estimate the trajectory of the Insta360 camera - in **any reference frame**,
2. **Localization**: we ask you to estimate the camera trajectory in the **floorplan reference frame** (map coordinate frame). We consider the bottom-left pixel of the floorplan as the $(0,0)$ coordinate.

<a name="what-to-deliver"></a>
## What to Deliver

Use the [ground truth pose conventions](ground-truth.md) and [floorplan frame](floorplans.md#coordinate-frame-and-initial-pose) when preparing trajectories.

For each run, we require your trajectory estimates to achieve **at least $99$**% coverage of the ground truth trajectory. **Otherwise**, the score for that run will be automatically set to 0.

The coverage is computed as $\dfrac{\text{|matched predicted poses|}}{\text{|ground truth poses|}} * 100$.

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

<a name="metric"></a>
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

<a name="notes"></a>
### Notes

1) The values $a$ and $c$ were obtained by fitting an exponential function such that:

- when the error is **0 m**, the score is **100**
- when the error is **10 m**, the score is **1**

2) Our LiDAR ground truth does not start at exactly the starting point of the camera recording, as a result we **will not evaluate the trajectory of the first 5 seconds of each run**.

3) When testing your solution with our evaluation server you will receive a report showing the error plots for your entry. We decided to hide details for six of the runs. For these six runs we will show you only the score **but we will not show the error plots for these six runs**.

4) In the **Localization task** we will not evaluate the _z-coordinate_.

5) **Important**: we will not adjust the trajectory scale (when using Evo), therefore you have to ensure that your trajectories are properly scaled.

<a name="final-score"></a>
## Evaluated Runs and Final Score

We provide 30 recordings to test, but we will evaluate only those that **were not** part of the Early Release made in November. This is because we provided the Ground Truth for these 5 Early Release runs:

- `floor_1_2025-05-05_run_1`,
- `floor_2_2025-05-05_run_1`,
- `floor_2_2025-10-28_run_1`,
- `floor_2_2025-10-28_run_2`,
- `floor_UG1_2025-10-16_run_1`.

As a result, for the **SLAM** task we will evaluate 25 out of 30 runs, and the maximum achievable total score will be **2500 points**.

For the **Localization** task we also will exclude `floor_UG2_2025-12-02_run_1` as we do not provide a floorplan for this recording. As a result, the maximum achievable total score for localization task will be **2400 points** (24 runs out of 30).

<a name="evaluation-server"></a>
## Evaluation Server

Submit your entry using the [evaluation server](https://submit.hilti-challenge.com/). The [challenge homepage](https://hilti-trimble-challenge.com) provides prize information, and the [leaderboard](https://hilti-challenge.com/leaderboard-2026) lists results.

---

Reference: [sequence guide](dataset.md#preview-and-choose-a-sequence) · [ground truth](ground-truth.md) · [floorplans](floorplans.md).
