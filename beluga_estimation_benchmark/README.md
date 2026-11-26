# Beluga Estimation odometry benchmark

This [Lambkin](https://github.com/Ekumen-OS/lambkin) benchmark compares four odometry estimates from the same ROS 2 bag. The three filters fuse wheel odometry twist and IMU yaw rate.

- `raw_odom` reads the recorded wheel odometry on `/odom` as a baseline.
- `rl_ekf` runs the `robot_localization` EKF.
- `rl_ukf` runs the `robot_localization` UKF.
- `beluga_estimation_ekf` runs the Beluga Estimation EKF and reads its estimate from the `beluga_odom` → `beluga_base_footprint` transform.

## Build and run on ROS 2 Jazzy

From the Beluga repository root, build the development image and launch the benchmark container:

```bash
ROSDISTRO=jazzy docker compose -f docker/docker-compose.yml build dev
USERID=$(id -u) GROUPID=$(id -g) ROSDISTRO=jazzy \
  docker compose -f beluga_estimation_benchmark/docker/docker-compose.yml up --build -d benchmark
docker compose -f beluga_estimation_benchmark/docker/docker-compose.yml exec benchmark bash
```

Inside the container:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to beluga_estimation_odometry_benchmark
source install/setup.bash
cd src/beluga/beluga_estimation_benchmark/odometry
python3 benchmark.py
```

The benchmark uses the bag at `/data/datasets/input` and the TUM ground truth at `/data/ground_truth/groundtruth.tum`. For other data, mount the bag and ground truth in the container and pass `--dataset` and `--ground-truth`. The benchmark expects the bag to publish `/clock`.

## Outputs

Files are written under `odometry/results/`.

- `var_N/iter_M/` contains the resampled trajectory `estimate.tum` and the evo result archives `output.rpe.zip` and `output.rpe_rot.zip` for that run.
- `reference.tum` contains the shared reference trajectory, and `plots.png` shows the trajectories and relative pose errors.
- Each run reports RMSE, mean, standard deviation, median, and maximum for both metrics on stdout.

## Evaluation

Every estimate is interpolated at the same ground-truth timestamps after dropping the first `--warmup` seconds (1 s by default). The benchmark uses [evo](https://github.com/MichaelGrupp/evo) to measure relative pose error (RPE).

| Metric | evo command options | Measures |
| --- | --- | --- |
| `rpe_trans` | `evo_rpe tum --delta 1 --delta_unit m` | Position error over 1 m, in meters |
| `rpe_rot` | `evo_rpe tum --pose_relation angle_deg --delta 10 --delta_unit m --all_pairs` | Heading error over 10 m, in degrees |

## Dataset attribution

The `ekumenlabs/lambkin-beluga-datasets:jazzy` image packages bag streams and ground-truth trajectories derived from the [Magazino Cartographer Dataset](https://github.com/magazino/cartographer_magazino), copyright 2018 Magazino GmbH, licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0). See the [dataset notice](docker/NOTICE).
