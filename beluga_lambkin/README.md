# Beluga Lambkin Benchmark

This directory integrates the [Lambkin](https://github.com/Ekumen-OS/lambkin) benchmarking suite into Beluga to evaluate `beluga_amcl` performance.

## Structure

- `beluga_benchmark.py`: Nominal benchmark script defining trial variants (`laser_model_type` and `max_particles`), data inputs, launch configuration, and metric evaluations.
- `requirements.txt`: Single source of truth pinning the external Lambkin SDK commit.
- `beluga_inference_ros2/`: Supporting ROS 2 package containing the launch file (`launch/beluga.launch.py`) and parameter defaults (`params/default.ros2.yaml`) executed by the benchmark.

## Supported Docker Environments

Lambkin is installed in the **Jazzy**, **Kilted**, and **Lyrical** development container images. Humble and Rolling images do not install Lambkin.

## Updating the Lambkin Version

To bump the pinned Lambkin commit, update the git revision SHA in `beluga_lambkin/requirements.txt`:
```text
lambkin @ git+https://github.com/Ekumen-OS/lambkin.git@<new_commit_sha>
```

## Datasets and Maps

The benchmark requires an MCAP dataset (with laser scan, odometry, and ground truth topics) and an occupancy grid map YAML. Reference datasets are available at [`ekumenlabs/lambkin-beluga-datasets`](https://huggingface.co/datasets/ekumenlabs/lambkin-beluga-datasets).

Mount your local dataset and map directory into the development container (defaults assume `/data`):
```bash
docker run -v /path/to/datasets:/data/datasets -v /path/to/maps:/data/maps ...
```

## Build and Run

Inside the development container:

1. Build the ROS 2 inference package:
   ```bash
   colcon build --packages-up-to beluga_inference_ros2
   source install/setup.bash
   ```

2. Execute the benchmark:
   ```bash
   python3 beluga_lambkin/beluga_benchmark.py --dataset /data/datasets/input --map /data/maps/map.yaml
   ```
   Or using the Lambkin CLI:
   ```bash
   lambkin run beluga_lambkin/beluga_benchmark.py --dataset /data/datasets/input --map /data/maps/map.yaml
   ```

## Benchmark Outputs

Execution produces the following artifacts in the current output directory:
- `output.ape.zip`: Saved evo Absolute Pose Error (APE) metrics and trajectory evaluation results.
- `plots.png`: Timeseries error plot across variants and iterations.
- Stats log: Summary containing RMSE, mean, and maximum APE metrics logged to stdout/logger.
