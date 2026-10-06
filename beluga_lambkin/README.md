# Beluga Lambkin Benchmark

Benchmarks [`beluga_amcl`](../beluga_amcl) with [Lambkin](https://github.com/Ekumen-OS/lambkin).

## Contents

- `beluga_benchmark.py`
- `requirements.txt` (Lambkin pin)
- `beluga_inference_ros2/` (launch + params used by the benchmark)

## Environment

Lambkin is installed in the jazzy, kilted and lyrical dev images.

Start the container with:
```bash
ROSDISTRO=jazzy docker/run.sh --build     # or kilted / lyrical
```

## Data

Needs an MCAP dataset and a map YAML. Reference data: https://huggingface.co/datasets/ekumenlabs/lambkin-beluga-datasets

Mount them with a local `docker/docker-compose.override.yml` (not committed), which `docker/run.sh` picks up automatically:
```yaml
services:
  dev:
    volumes:
      - /path/to/datasets:/data/datasets
      - /path/to/maps:/data/maps
```

## Run

Inside the container, from `/ws`:
```bash
colcon build --packages-up-to beluga_inference_ros2
source install/setup.bash
python3 src/beluga/beluga_lambkin/beluga_benchmark.py --dataset /data/datasets/input --map /data/maps/map.yaml
```

## Outputs

- `output.ape.zip`
- `plots.png`
- RMSE/mean/max stats in the log

## Updating Lambkin

Change the commit SHA in `requirements.txt`, then rebuild the image with `--build`.
