# Beluga Lambkin Benchmark

Benchmarks [`beluga_amcl`](../beluga_amcl) with [Lambkin](https://github.com/Ekumen-OS/lambkin).

## Contents

- `Dockerfile` (extends the dev image with Lambkin and evo)
- `docker-compose.yml` (benchmark container configuration)
- `beluga_benchmark.py`
- `requirements.txt` (Lambkin pin)
- `beluga_inference_ros2/` (launch + params used by the benchmark)

## Environment

Build the main dev image first:
```bash
ROSDISTRO=jazzy docker/run.sh --build
```

Then build and start the benchmark container:
```bash
docker compose -f beluga_lambkin/docker-compose.yml up --build -d benchmark
docker compose -f beluga_lambkin/docker-compose.yml exec benchmark /bin/bash
```

## Data

Reference data is mounted automatically by the benchmark compose file at `/data` (`/data/datasets/input` and `/data/maps/map.yaml`) directly from `ekumenlabs/lambkin-beluga-datasets:jazzy`.

To use your own data, mount your data directory to a separate host path (e.g. `/host_data`) in `beluga_lambkin/docker-compose.yml` and pass `--dataset /host_data/... --map /host_data/...`.

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

Change the commit SHA in `requirements.txt`, then rebuild the benchmark image.
