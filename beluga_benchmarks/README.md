# Beluga Benchmarks

Benchmarks [`beluga_amcl`](../beluga_amcl) with [Lambkin](https://github.com/Ekumen-OS/lambkin).

Each benchmark lives in its own folder (currently `amcl/`).

## Contents

- `docker/` (`Dockerfile`, `docker-compose.yml`, `requirements.txt`)
- `amcl/`
  - `benchmark.py`
  - `beluga_amcl_benchmark/` (launch + params used by the benchmark)

## Environment

Tested on jazzy, kilted and lyrical. Humble is not supported (its numpy 1.21 is older than Lambkin's minimum, 1.24).

Build the main dev image first:
```bash
ROSDISTRO=jazzy docker/run.sh --build
```

Then build and start the benchmark container:
```bash
docker compose -f beluga_benchmarks/docker/docker-compose.yml up --build -d benchmark
docker compose -f beluga_benchmarks/docker/docker-compose.yml exec benchmark /bin/bash
```

## Data

Reference data is mounted automatically by the benchmark compose file at `/data` (`/data/datasets/input` and `/data/maps/map.yaml`) directly from `ekumenlabs/lambkin-beluga-datasets:jazzy`.

To use your own data, mount your data directory to a separate host path (e.g. `/host_data`) in `beluga_benchmarks/docker/docker-compose.yml` and pass `--dataset /host_data/... --map /host_data/...`.

## Run

Inside the container, from `/ws`:
```bash
colcon build --packages-up-to beluga_amcl_benchmark
source install/setup.bash
python3 src/beluga/beluga_benchmarks/amcl/benchmark.py --dataset /data/datasets/input --map /data/maps/map.yaml
```

## Outputs

- `output.ape.zip`
- `plots.png`
- RMSE/mean/max stats in the log

## Updating Lambkin

Change the commit SHA in `docker/requirements.txt`, then rebuild the benchmark image.
