# How to benchmark Beluga

1. [Setup a development environment](../../DEVELOPING.md#environment).

2. Build benchmarks and source them:

```sh
colcon build --packages-up-to beluga_benchmark
source install/setup.bash
```

3. Pull benchmark datasets:

```sh
mkdir -p src/beluga_benchmark/playground/datasets
# TODO(hidmic): bring datasets from where?
```

4. Run a benchmark e.g.:

```sh
pushd src/beluga_benchmark/playground
ros2 run beluga_benchmark nominal.robot
```

5. Inspect benchmark results:
```sh
xdg-open nominal/report/latex/report.pdf
```
