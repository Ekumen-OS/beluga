# Performance comparison between beluga_amcl and nav2_amcl

## Environment details

- CPU: **Intel(R) Core(TM) i9-9900 CPU @ 3.10GHz x 16 cores**
- CPU Caches: L1 Data 32 KiB (x8), L1 Instruction 32 KiB (x8), L2 Unified 256 KiB (x8), L3 Unified 16384 KiB (x1)
- RAM: 16384 MB
- Host OS: Ubuntu 22.04.6 LTS
- ROS 2 version: **Humble Hawksbill**
- Beluga revision commit hash: c0e2de14c6a8eac9c5353cc56339d9975bc0ea19

## Experimental setup

The following configuration was used during the experiments:

- The benchmarks were run using 250, 300, 400, 500, 750, 1000, 2000, 5000, 10000, 20000, 50000, 100000 and 200000 particles.
- `beluga_amcl` was run both using multithreaded (`par`) and non-multithreaded (`seq`) configurations. `nav2_amcl` only provides non-multithreaded execution.
- Both the `beam sensor` and the `likelihood field` sensor model were tested.
- The bagfile containing the synthetic dataset was replayed at 1x speed (real time).
- Due to the issue described in [this issue](https://github.com/Ekumen-OS/beluga/issues/253), `nav2_amcl` was run built from source instead of using the binary provided by the ROS 2 Humble packages. The version corresconpnds to the `1.1.9` release of the `navigation2` repository.

More specific configuration details can be found in the `yaml` files in the [baseline configuration folder](../baseline_configurations/):

- `nav2_amcl` (likelihood field) uses [likelihood_params.yaml](likelihood_params.yaml)
- `beluga_amcl` (likelihood field, non-multithreaded) uses [likelihood_params.yaml](likelihood_params.yaml)
- `beluga_amcl` (likelihood field, multithreaded) uses [likelihood_params_par.yaml](likelihood_params_par.yaml)
- `nav2_amcl` (beam) uses [beam_params.yaml](beam_params.yaml)
- `beluga_amcl` (beam, non-multithreaded) uses [beam_params.yaml](beam_params.yaml)
- `beluga_amcl` (beam, multithreaded) uses [beam_params_par.yaml](beam_params_par.yaml)

Except for the multithreading and sensor model parameters, the configuration on all of the files is identical.

## Recorded metrics

The following metrics were recorded during each run:

- RSS (Resident Set Size), amount of memory alloated to the process. Measured in megabytes.
- CPU usage. Measured in percentage of the total CPU usage.
- APE (Absolute Pose Error) statistics: `mean`, `median`, `max` and `rmse`. In meters.

## Results

#### Beluga vs. Nav2 AMCL using Likelihood Field sensor model

In the following plot the results of the benchmark are shown for all three of the tested configurations. The vertical scale is logarithmic to better show the differences between the configurations throughout the whole range of particle counts.

![Beluga Seq vs Beluga Par vs. Nav2 AMCL with Likelihood Field Sensor Model](likelihood_beluga_vs_beluga_vs_amcl.png)

Comments on the results:

- The memory usage of `beluga_amcl` (both configurations) is significantly lower than that of `nav2_amcl`.
- Both `beluga_amcl` configurations perform at a lower CPU usage level than `nav2_amcl` when using the `likelihood` sensor model.
- Above $50k$ particles `nav2_amcl` begins to saturate the CPU and its APE metrics begin to deteriorate significantly, while `beluga_amcl`'s remain stable. For particle conts below $50k$ the APE metrics of both `beluga_amcl` and `nav2_amcl` are similar.

#### Beluga vs. Nav2 AMCL using Beam sensor model

In the following plot the results of the benchmark are shown for all three of the tested configurations when using the Beam Sensor model. The vertical scale is logarithmic to better show the differences between the configurations throughout the whole range of particle counts.

![Beluga Seq vs Beluga Par vs. Nav2 AMCL with Beam Sensor Model](beam_beluga_vs_beluga_vs_amcl.png)

Comments on the results:

- `beluga_amcl` in both multithreaded and non-multithreaded configurations uses significantly less memory than `nav2_amcl`.
- Both `beluga_amcl` configurations perform at a lower CPU usage level than `nav2_amcl` when using the `beam` sensor model.
- The APE performance of both multithreaded and non-multithreaded `beluga_amcl` is similar to that of `nav2_amcl` throughout the whole range of particle counts.

## Conclusions

- `beluga_amcl`'s memory usage is significantly lower than that of `nav2_amcl` in all configurations.
- `beluga_amcl`'s CPU usage is consistently lower than that of `nav2_amcl` in all configurations, scaling better with particle count.
- `beluga_amcl`'s APE performance is similar to that of `nav2_amcl` for lower particle counts. For particles counts higher than $50k$, `nav2_amcl` single-threaded process saturates the CPU and its APE performance drops significantly, while `beluga_amcl` remains stable.

## How to reproduce

To replicate the benchmarks, after building and sourcing the workspace, run the following commands from the current directory:

```bash
mkdir beam_beluga_seq
cd beam_beluga_seq
ros2 run beluga_benchmark parameterized_run 250 300 400 500 750 1000 2000 5000 10000 20000 50000 100000 200000  --params-file ../../baseline_configurations/beam_params.yaml
cd -
mkdir beam_beluga_par
cd beam_beluga_par
ros2 run beluga_benchmark parameterized_run 250 300 400 500 750 1000 2000 5000 10000 20000 50000 100000 200000  --params-file ../../baseline_configurations/beam_params_par.yaml
cd -
mkdir beam_nav2_amcl
cd beam_nav2_amcl
ros2 run beluga_benchmark parameterized_run 250 300 400 500 750 1000 2000 5000 10000 20000 50000 100000 200000 --params-file ../../baseline_configurations/beam_params.yaml --package nav2_amcl --executable amcl
cd -
mkdir likelihood_beluga_seq
cd likelihood_beluga_seq
ros2 run beluga_benchmark parameterized_run 250 300 400 500 750 1000 2000 5000 10000 20000 50000 100000 200000  --params-file ../../baseline_configurations/likelihood_params.yaml
cd -
mkdir likelihood_beluga_par
cd likelihood_beluga_par
ros2 run beluga_benchmark parameterized_run 250 300 400 500 750 1000 2000 5000 10000 20000 50000 100000 200000  --params-file ../../baseline_configurations/likelihood_params_par.yaml
cd -
mkdir likelihood_nav2_amcl
cd likelihood_nav2_amcl
ros2 run beluga_benchmark parameterized_run 250 300 400 500 750 1000 2000 5000 10000 20000 50000 100000 200000 --params-file ../../baseline_configurations/likelihood_params.yaml --package nav2_amcl --executable amcl
cd -
```

Once the data has been acquired, it can be visualized using the following commands:

```bash
ros2 run beluga_benchmark compare_results \
    -s beam_beluga_seq -l beam_beluga_seq \
    -s beam_beluga_par -l beam_beluga_par \
    -s beam_nav2_amcl  -l beam_nav2_amcl --use-ylog

ros2 run beluga_benchmark compare_results \
    -s likelihood_beluga_seq -l likelihood_beluga_seq \
    -s likelihood_beluga_par -l likelihood_beluga_par \
    -s likelihood_nav2_amcl  -l likelihood_nav2_amcl --use-ylog

ros2 run beluga_benchmark compare_results \
    -s beam_beluga_seq -l beam_beluga_seq \
    -s likelihood_beluga_seq -l likelihood_beluga_seq \
    -s beam_beluga_par -l beam_beluga_par \
    -s likelihood_beluga_par -l likelihood_beluga_par \
    -s beam_nav2_amcl  -l beam_nav2_amcl \
    -s likelihood_nav2_amcl  -l likelihood_nav2_amcl --use-ylog
```
