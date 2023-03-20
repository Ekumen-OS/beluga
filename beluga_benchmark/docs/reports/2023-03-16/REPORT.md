# Performance comparison with nav2_amcl

## Environment details

- CPU: Intel® Core™ i7-8565U CPU @ 1.80GHz × 8
- RAM: 2 x 16384 MB x 2667 MT/s
- Host OS: Ubuntu 18.04.6 LTS
- Commit hash: 47ed3643ec132e9357a0e16d7d0e97be902e2450

## Results

The benchmarks was run using 1000, 2000, 5000, 10000 and 20000 particles.
Beluga was run both using a sequential and a parallel executor.
`nav2_amcl` only provides sequential execution.

The results were the following:

![results-1.png](results-1.png)

It can be seen that:

- Beluga needs less RAM memory, it's peak usage for 20000 particles was 35MB, whereas it was 44MB for `nav2_amcl`.
- Beluga uses less CPU when using the sequential executor.
  For 20000 particles, Beluga was using 85% of the CPU, whereas `nav2_amcl` was using 103%.
- Beluga estimated trajectory absolute pose error is on par or slightly better than `nav2_amcl`.
  It can also be seen that `beluga` tends to improve on those metrics when using a higher number of particles, while `nav2_amcl` starts getting worst results.
  Probably because `nav2_amcl` starts dropping messages without being able to process the input fast enough.

Besides that, beluga can handle a lot more particles using the parallel executor:

![results-2.png](results-2.png)

For more than 20000 particles, both `nav2_amcl` and Beluga using a sequential executor start using close to 100% of one cpu and can't handle more load without starting to drop messages.
Beluga can use a parallel executor, making use of multiple cores.
This may not only be interesting when there is a larger amount of particles, but may also allow the use of more complex measurement models in the future.

## How to reproduce

Running the benchmarks:

```bash
cd beluga_seq
ros2 run beluga_benchmark parameterized_run --initial-pose-y 2.0 1000 2000 5000 10000 20000 100000 --params-file ../params.yaml
cd -
cd beluga_par
ros2 run beluga_benchmark parameterized_run --initial-pose-y 2.0 1000 2000 5000 10000 20000 100000 --params-file ../params_par.yaml
cd -
cd nav2_amcl
ros2 run beluga_benchmark parameterized_run --initial-pose-y 2.0 1000 2000 5000 10000 20000 100000 --params-file ../params.yaml --package nav2_amcl --executable amcl
cd -
```

Visualizing the results:

```bash
ros2 run beluga_benchmark compare_results beluga_par/ beluga_seq/ nav2_amcl --max-particles 20000
ros2 run beluga_benchmark compare_results beluga_par/ beluga_seq/ nav2_amcl
```
