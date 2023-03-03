# Beluga

![CI badge](https://github.com/ekumenlabs/beluga/actions/workflows/ci_pipeline.yml/badge.svg?event=push)

This repository provides three packages:

- `beluga`: A ROS agnostic extensible implementation of a particle filter.
- `beluga_amcl`: A ROS 2 wrapper, providing an executable and a ROS 2 component.
    It provides almost feature parity with `nav2_amcl`.
- `beluga_example`: Example launch files, showing how to run beluga.
- `beluga_benchmark`: Scripts to benchmark, profile and also compare beluga with other AMCL implementations.

## FAQ

- How can I try an example?

  Check the ["run an example"](CONTRIBUTING.md#running_an_example) section in the contributing guide.

- How can I contribute to the project?

  Check the [contributing guidelines](CONTRIBUTING.md)

- Is there API documentation?

  Yes, but it's not currently being hosted online.
  Use [generate_docs.sh](beluga/docs/generate_docs.sh) to generate docs for the beluga package.

- Are the node parameters documented?

  You can find all the supported parameter documented in the example parameters [yaml file](beluga_example/config/params.yaml).


- How to benchmark beluga?

  You can find micro-benchmarks in the test folder of [beluga](beluga/test/benchmark/).
  For macro-benchmarking and comparing with other AMCL implementations, check [beluga_benchmark docs](beluga_benchmark/docs/BENCHMARKING.md).

- How to cpu profile beluga?

  Check [beluga_benchmark docs](beluga_benchmark/docs/PROFILING.md).
