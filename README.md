# Beluga

![CI badge](https://github.com/ekumenlabs/beluga/actions/workflows/ci_pipeline.yml/badge.svg?event=push)

## Overview

Beluga is a library providing an extensible particle filter implementation, including sensor and motion models.
It also provides a `drop-in` replacement for the `amcl` node needed by [navigation2](https://github.com/ros-planning/navigation2).

https://user-images.githubusercontent.com/26796393/222827672-b8ab8421-5933-42f3-8144-be8d417e8aeb.mp4

## Packages

| Package            | Description |
|--------------------| ------------|
| `beluga`           | A ROS agnostic extensible library to implement algorithms based on particle filters. |
| `beluga_amcl`      | A ROS 2 wrapper, providing an executable node and a ROS 2 component.<br> It provides almost feature parity with `nav2_amcl`. |
| `beluga_example`   | Example launch files, showing how to run beluga. |
| `beluga_benchmark` | Scripts to benchmark, profile and also compare beluga with other AMCL implementations. |

## FAQ

- How can I try an example?

  Check the [contributing guidelines](CONTRIBUTING.md), specifically the [_run an example application_](CONTRIBUTING.md#running_an_example) section.

- How can I contribute to the project?

  Check the [contributing guidelines](CONTRIBUTING.md).

- Is there API documentation?

  Yes, but it's not currently being hosted online.
  You can use [a script](beluga/docs/generate_docs.sh) to generate docs for the beluga package locally.

- Are the node parameters documented?

  You can find all the supported parameter documented in the example parameters [yaml file](beluga_example/config/params.yaml).

- How to benchmark Beluga?

  You can find micro-benchmarks in the [test folder](beluga/test/benchmark/) of Beluga.<br/>
  For macro-benchmarking and comparing with other AMCL implementations, check the [benchmarking documentation](beluga_benchmark/docs/BENCHMARKING.md).

- How to cpu profile Beluga?

  Check the [profiling documentation](beluga_benchmark/docs/PROFILING.md).

## License

Beluga is available under the `Apache 2.0` license.
See [LICENSE](LICENSE).
