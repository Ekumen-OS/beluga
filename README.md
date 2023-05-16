# Beluga

[![CI pipeline](https://github.com/Ekumen-OS/beluga/actions/workflows/ci_pipeline.yml/badge.svg?branch=main)](https://github.com/Ekumen-OS/beluga/actions/workflows/ci_pipeline.yml?query=branch:main)
[![License Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

## Overview

Beluga is an extensible C++17 library with a ground-up implementation of the Monte Carlo Localization (MCL) family of estimation algorithms featuring:

- A modular design based on orthogonal components that can be merged together using the mixin pattern.
- Good test coverage to prevent regressions and facilitate code improvements.
- Automated benchmarks that can be used to validate different configurations.

https://github.com/Ekumen-OS/beluga/assets/33042669/4481cc43-f0cd-40df-a5b4-7e0a8ccb87c6

## Packages

This repository contains the following packages:

| Package                                      | Description                                                                                                             |
|----------------------------------------------| ------------------------------------------------------------------------------------------------------------------------|
| [`beluga`](beluga)                           | A ROS-agnostic extensible library to implement algorithms based on particle filters.                                    |
| [`beluga_amcl`](beluga_amcl)                 | A ROS 2 wrapper, providing an executable node and a ROS 2 component.<br> It provides interface parity with `nav2_amcl`. |
| [`beluga_example`](beluga_example)           | Example launch files, showing how to run Beluga-based nodes.                                                            |
| [`beluga_benchmark`](beluga_benchmark)       | Scripts to benchmark, profile and also compare Beluga with other MCL implementations.                                   |
| [`beluga_system_tests`](beluga_system_tests) | System integration tests for Beluga.                                                                                    |

## First Steps

- Get hands-on experience with the [getting started](GETTING_STARTED.md) tutorial.
- Read the [contributing guidelines](CONTRIBUTING.md).
