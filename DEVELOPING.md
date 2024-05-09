# Developing Beluga

## Table of Contents

- [Environment](#environment)
- [Workflow](#workflow)
- [CI/CD](#ci-cd)

## Environment

Beluga developers use [Docker](https://www.docker.com) containers as development environment. These containers include both project dependencies and development tooling for each of the supported platforms. Source code [bind mounts](https://docs.docker.com/storage/bind-mounts) afford Beluga developers the freedom of applying their own personal programming workflows to Beluga (e.g. which editor to use), so long as they are compatible with the project guidelines.

To bring up a development environment:

1. **Clone the repository**. You will need `git`.

   ```bash
   git clone --recursive git@github.com:Ekumen-OS/beluga.git
   ```

2. **Build and run the development container**. You will need [`docker-compose v2.10+`](https://github.com/docker/compose/tree/v2).

   ```bash
   (cd beluga && docker/run.sh)
   ```

   To rebuild the image before starting the container, use:

   ```bash
   (cd beluga && docker/run.sh --build)
   ```

   To target an specific ROS distribution, use:

   ```bash
   (cd beluga && ROSDISTRO=humble docker/run.sh)
   ```

   Supported distributions include `noetic`, `humble`, `iron`, and `rolling`.

## Workflow

The Beluga project started as a ROS 2 project and therefore relies on a typical ROS 2 workflow. In particular, the Beluga project subscribes to [REP-2004](https://ros.org/reps/rep-2004.html) and aims for Quality Level 1. For development, this translates to mandatory API documentation and a 95% code coverage policy for unit and integration testing.

Within a development environment:

1. **Build and test the project**. You will usually use `colcon`.

    ```bash
    cd /ws
    colcon build --symlink-install
    colcon test
    ```

    You may also use `catkin_make_isolated` and `catkin-tools` for ROS 1 distributions.

2. **Update package dependencies**. Listed in `package.xml` files.

    ```bash
    cd /ws
    rosdep update
    rosdep install --from-path src
    ```

For more advanced tooling, check repository [tools](./tools).

## CI/CD

Every pull request and every push to the project repository `main` branch will be subject to a [continuous integration workflow](./.github/workflows/ci_pipeline.yml). This workflow will lint project sources, static analyze, build, and test project packages against the project platform support matrix, enforcing its code coverage policy, and build project documentation (which will be deployed to Github Pages when pushing to `main`).

Pull request acceptance is predicated on these checks passing.

## Next Steps

- Go over the [project documentation](https://ekumen-os.github.io/beluga) to better understand what, how, and why.
- If you want to contribute to this project, please read the [contribuing guidelines](CONTRIBUTING.md) first.
