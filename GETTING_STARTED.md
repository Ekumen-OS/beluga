# Getting started with Beluga

## Crash course

1. **Clone the repository**. You will need `git`.

   ```bash
   git clone --recursive git@github.com:Ekumen-OS/beluga.git
   ```

1. **Build and run the development docker container**. You will need [`docker-compose v2.10+`](https://github.com/docker/compose/tree/v2).

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
   Supported distributions include `noetic`, `humble`, and `rolling`.

1. **Build and test the project** (inside development container).

    ```bash
    cd /ws
    colcon build --symlink-install
    colcon test
    ```

    You may also use `catkin_make_isolated` and `catkin-tools` in ROS 1 distributions.

1. **Run an example application using a ROS bag** (inside development container).

    For ROS 2 distributions, run:
    ```bash
    cd /ws
    source install/setup.bash
    ros2 launch beluga_example perfect_odometry.launch.xml
    ```

    For ROS 1 distributions, run:
    ```bash
    cd /ws
    source devel*/setup.bash
    roslaunch beluga_example perfect_odometry.launch
    ```

1. **Run an example application using a simulation and teleop controls** (inside development container).

    For ROS 2 distributions, in two separate terminals run:
    ```bash
    cd /ws
    source install/setup.bash
    ros2 launch beluga_example simulation.launch.xml
    ```
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

    For ROS 1 distributions, in two separate terminals run:
    ```bash
    cd /ws
    source devel*/setup.bash
    roslaunch beluga_example simulation.launch
    ```
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard
    ```

1. **Lint your code**. You will need [`pre-commit`](https://pre-commit.com/) and ROS dependencies (already installed in the development container).

    ```bash
    cd /ws/src/beluga
    pre-commit run --all-files
    ```

## Next steps

If you want to contribute to this project, please read the [contribuing guidelines](CONTRIBUTING.md).

For more advanced tools useful for contributing, check out the [tools](./tools/) directory.
