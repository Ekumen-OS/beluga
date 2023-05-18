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
   Supported distributions include `humble` and `rolling`.

1. **Build and test the project** (inside development container).

    ```bash
    cd /ws
    colcon build --symlink-install
    colcon test
    ```

1. **Run an example application using a ROS bag** (inside development container).

    ```bash
    cd /ws
    source install/setup.bash
    ros2 launch beluga_example example_rosbag_launch.py
    ```

1. **Run an example application using a simulation and teleop controls** (inside development container).

    In two separate terminals run:
    ```bash
    cd /ws
    source install/setup.bash
    ros2 launch beluga_example example_launch.py
    ```
    ```bash
    cd /ws
    source install/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

1. **Lint your code**. You will need [`pre-commit`](https://pre-commit.com/) and ROS dependencies (already installed in the development container).

    ```bash
    cd /ws/src/beluga
    pre-commit run --all-files
    ```

1. **Build documentation** (inside development container).

   ```bash
   cd /ws
   ./src/beluga/docs/generate_docs.sh
   ```

## Next steps

If you want to contribute to this project, please read the [contribuing guidelines](CONTRIBUTING.md).
