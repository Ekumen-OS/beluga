# How to contribute to Beluga

Thank you for investing your time in contributing to this project!

## Getting started

### Issues

#### Create a new issue

If you spot a problem or have a feature request you'd like to discuss, [search if an issue already exists](https://docs.github.com/en/github/searching-for-information-on-github/searching-on-github/searching-issues-and-pull-requests#search-by-the-title-body-or-comments).
If a related issue doesn't exist, you can [open a new issue](https://github.com/ekumenlabs/beluga/issues/new/choose).

### Make changes

#### Prerequisites

- Make sure you have [`docker compose`](https://github.com/docker/compose/tree/v2) installed ([`v2.10+`](https://github.com/docker/compose/releases/tag/v2.10.0)).
   ```bash
   docker compose version
   ```
   If you don't, please follow [these instructions](https://docs.docker.com/compose/install/linux/).

#### Make changes locally

1. Clone the repository.
   ```bash
   git clone --recursive git@github.com:ekumenlabs/beluga.git
   ```

2. Build and run the development docker container.
   ```bash
   cd <REPOSITORY_PATH>
   docker/run.sh
   ```
   To build the image before starting the container, use:
   ```bash
   docker/run.sh --build
   ```

3. **[Optional]** Install pre-commit hooks. _This will probably cause you to not be able to create commits from your host machine since the hooks have dependencies that exist only in the development container._
   ```bash
   cd /ws/src
   pre-commit install
   ```
   Alternatively, you can run the hooks manually.
   ```bash
   cd /ws/src
   pre-commit run --all-files
   ```

4. Create a working branch and start with your changes. The suggested branch name convention is `<user_name>/<feature_name>`.

5. Build and test the project.
   ```bash
   cd /ws
   colcon build --symlink-install
   colcon test
   ```

6. Run an example application using a pre-recorded rosbag.
   ```bash
   cd /ws
   source install/setup.bash
   ros2 launch beluga_example example_rosbag_launch.py
   ```
   You also can use `teleop_twist_keyboard` to control a simulated robot. In two separate terminals run:
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

7. Push your changes and [create a PR](https://github.com/ekumenlabs/beluga/compare)!
