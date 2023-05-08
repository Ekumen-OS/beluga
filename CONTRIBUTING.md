# How to contribute to Beluga

Thank you for investing your time in contributing to this project!

## Contributions

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](./LICENSE):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

Contributors must sign-off each commit by adding a `Signed-off-by: ...`
line to commit messages to certify that they have the right to submit
the code they are contributing to the project according to the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/).

## Getting started

### Issues

#### Create a new issue

If you spot a problem or have a feature request you'd like to discuss, [search if an issue already exists](https://docs.github.com/en/github/searching-for-information-on-github/searching-on-github/searching-issues-and-pull-requests#search-by-the-title-body-or-comments).
If a related issue doesn't exist, you can [open a new issue](https://github.com/Ekumen-OS/beluga/issues/new/choose).

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
   git clone --recursive git@github.com:Ekumen-OS/beluga.git
   ```

1. Build and run the development docker container.
   ```bash
   cd <REPOSITORY_PATH>
   docker/run.sh
   ```
   To build the image before starting the container, use:
   ```bash
   docker/run.sh --build
   ```
   To target an specific ROS distribution, use:
   ```bash
   ROSDISTRO=humble docker/run.sh
   ```
   Supported distributions include `humble` and `rolling`.

1. **[Optional]** Install pre-commit hooks. _This will probably cause you to not be able to create commits from your host machine since the hooks have dependencies that exist only in the development container._
   ```bash
   cd /ws/src
   pre-commit install
   ```
   Alternatively, you can run the hooks manually.
   ```bash
   cd /ws/src
   pre-commit run --all-files
   ```

1. Create a working branch and start with your changes. The suggested branch name convention is `<user_name>/<feature_name>`.

1. Build and test the project.
   ```bash
   cd /ws
   colcon build --symlink-install
   colcon test
   ```

1. <a name="running_an_example"></a>Run an example application using a pre-recorded rosbag.

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

1. Push your changes and [create a PR](https://github.com/Ekumen-OS/beluga/compare)!

1. At the time a feature branch is squashed-and-merged into `main`, the commit message should adhere to the following good practices:
   - Limit the subject line to 50 characters.
   - Capitalize the subject line.
   - Do not end the subject line with a period.
   - Use the imperative mood in the subject line.
   - Wrap the body at 72 characters.
   - Use the body to explain _what_ and _why_ vs. _how_.
   - See https://cbea.ms/git-commit/ for more information and the reasoning behind this.
