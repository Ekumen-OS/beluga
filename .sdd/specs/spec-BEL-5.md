# Technical Specification: BEL-5 (Integrate Lambkin benchmark package into Beluga 2)

## Overview
This specification details the final architecture for integrating the Lambkin benchmark into the Beluga 2 repository. Based on the latest constraints, we are moving away from modifying the core development images. Instead, we will use a separate benchmark Docker image and `docker-compose` setup to isolate the Lambkin and `evo` dependencies, and we will use Docker image mounts to provide the reference dataset.

## 1. Revert Dev Dockerfile Changes
- The changes introduced in commit `ad1622cab` that added `ENV PIP_BREAK_SYSTEM_PACKAGES=1` and the `pip install` commands to the development Dockerfiles must be reverted.
- The `docker/images/{jazzy,kilted,lyrical,rolling}/Dockerfile` files must remain entirely unchanged from their state on `main`.

## 2. Separate Benchmark Image (`beluga_lambkin/Dockerfile`)
Create a new Dockerfile at `beluga_lambkin/Dockerfile` that builds on top of the development images.
- **Base image**: `FROM ekumenlabs/beluga-${ROSDISTRO}-dev`
- **Dependencies installation**:
  - Install `uv`.
  - Install Lambkin from `requirements.txt` into a virtual environment using `uv venv --system-site-packages` (so that apt-installed packages like numpy/matplotlib are available). Do **not** use `PIP_BREAK_SYSTEM_PACKAGES`.
  - Add the `venv/bin` directory to the front of the `$PATH`.
  - Install `evo` using `uv tool install evo` to keep it isolated.
- **Note on `evo`**: It is GPL-3.0 licensed. The benchmark must only run `evo_ape` as an external command and MUST NEVER `import evo` in Python code.

## 3. Benchmark Docker Compose (`beluga_lambkin/docker-compose.yml`)
Create a new compose file at `beluga_lambkin/docker-compose.yml` defining a `benchmark` service.
- **Service Configuration**:
  - Build the image defined in `beluga_lambkin/Dockerfile`.
  - Mirror the `dev` service from the main compose file (e.g., user configuration, repository mounted at `/ws/src/beluga`).
  - Set `privileged: true` (required for Lambkin to have cgroup write access).
- **Dataset Volume Mounts**:
  - Mount the reference dataset directly from the public image `ekumenlabs/lambkin-beluga-datasets:jazzy` using Docker's image mount feature.
  - Provide a commented-out option for mounting host data to a separate path like `/host_data` to prevent shadowing the reference data.
  ```yaml
      volumes:
        # Reference data, mounted straight from the image:
        - type: image
          source: ekumenlabs/lambkin-beluga-datasets:jazzy
          target: /data
          image:
            subpath: data
        # Optionally, your own data from the host, next to the reference data:
        # - /path/to/data:/host_data
  ```
- Do **not** modify the main `docker/docker-compose.yml`.

## 4. `beluga_inference_ros2` Updates
- **Dependencies**: Add `<exec_depend>python3-matplotlib</exec_depend>` to `beluga_lambkin/beluga_inference_ros2/package.xml` so it is installed via `rosdep`/`apt`.
- **Maintainer**: Add a new maintainer entry to `beluga_lambkin/beluga_inference_ros2/package.xml` after the existing maintainers:
  `<maintainer email="daffer.queque@ekumenlabs.com">Daffer Queque</maintainer>`

## 5. Documentation Updates (`beluga_lambkin/README.md`)
- **Execution Instructions**: Instruct the user to build the main dev image first, and then start the benchmark container using the new `beluga_lambkin/docker-compose.yml`.
- **Data Section**: Remove the Hugging Face link and the `docker/docker-compose.override.yml` example. Explain that the reference data is mounted directly by the benchmark compose file at `/data`, and explain how to pass custom host data using `--dataset /host_data/... --map /host_data/...`. Keep the style short and concise.

## 6. CI Configuration
- **DO NOT** add the new benchmark image to the `build-image` matrix in `docker.yml`. The dev images must remain the primary CI target.
- If a CI workflow is desired for the benchmark image, it must be a separate workflow triggered only on `beluga_lambkin/**` changes and scheduled weekly, and it must not be a required check for PRs.
