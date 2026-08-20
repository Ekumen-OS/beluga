# Technical Specification: BEL-5 (Integrate Lambkin) - Revised

## Overview
This specification details the integration of the Lambkin benchmark package into the Beluga 2 repository. Based on the revised scope, we will **NOT** vendor the entire Lambkin repository. Instead, Lambkin will remain an external dependency. We will only store the Beluga-specific evaluation files within a new directory `beluga_lambkin/`. The deployment container will install Lambkin directly from its GitHub repository at a pinned commit.

## 1. Cleanup of PR #580 (Fresh Start)
The Implementer MUST clean up the previous implementation on this branch:
- Remove the entire vendored `beluga_lambkin/` directory that contains the full Lambkin SDK.
- This effectively resets the state regarding Lambkin, making way for the new shape.

## 2. Beluga-Evaluation Files
We will only keep the Beluga-specific benchmark files from Lambkin's `examples/beluga` folder.
- **Target Location**: A new top-level directory `beluga_lambkin/` in the Beluga repository.
- **Source**: `https://github.com/Ekumen-OS/lambkin` (`main` branch, specifically the `examples/beluga` directory).
- **Files to include**:
  - `beluga_benchmark.py` (the evaluation script).
  - `beluga_ros2/` (the entire ROS 2 package containing launch and params).
- **Files to DROP (Do NOT include)**:
  - `docker/` (the redundant nested Docker configuration).
  - The `README.md` should be either dropped or rewritten to remove the nested Docker instructions.
- **Structure**:
  ```
  beluga_lambkin/
  ├── beluga_benchmark.py
  └── beluga_ros2/
      ├── CMakeLists.txt
      ├── launch/
      ├── package.xml
      └── params/
  ```

## 3. Deployment Container
Provide a minimal Dockerfile that sets up the environment and installs the external Lambkin SDK.
- **File Path**: `beluga_lambkin/Dockerfile`
- **Base Image**: `ros:jazzy-ros-base`
- **Steps**:
  1. Set `DEBIAN_FRONTEND=noninteractive`.
  2. Install system dependencies (`git` and `python3-pip`).
  3. Clean up `apt` lists.
  4. Enable `PIP_BREAK_SYSTEM_PACKAGES=1` (required on Ubuntu 24.04).
  5. Install Lambkin directly from the pinned commit of its GitHub repository using pip.

**Dockerfile Template:**
```dockerfile
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    git \
    python3-pip \
  && rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Install Lambkin SDK externally from a pinned commit on main
RUN pip install "git+https://github.com/Ekumen-OS/lambkin.git@d9512d8b674fcea785833f58ee7a2c386b5f5b3a"
```

## Security and Integrity Considerations
- Pinning the external Lambkin installation to the specific commit `d9512d8b674fcea785833f58ee7a2c386b5f5b3a` ensures reproducibility and prevents upstream breakages.
- The existing `beluga_benchmark` package is to remain untouched to allow side-by-side coexistence.
