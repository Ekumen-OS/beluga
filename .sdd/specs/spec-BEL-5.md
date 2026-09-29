# Technical Specification: BEL-5 (Integrate Lambkin)

## Overview
This specification details the integration of the Lambkin benchmark package into the Beluga 2 repository. Lambkin will remain an external dependency rather than being fully vendored. The deployment container will install Lambkin directly from its GitHub repository at a pinned commit, specifically for Jazzy, Kilted, and Rolling distributions.

## 1. Beluga-Evaluation Files
We will only keep the Beluga-specific benchmark files from Lambkin's `examples/beluga` folder.
- **Target Location**: A new top-level directory `beluga_lambkin/` in the Beluga repository.
- **Files to include**:
  - `beluga_benchmark.py` (the evaluation script). It MUST be modified to use `laser_model_type` and `max_particles` instead of `sensor_model` and `num_particles`.
  - `beluga_ros2/` (the entire ROS 2 package containing launch and params).
- **Files to DROP (Do NOT include)**:
  - `docker/` (the redundant nested Docker configuration).
- **Lambkin Requirements File**:
  - Create `beluga_lambkin/requirements.txt` containing only:
    `lambkin @ git+https://github.com/Ekumen-OS/lambkin.git@d9512d8b674fcea785833f58ee7a2c386b5f5b3a`
  - Do NOT create `beluga_lambkin/Dockerfile`.

## 2. Docker Configuration Updates
We will integrate the Lambkin installation directly into the main Beluga development Dockerfiles. 

- **Target Distributions**: Update `docker/images/jazzy/Dockerfile`, `docker/images/kilted/Dockerfile`, and `docker/images/rolling/Dockerfile`.
- **Humble**: Do NOT update `docker/images/humble/Dockerfile` (Humble is explicitly excluded from this integration to avoid `uv-build` compatibility issues).
- **Changes**:
  1. Modify the existing `evo` installation to `evo>=1.34.3`.
  2. Right after the `evo/numpy/pre-commit` pip install layer, append the following snippet to install Lambkin with an exponential retry backoff. To resolve Debian-managed package errors on newer distributions (e.g. `typing_extensions` on Kilted), use `--break-system-packages`.

```dockerfile
# Install Lambkin SDK (pinned in beluga_lambkin/requirements.txt), retrying on transient failures.
COPY beluga_lambkin/requirements.txt /tmp/lambkin-requirements.txt
RUN set -eu; \
    for delay in 1 2 4 8; do \
      pip install --no-cache-dir --break-system-packages -r /tmp/lambkin-requirements.txt && exit 0; \
      echo "pip install failed, retrying in ${delay}s..." >&2; \
      sleep "$delay"; \
    done; \
    pip install --no-cache-dir --break-system-packages -r /tmp/lambkin-requirements.txt
```

## 3. Pull Request & Git Constraints
- **DCO Compliance via Force-Push**: The PR branch (`jtlorente/b39bc7e8-e451-4dd3-af53-8d21c642872e`) has a failing DCO check due to an unsigned commit (`af560aa38`). The Implementer MUST perform an interactive rebase (`git rebase -i`) to squash commits and ensure all commits have the `Signed-off-by` trailer, then execute a **force-push** to update the PR.
- **PR Description**: Rename the PR to "Add Lambkin benchmark for Beluga AMCL" and write a complete description including: summary, changes, run instructions, a short testing list, and noted pre-existing CI failures (like Rolling).
