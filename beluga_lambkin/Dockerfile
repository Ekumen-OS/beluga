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