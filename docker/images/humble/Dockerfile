FROM ros:humble-ros-base-jammy AS cacher

WORKDIR /ws/src

COPY docker/files/humble.repos .
RUN vcs import < humble.repos

COPY . beluga/

RUN mkdir -p /tmp/ws/src \
  && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
  || true

FROM ros:humble-ros-base-jammy AS builder

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    ccache \
    curl \
    doxygen \
    gdb \
    git \
    lcov \
    linux-cloud-tools-generic \
    linux-tools-common \
    linux-tools-generic \
    perl \
    python3-colcon-coveragepy-result \
    python3-pip \
    texlive-base \
    texlive-binaries \
    tmux \
  && rm -rf /var/lib/apt/lists/*

# Fix for the RViz black screen issue.
# TODO(nahuel): Remove this when the Mesa version available
# in Ubuntu 22.04 is 22.3.5 or greater.
# See https://github.com/Ekumen-OS/beluga/issues/123
RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    software-properties-common \
  && add-apt-repository -y ppa:kisak/kisak-mesa \
  && apt-get install --no-install-recommends -y \
    libegl-mesa0 libglx-mesa0 \
  && rm -rf /var/lib/apt/lists/*

RUN pip install \
  colcon-lcov-result==0.5.0 \
  evo==1.21.0 \
  pre-commit==2.20.0 \
  # Note: Installing timemory takes a long time.
  timemory==3.3.0rc4

# Check if perf is working, if not replace it with the perf version available
RUN perf help || ( \
  mv /usr/bin/perf /usr/bin/perf.bkp && \
  ln -s $(ls -d /usr/lib/linux-tools/* | tail -n1)/perf /usr/bin/perf)

ARG USER=developer
ARG GROUP=ekumen

RUN addgroup --gid 1000 $GROUP \
  && adduser --uid 1000 --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
  && adduser $USER sudo \
  && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

COPY docker/files/fixuid_config.yml /etc/fixuid/config.yml
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - \
  && chmod 4755 /usr/local/bin/fixuid \
  && cd /etc/fixuid \
  && sed -i "s/_USER_/$USER/" config.yml \
  && sed -i "s/_GROUP_/$GROUP/" config.yml

RUN cd /opt && git clone https://github.com/brendangregg/FlameGraph

USER $USER:$GROUP

ENV USER_WS /ws

WORKDIR $USER_WS

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
  && colcon mixin update default
COPY --chown=$USER:$GROUP docker/files/colcon_defaults.yaml /home/$USER/.colcon/defaults.yaml
RUN mkdir -p /home/$USER/.ccache $USER_WS/src

COPY --from=cacher --chown=$USER:$GROUP /tmp/ws/ $USER_WS/

RUN sudo apt-get update \
  && . /opt/ros/humble/setup.sh \
  && rosdep update \
  && rosdep install -i --from-path src -y \
  && sudo rm -rf /var/lib/apt/lists/*

COPY --from=cacher --chown=$USER:$GROUP /ws/ $USER_WS/

ENV WITHIN_DEV 1

ENV SHELL /bin/bash
ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
