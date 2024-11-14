FROM ros:rolling-ros-base AS cacher

WORKDIR /ws/src

COPY docker/files/rolling.repos .
RUN vcs import < rolling.repos

COPY . beluga/

RUN mkdir -p /tmp/ws/src \
  && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
  || true

FROM ros:rolling-ros-base AS timem-builder

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    git \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN git clone -b develop https://github.com/NERSC/timemory.git \
  && cd timemory \
  && git checkout 415650ee26f358218908983c87212b620c3a0328 \
  && cmake -B ./build \
    -DTIMEMORY_INSTALL_HEADERS=OFF \
    -DTIMEMORY_INSTALL_CONFIG=OFF \
    -DTIMEMORY_INSTALL_ALL=OFF \
    -DTIMEMORY_BUILD_TIMEM=ON \
    . \
  && cmake --build ./build --target timem

FROM ros:rolling-ros-base AS builder

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    ccache \
    curl \
    gcovr \
    gdb \
    git \
    lcov \
    linux-tools-common \
    linux-tools-generic \
    python3-colcon-coveragepy-result \
    python3-colcon-lcov-result \
    python3-pip \
    libboost-all-dev \
    libopenvdb-dev \
    tmux \
  && rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES 1
RUN pip install \
  evo==1.21.0 \
  pre-commit==2.20.0 \
  plyfile==1.0.3

COPY --from=timem-builder --chown=$USER:$GROUP /opt/timemory/ /opt/timemory
RUN cmake --build /opt/timemory/build --target install

# Check if perf is working, if not replace it with the perf version available
RUN perf help || ( \
  mv /usr/bin/perf /usr/bin/perf.bkp && \
  ln -s $(ls -d /usr/lib/linux-tools/* | tail -n1)/perf /usr/bin/perf)

ARG USER=developer
ARG GROUP=ekumen

# Drop ubuntu user to avoid fixuid clashes.
RUN deluser ubuntu

RUN addgroup --gid 1001 $GROUP \
  && adduser --uid 1001 --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
  && adduser $USER sudo \
  && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

COPY docker/files/fixuid_config.yml /etc/fixuid/config.yml
RUN /bin/bash -c '\
     ARCH=`uname -m` && if [ "$ARCH" == "aarch64" ]; then FIXUID_ARCH="arm64"; else FIXUID_ARCH="amd64"; fi \
  && curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-$FIXUID_ARCH.tar.gz | tar -C /usr/local/bin -xzf - \
  && chmod 4755 /usr/local/bin/fixuid \
  && cd /etc/fixuid \
  && sed -i "s/_USER_/$USER/" config.yml \
  && sed -i "s/_GROUP_/$GROUP/" config.yml'

RUN cd /opt && git clone https://github.com/brendangregg/FlameGraph

USER $USER:$GROUP

ENV USER_WS /ws

WORKDIR $USER_WS

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
  && colcon mixin update default
COPY --chown=$USER:$GROUP docker/files/colcon_defaults.yaml /home/$USER/.colcon/defaults.yaml
RUN mkdir -p /home/$USER/.ccache $USER_WS/src

COPY --from=cacher --chown=$USER:$GROUP /tmp/ws/ $USER_WS/

USER root
ENV PIP_BREAK_SYSTEM_PACKAGES 1

RUN apt-get update \
  && apt-get -y dist-upgrade \
  && . /opt/ros/rolling/setup.sh \
  && rosdep update \
  && rosdep install -i -y --from-path src --skip-keys 'slam_toolbox' \
  && rosdep install -i -y --from-path src -t doc \
  && rm -rf /var/lib/apt/lists/*

USER $USER:$GROUP

COPY --from=cacher --chown=$USER:$GROUP /ws/ $USER_WS/

ENV WITHIN_DEV 1

ENV SHELL /bin/bash
ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
