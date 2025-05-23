FROM ros:noetic-ros-base-focal AS cacher

WORKDIR /ws/src

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    git \
    python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

COPY docker/files/noetic.repos .
RUN vcs import < noetic.repos

COPY . beluga/
RUN touch \
  beluga/beluga_benchmark/CATKIN_IGNORE \
  beluga/beluga_system_tests/CATKIN_IGNORE

RUN mkdir -p /tmp/ws/src \
  && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "CATKIN_IGNORE" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
  || true

FROM ros:noetic-ros-base-focal AS timem-builder

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

FROM ros:noetic-ros-base-focal AS builder

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    ccache \
    curl \
    gcovr \
    gdb \
    git \
    lcov \
    linux-cloud-tools-generic \
    linux-tools-common \
    linux-tools-generic \
    python3-colcon-common-extensions \
    python3-colcon-coveragepy-result \
    python3-colcon-lcov-result \
    python3-colcon-mixin \
    python3-pip \
    tmux \
  && rm -rf /var/lib/apt/lists/*

RUN pip install \
  evo==1.21.0 \
  pre-commit==2.20.0

COPY --from=timem-builder --chown=$USER:$GROUP /opt/timemory/ /opt/timemory
RUN cmake --build /opt/timemory/build --target install

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

RUN sudo apt-get update \
  && . /opt/ros/noetic/setup.sh \
  && rosdep update \
  && rosdep install -i -y --from-path src \
  && rosdep install -i -y --from-path src -t doc \
  && sudo rm -rf /var/lib/apt/lists/*

COPY --from=cacher --chown=$USER:$GROUP /ws/ $USER_WS/

RUN /bin/sh -c 'echo "export PATH=/usr/lib/ccache:\$PATH" >> ~/.bashrc'

ENV WITHIN_DEV 1

ENV SHELL /bin/bash
ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
