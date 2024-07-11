FROM ekumenlabs/lambkin:ros-humble-jammy AS cacher

WORKDIR /ws/src

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

COPY docker/files/humble.repos .
RUN vcs import < humble.repos

COPY . beluga/

RUN mkdir -p /tmp/ws/src \
  && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
  || true

FROM ekumenlabs/lambkin:ros-humble-jammy AS builder

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    ccache \
    curl \
    gdb \
    git \
    lcov \
    linux-tools-common \
    linux-tools-generic \
    python3-colcon-coveragepy-result \
    python3-colcon-lcov-result \
    python3-pip \
    ros-dev-tools \
    tmux \
  && rm -rf /var/lib/apt/lists/*

RUN pip install \
  linuxdoc \
  sphinxcontrib.datatemplates \
  sphinxcontrib-repl \
  pre-commit==2.20.0

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
  && . /opt/ros/humble/setup.sh \
  && rosdep update \
  && rosdep install -i -y --from-path src \
  && rosdep install -i -y --from-path src -t doc \
  && sudo rm -rf /var/lib/apt/lists/*

COPY --from=cacher --chown=$USER:$GROUP /ws/ $USER_WS/

ENV WITHIN_DEV 1

ENV SHELL /bin/bash
ENTRYPOINT ["fixuid", "-q", "/bin/bash", "-l"]
