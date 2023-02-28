FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    ccache \
    curl \
    doxygen \
    git \
    linux-cloud-tools-generic \
    linux-tools-common \
    linux-tools-generic \
    perl \
    python3-pip \
    texlive-base \
    texlive-binaries \
  && rm -rf /var/lib/apt/lists/*

RUN pip install \
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
RUN mkdir -p /home/$USER/.ccache

COPY . $USER_WS/src

RUN sudo chown -R $USER:$GROUP $USER_WS

RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -i --from-path src -y \
  && sudo rm -rf /var/lib/apt/lists/*

ENV WITHIN_DEV 1

ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
