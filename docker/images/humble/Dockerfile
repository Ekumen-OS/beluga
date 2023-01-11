FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    curl \
    python3-pip \
    doxygen \
  && rm -rf /var/lib/apt/lists/*

RUN pip install \
  pre-commit==2.20.0

ARG USER=developer
ARG GROUP=ekumen

RUN addgroup --gid 1000 $GROUP \
  && adduser --uid 1000 --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
  && adduser $USER sudo \
  && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - \
  && chmod 4755 /usr/local/bin/fixuid \
  && mkdir -p /etc/fixuid \
  && printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

USER $USER:$GROUP

ENV USER_WS /ws

WORKDIR $USER_WS

COPY . $USER_WS/src

RUN sudo chown -R $USER:$GROUP $USER_WS

RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -i --from-path src -y \
  && sudo rm -rf /var/lib/apt/lists/*

ENV WITHIN_DEV 1

ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
