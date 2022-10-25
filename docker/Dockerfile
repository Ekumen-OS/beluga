FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    apt-transport-https \
    apt-utils \
    bash-completion \
    build-essential \
    ca-certificates \
    clang-format \
    clang-tidy \
    cmake \
    curl \
    git \
    software-properties-common \
    sudo \
  && rm -rf /var/lib/apt/lists/*

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

WORKDIR /ws

COPY . /ws

ENV WITHIN_DEV 1

ENTRYPOINT ["fixuid", "-q", "/bin/bash"]
