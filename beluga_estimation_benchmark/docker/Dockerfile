# Copyright 2026 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

ARG ROSDISTRO=jazzy
FROM ekumenlabs/beluga-${ROSDISTRO}-dev

ARG ROSDISTRO=jazzy

USER root

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-matplotlib \
    ros-${ROSDISTRO}-robot-localization \
 && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:0.13.6 /uv /uvx /bin/

ENV UV_TOOL_DIR=/opt/uv/tools
ENV UV_TOOL_BIN_DIR=/usr/local/bin

RUN uv tool install evo==1.34.3

USER developer

ENV VIRTUAL_ENV=/home/developer/.venv
ENV PATH="/home/developer/.venv/bin:/home/developer/.local/bin:$PATH"

RUN uv venv --system-site-packages /home/developer/.venv

COPY beluga_estimation_benchmark/docker/requirements.txt /tmp/requirements.txt
RUN uv pip install --no-cache -r /tmp/requirements.txt "numpy==$(python3 -c 'import numpy; print(numpy.__version__)')"
