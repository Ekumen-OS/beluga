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

# Install uv from the official binary distribution
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Set up virtual environment inheriting system site-packages (for apt numpy, matplotlib)
ENV VIRTUAL_ENV=/home/developer/.venv
RUN uv venv --system-site-packages $VIRTUAL_ENV

# Ensure venv and uv tools are at the front of PATH
ENV PATH="$VIRTUAL_ENV/bin:/home/developer/.local/bin:$PATH"

# Install Lambkin into the virtual environment from requirements.txt
COPY beluga_lambkin/requirements.txt /tmp/requirements.txt
RUN uv pip install --no-cache -r /tmp/requirements.txt

# Install evo using uv tool in an isolated environment (GPL-3.0 isolated CLI)
RUN uv tool install evo
