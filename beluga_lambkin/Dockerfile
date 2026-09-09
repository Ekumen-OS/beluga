FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    git \
    python3-pip \
  && rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Install Lambkin SDK externally from a pinned commit on main with exponential retry backoff and jitter
RUN printf "import random, time, subprocess, sys\n\
cmd = ['pip', 'install', 'numpy==1.26.4', 'git+https://github.com/Ekumen-OS/lambkin.git@d9512d8b674fcea785833f58ee7a2c386b5f5b3a']\n\
base, factor, cap, max_attempts = 1, 2, 60, 5\n\
for attempt in range(max_attempts):\n\
    print(f'Attempt {attempt + 1}/{max_attempts} to install lambkin...', flush=True)\n\
    res = subprocess.run(cmd)\n\
    if res.returncode == 0:\n\
        print('Installation succeeded!', flush=True)\n\
        sys.exit(0)\n\
    if attempt == max_attempts - 1:\n\
        print('Max attempts reached. Installation failed.', flush=True)\n\
        sys.exit(res.returncode)\n\
    sleep_time = random.uniform(0, min(cap, base * (factor ** attempt)))\n\
    print(f'Installation failed. Retrying in {sleep_time:.2f} seconds...', flush=True)\n\
    time.sleep(sleep_time)\n\
" > install_lambkin.py \
  && python3 install_lambkin.py \
  && rm install_lambkin.py
