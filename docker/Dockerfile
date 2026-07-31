# rosetta_ws devcontainer image — a thin wrapper around the pixi setup.
#
# This image is toolchain only: CUDA base, the pixi binary, X11/GL client
# libs, and a non-root user. No source, no dependencies, no colcon build —
# all of that lives in the bind-mounted workspace and is provisioned by
# `pixi run setup` (devcontainer postCreateCommand or manually), exactly like
# a host checkout. Rebuilding the image never touches src/, libs/, or the
# .pixi env (named volume).
#
# Usage: .devcontainer/x86/devcontainer.json → "target": "dev"
FROM ghcr.io/prefix-dev/pixi:0.72.2-noble-cuda-12.9.1 AS dev

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System deps pixi can't provide: X11/GL client libs so RViz/rqt render
# through the host display server, plus basic dev conveniences.
RUN apt-get update && apt-get install -y --no-install-recommends \
      locales \
      tzdata \
      ca-certificates \
      git \
      git-lfs \
      sudo \
      vim \
      less \
      libglvnd0 \
      libgl1 \
      libglx0 \
      libegl1 \
      libx11-6 \
      libxext6 \
      libxrandr2 \
      libxcursor1 \
      libxinerama1 \
      libxi6 \
      libxss1 \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 LANGUAGE=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Create non-root user matching the host UID/GID (1000:1000).
# The base image may already have GID/UID 1000 under a different name.
RUN existing_group=$(getent group 1000 | cut -d: -f1) \
    && if [ -z "$existing_group" ]; then groupadd --gid 1000 ros; \
       elif [ "$existing_group" != "ros" ]; then groupmod -n ros "$existing_group"; fi \
    && existing_user=$(getent passwd 1000 | cut -d: -f1) \
    && if [ -z "$existing_user" ]; then useradd -ms /bin/bash --uid 1000 --gid 1000 ros; \
       elif [ "$existing_user" != "ros" ]; then usermod -l ros -d /home/ros -m "$existing_user"; fi \
    && mkdir -p /home/ros/.cache \
    && chown -R ros:ros /home/ros \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros

# Enable pixi bash completion for the ros user
RUN echo 'eval "$(pixi completion -s bash)"' >> /home/ros/.bashrc

USER ros
WORKDIR /workspaces/rosetta_ws
