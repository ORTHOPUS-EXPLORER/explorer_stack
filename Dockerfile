## Overridable ROS distro argument (generic)
ARG ROS_DISTRO=iron
ARG ROS_USER=orthopus
ARG ROS_WS=/home/${ROS_USER}/src/

## Multi stage build

##  ---------------- Base / cache part ---------------- 
FROM osrf/ros:${ROS_DISTRO}-desktop AS explorer_cacher
ARG ROS_WS

## Overwrite apt-get defaults to prevent rosdep from installing optional packages
RUN rosdep update --rosdistro $ROS_DISTRO && \
    cat <<EOF > /etc/apt/apt.conf.d/docker-clean && apt-get update
APT::Install-Recommends "false";
APT::Install-Suggests "false";
EOF

WORKDIR ${ROS_WS}
COPY --exclude=build --exclude=install . ${ROS_WS}

## Derive build/exec dependencies into a /tmp/[build|exec]_dependencies.txt
## Taken from an official ROS image
RUN bash -e <<'EOF'
declare -A types=(
  [exec]="--dependency-types=build --dependency-types=exec --dependency-types=test --dependency-types=doc"
  [build]="--dependency-types=build --dependency-types=test")
for type in "${!types[@]}"; do
  rosdep install -y \
    --from-paths . \
    --ignore-src \
    --reinstall \
    --simulate \
    ${types[$type]} \
    | grep 'apt-get install' \
    | awk '{gsub(/'\''/,"",$4); print $4}' \
    | sort -u > /tmp/${type}_dependencies.txt
done
EOF


##  ---------------- Builder part (CI)---------------- 
# Lighter image for CI/CD
FROM amd64/ros:${ROS_DISTRO}-ros-base AS explorer_builder
LABEL org.opencontainers.image.source="https://github.com/ORTHOPUS-EXPLORER/explorer_stack"
LABEL org.opencontainers.image.description="CI/CD image for Orthopus Explorer project"
# LABEL org.opencontainers.image.licenses=
ARG ROS_WS
ARG ROS_USER
ENV ROS_USER=${ROS_USER}

COPY --from=explorer_cacher /tmp/build_dependencies.txt /tmp/build_dependencies.txt
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=explorer_cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=explorer_cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    # Install build dependencies from rosdep
    < /tmp/build_dependencies.txt xargs apt-get install -y --no-install-recommends \
    # CI build caching
    && apt install -y ccache


##  ---------------- Runner part (dev)---------------- 
FROM osrf/ros:${ROS_DISTRO}-desktop AS explorer_dev
LABEL org.opencontainers.image.source="https://github.com/ORTHOPUS-EXPLORER/explorer_stack"
LABEL org.opencontainers.image.description="Development image for Orthopus Explorer project"
# LABEL org.opencontainers.image.licenses=
ARG ROS_WS
ARG ROS_USER
ENV ROS_USER=${ROS_USER}

## Support / dev dependencies
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=explorer_cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=explorer_cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt install -y --no-install-recommends\
    bash-completion \
    can-utils \
    ccache \
    iproute2 \
    python3-pip \
    ros-${ROS_DISTRO}-plotjuggler \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    terminator \
    tmux \
    vim

RUN cat <<EOT >> ~/.bashrc
if [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
fi
EOT

# Control how far ROS nodes will try to discover each other: SUBNET|LOCALHOST|OFF|SYSTEM_DEFAULT
ENV ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"

RUN sed --in-place \
    # Source build (if exists) automatically
    -e '/^source .*/a [[ -f "${ROS_WS}/install/setup.bash" ]] && source "${ROS_WS}/install/setup.bash" --' \
    # Fix UID/GUID issue dynamically (issue about permissions when mounting folder)
    -e 's|^exec "$@"|if [ -z "${USER_UID}" ]; then\n\
    \techo "USER_UID env var not set, file permissions will occurs, please provides: -e USER_UID=\\$(id -u)"\n\
    else\n\
    \tusermod -u ${USER_UID} ${ROS_USER}\n\
    \tif [[ -d "/home/${ROS_USER}/.ccache" ]]; then\n\
    \t\tchown -R ${ROS_USER}:${ROS_USER} /home/${ROS_USER}/.ccache\n\
    \tfi\n\
    \tif [[ -d "${PWD}/.cache" ]]; then\n\
    \t\tchown -R ${ROS_USER}:${ROS_USER} ${PWD}/.cache\n\
    \tfi\n\
    \tif [[ -d "${PWD}/build" ]]; then\n\
    \t\tchown -R ${ROS_USER}:${ROS_USER} ${PWD}/build\n\
    \tfi\n\
    \tif [[ -d "${PWD}/install" ]]; then\n\
    \t\tchown -R ${ROS_USER}:${ROS_USER} ${PWD}/install\n\
    \tfi\n\
    fi\n\
    exec gosu ${ROS_USER} "$@"|' \
    /ros_entrypoint.sh

COPY --from=explorer_cacher /tmp/exec_dependencies.txt /tmp/
# Install build/exec dependencies from rosdep
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=explorer_cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=explorer_cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    ## Delete any clang related packages (latest version will be installed at next step)
    sed -i '/.*clang.*/d' /tmp/exec_dependencies.txt \
    && < /tmp/exec_dependencies.txt xargs apt-get install -y --no-install-recommends

# Install dev dependencies (clangd related)
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=explorer_cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=explorer_cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get install -y gosu software-properties-common wget \
    ## Install latest LLVM toolchain (clangd)
    && UBUNTU_CODENAME=$(lsb_release -c -s) \
    && wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | sudo tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc \
    && add-apt-repository -y deb http://apt.llvm.org/${UBUNTU_CODENAME}/ llvm-toolchain-${UBUNTU_CODENAME}-22 main \
    && apt update \
    # Install clang related packages
    && apt-get install -y --no-install-recommends clang-22 clang-tools-22 clang-22-doc libclang-common-22-dev libclang-22-dev libclang1-22 clang-format-22 python3-clang-22 clangd-22 clang-tidy-22 \
    && update-alternatives --install /usr/bin/clang clang /usr/bin/clang-22 22 \
    && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-22 22 \
    && update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-22 22 \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-22 22 \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-22 22

# Install dev dependencies (ruff related)
RUN pip install ruff

RUN groupadd -r ${ROS_USER} && useradd -m --no-log-init -r -g ${ROS_USER} ${ROS_USER}
# Copy colcon config (no need to reinstall mixins)
RUN cp -r /root/.colcon /home/${ROS_USER}
WORKDIR ${ROS_WS}


## ---------------- Runner part (prod) ----------------
FROM explorer_dev AS explorer_prod
LABEL org.opencontainers.image.description="Ready to uses image for Orthopus Explorer project"

COPY . ${ROS_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install --continue-on-error --mixin release