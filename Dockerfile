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
COPY . ${ROS_WS}

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
RUN apt update && apt install -y --no-install-recommends\
    can-utils \
    ccache \
    iproute2 \
    python3-pip \
    ros-${ROS_DISTRO}-plotjuggler \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    terminator \
    tmux \
    vim \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_LOCALHOST_ONLY=1
RUN sed --in-place \
    # Source build (if exists) automatically
    -e '/^source .*/a [[ -f "${ROS_WS}/install/setup.bash" ]] && source "${ROS_WS}/install/setup.bash" --' \
    # Fix UID/GUID issue dynamically (issue about permissions when mounting folder)
    -e 's|^exec "$@"|if [ -z "${USER_UID}" ]; then\n    echo "USER_UID env var not set, file permissions will occurs, please provides: -e USER_UID=\\$(id -u)"\nelse\n    usermod -u $USER_UID ${ROS_USER}\nfi\nexec gosu ${ROS_USER} "$@"|' \
    /ros_entrypoint.sh

COPY --from=explorer_cacher /tmp/exec_dependencies.txt /tmp/
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=explorer_cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=explorer_cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    # Install build/exec dependencies from rosdep
    < /tmp/exec_dependencies.txt xargs apt-get install -y --no-install-recommends \
    # Install gosu
    && apt-get install -y --no-install-recommends gosu

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