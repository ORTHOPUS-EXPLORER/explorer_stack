#!/usr/bin/env bash
# Copy the host X11 authority cookie next to this script so the container can open windows
# on the host display (rviz, plotjuggler...).
#
# Runs on the host, from devcontainer.json's initializeCommand. The file is always created
# (empty when no cookie is found) because devcontainer.json bind-mounts it.
set -eu
xauth_file="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.xauthority"

if [ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ]; then
  cp "${XAUTHORITY}" "${xauth_file}"
elif [ -f "${HOME}/.Xauthority" ]; then
  cp "${HOME}/.Xauthority" "${xauth_file}"
else
  touch "${xauth_file}"
fi

chmod 600 "${xauth_file}"
