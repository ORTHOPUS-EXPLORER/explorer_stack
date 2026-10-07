#!/usr/bin/env bash
# Copies the host X11 cookie for container GUIs (run on host by initializeCommand).
# Always creates the file even when nothing inside because devcontainer.json bind-mounts it.
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
