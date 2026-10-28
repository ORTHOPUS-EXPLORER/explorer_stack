#!/usr/bin/env bash
# Choose a ROS_DOMAIN_ID that is stable for this machine and unlikely to clash with
# another developer running the same stack on the same network.
#
# Runs on the host, from devcontainer.json's initializeCommand, and writes
# .devcontainer/.env which every container of this project injects with --env-file. The
# value is derived from the host name, so all containers on one machine agree while two
# machines differ.
set -eu
env_file="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.env"

if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  domain_id="${ROS_DOMAIN_ID}"          # explicit override always wins
else
  # 1..100. Domain 0 is what every unconfigured ROS 2 install uses, so it is the one
  # value guaranteed to collide. 101 and above overlap the Linux ephemeral port range:
  # RTPS uses ports 7400 + 250*domain .. +249, and the range starts at 32768, so domain
  # 100 ends at 32649 (safe) while domain 101 reaches 32899 (overlaps).
  domain_id=$(( ($(hostname | cksum | cut -d' ' -f1) % 100) + 1 ))
fi

# Replace only our own line, so anything else already in .env survives.
tmp="$(mktemp)"
[ -f "${env_file}" ] && grep -v '^ROS_DOMAIN_ID=' "${env_file}" > "${tmp}" || true
printf 'ROS_DOMAIN_ID=%s\n' "${domain_id}" >> "${tmp}"
mv "${tmp}" "${env_file}"

echo "ROS_DOMAIN_ID=${domain_id} (${env_file})"
