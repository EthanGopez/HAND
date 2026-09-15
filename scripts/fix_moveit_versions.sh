#!/usr/bin/env bash
set -eo pipefail

# A hand-run `apt-get install --reinstall` on some machines upgraded only
# some ros-humble-moveit* packages to a newer apt snapshot, leaving others
# behind. The resulting version skew breaks plugin loading at runtime, e.g.
# move_group failing to dlopen libmoveit_ompl_planner_plugin.so because it
# was built against a libmoveit_planning_interface.so that never got
# upgraded. This script detects that skew and re-syncs every moveit-family
# package to a single version.

readonly PACKAGE_PATTERN='^ros-humble-(moveit|ompl|pilz-industrial-motion-planner)'

# moveit_msgs (2.2.x) and ompl (1.7.x) are separate upstream projects with
# their own version schemes; only packages released alongside MoveIt core
# itself (2.5.x) are expected to share the same upstream version. Each
# binary also carries its own build-timestamp suffix even within a single
# healthy release (e.g. 2.5.10-1jammy.20260908.071525 vs
# 2.5.10-1jammy.20260908.204514), so comparison must strip that suffix and
# look only at the upstream version (the part before the first '-').
readonly ABI_VERSION_PATTERN='^2\.5\.'

mapfile -t packages < <(dpkg-query -W -f '${Package} ${Version}\n' 2>/dev/null \
  | grep -E "${PACKAGE_PATTERN}" || true)

if [[ "${#packages[@]}" -eq 0 ]]; then
  echo 'No ros-humble-moveit packages are installed; nothing to check.'
  exit 0
fi

abi_packages=()
for entry in "${packages[@]}"; do
  if [[ "${entry#* }" =~ ${ABI_VERSION_PATTERN} ]]; then
    abi_packages+=("${entry}")
  fi
done

upstream_versions="$(printf '%s\n' "${abi_packages[@]}" | awk '{print $2}' | cut -d- -f1 | sort -u)"
version_count="$(printf '%s\n' "${upstream_versions}" | grep -c .)"

if [[ "${version_count}" -le 1 ]]; then
  echo "All ${#abi_packages[@]} MoveIt-core-family packages are on upstream version ${upstream_versions}."
  exit 0
fi

echo "Found ${version_count} different upstream versions among MoveIt-core-family packages:"
printf '%s\n' "${abi_packages[@]}" | sort -k2
echo
echo 'Re-syncing the whole moveit/ompl/pilz package set to the current apt candidate version...'

names=()
for entry in "${packages[@]}"; do
  names+=("${entry%% *}")
done

apt-get update
apt-get install --only-upgrade -y "${names[@]}"

echo 'Done. Re-run scripts/build_ros_workspace.sh if this changed any headers/libs your workspace links against.'
