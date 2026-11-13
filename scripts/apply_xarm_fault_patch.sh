#!/usr/bin/env bash
# Preserve local vendor corrections across fresh submodule checkouts.
set -eo pipefail
readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly VENDOR_ROOT="${PROJECT_ROOT}/ros_ws/src/xarm_ros2"
readonly PATCH_PATH="${PROJECT_ROOT}/patches/xarm_communication_faults.patch"
readonly PATCH_BASE=62936f7ea1846a85f7350de2c4c18f39e6d19715
if [[ "$(git -C "$VENDOR_ROOT" rev-parse HEAD)" != "$PATCH_BASE" ]]; then
  echo 'The xarm_ros2 revision changed. Review the communication patch against the new vendor source before building.' >&2
  exit 1
fi
if git -C "$VENDOR_ROOT" apply --reverse --check "$PATCH_PATH" 2>/dev/null; then
  echo 'xArm communication fault patch already applied.'
  exit 0
fi
if ! git -C "$VENDOR_ROOT" apply --check "$PATCH_PATH"; then
  echo 'Vendor files differ from the patch base; no changes applied. Resolve the source differences before building.' >&2
  exit 1
fi
git -C "$VENDOR_ROOT" apply "$PATCH_PATH"
echo 'Applied xArm communication fault handling patch.'
