#!/usr/bin/env bash
set -euo pipefail

devices=(/dev/roboteq /dev/lidar_front /dev/lidar_back)
resolved=()
failed=0

for device in "${devices[@]}"; do
  if [[ ! -e "$device" ]]; then
    echo "MISSING: $device"
    failed=1
    continue
  fi

  real_device="$(readlink -f "$device")"
  resolved+=("$real_device")
  printf '%-18s -> %s\n' "$device" "$real_device"
done

if [[ $failed -ne 0 ]]; then
  exit 1
fi

if [[ "${resolved[0]}" == "${resolved[1]}" || \
      "${resolved[0]}" == "${resolved[2]}" || \
      "${resolved[1]}" == "${resolved[2]}" ]]; then
  echo "ERROR: Two persistent names point to the same physical device." >&2
  exit 1
fi

echo "All persistent USB names exist and point to distinct devices."
