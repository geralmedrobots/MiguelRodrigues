#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${D455_SENSOR_IMAGE:-pharmarobot:d455-sensor}"
BASE_IMAGE="${D455_SENSOR_BASE_IMAGE:-ros:humble-ros-base-jammy}"
IMAGE_MANIFEST="${D455_SENSOR_IMAGE_MANIFEST:-/var/lib/pharmarobot/d455-sensor-image.env}"

base_id="$(
  docker image inspect --format '{{.Id}}' "$BASE_IMAGE"
)"
if [[ ! "$base_id" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  echo "[d455-sensor-build] base image identity is not a digest" >&2
  exit 1
fi

context_dir="$(mktemp -d)"
base_alias=""
base_alias_owned=0
cleanup()
{
  if [[ "$base_alias_owned" == "1" && -n "$base_alias" ]]; then
    docker image rm "$base_alias" >/dev/null 2>&1 || true
  fi
  rm -rf "$context_dir"
}
trap cleanup EXIT

(
  cd "$ROOT_DIR"
  tar \
    --exclude='src/realsense_imu/validation_evidence' \
    --exclude='*/.pytest_cache' \
    --exclude='*/__pycache__' \
    --exclude='*/build' \
    --exclude='*/install' \
    --exclude='*/log' \
    --exclude='*.pyc' \
    --exclude='*.pyo' \
    -cf - \
    src/realsense_imu \
    deployment/docker/Dockerfile.d455_sensor \
    deployment/scripts/d455_sensor_entrypoint.sh
) | tar -xf - -C "$context_dir"

source_manifest="$(
  cd "$context_dir"
  {
    find src/realsense_imu -type f \
      ! -path '*/validation_evidence/*' \
      ! -path '*/.pytest_cache/*' \
      ! -path '*/__pycache__/*' \
      ! -path '*/build/*' \
      ! -path '*/install/*' \
      ! -path '*/log/*' \
      ! -name '*.pyc' \
      ! -name '*.pyo' -print0
    printf '%s\0' \
      deployment/docker/Dockerfile.d455_sensor \
      deployment/scripts/d455_sensor_entrypoint.sh
  } | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}'
)"
base_alias="pharmarobot:d455-sensor-base-${source_manifest:0:16}"

if docker image inspect "$base_alias" >/dev/null 2>&1; then
  echo "[d455-sensor-build] run-owned base alias already exists" >&2
  exit 1
fi
docker image tag "$base_id" "$base_alias"
base_alias_owned=1
alias_id="$(
  docker image inspect --format '{{.Id}}' "$base_alias"
)"
if [[ "$alias_id" != "$base_id" ]]; then
  echo "[d455-sensor-build] local base alias identity mismatch" >&2
  exit 1
fi

docker build \
  --pull=false \
  --build-arg "ROS_BASE_IMAGE=$base_alias" \
  --build-arg "BASE_IMAGE_ID=$base_id" \
  --build-arg "SOURCE_MANIFEST_SHA256=$source_manifest" \
  --label "pharmarobot.d455.source-manifest-sha256=$source_manifest" \
  --file "$context_dir/deployment/docker/Dockerfile.d455_sensor" \
  --tag "$IMAGE" \
  "$context_dir"

read -r built_id built_sensor_label built_source built_base < <(
  docker image inspect \
    --format \
    '{{.Id}} {{index .Config.Labels "pharmarobot.d455.sensor-image"}} {{index .Config.Labels "pharmarobot.d455.source-manifest-sha256"}} {{index .Config.Labels "pharmarobot.d455.base-image-id"}}' \
    "$IMAGE"
)
if [[ ! "$built_id" =~ ^sha256:[0-9a-f]{64}$ ]] ||
   [[ "$built_sensor_label" != "true" ]] ||
   [[ "$built_source" != "$source_manifest" ]] ||
   [[ "$built_base" != "$base_id" ]]; then
  echo "[d455-sensor-build] built image provenance verification failed" >&2
  exit 1
fi

mkdir -p "$(dirname "$IMAGE_MANIFEST")"
manifest_tmp="$(mktemp "${IMAGE_MANIFEST}.tmp.XXXXXX")"
trap 'rm -f "$manifest_tmp"; cleanup' EXIT
printf '%s\n' \
  "IMAGE=$IMAGE" \
  "IMAGE_ID=$built_id" \
  "SOURCE_MANIFEST_SHA256=$source_manifest" \
  "BASE_IMAGE_ID=$base_id" \
  > "$manifest_tmp"
chmod 0644 "$manifest_tmp"
mv -f "$manifest_tmp" "$IMAGE_MANIFEST"
trap cleanup EXIT

printf '[d455-sensor-build] image=%s id=%s manifest=%s\n' \
  "$IMAGE" "$built_id" "$IMAGE_MANIFEST"
