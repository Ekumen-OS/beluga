#!/bin/bash

# Get the directory where this script is stored
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}" .sh)"
PATCH_FILE="${SCRIPT_DIR}/${SCRIPT_NAME}.patch"

# Apply patch if it exists
if [[ -f "$PATCH_FILE" ]]; then
  echo "Applying patch: $PATCH_FILE"
  patch -p1 < "$PATCH_FILE"
else
  echo "No patch file found: $PATCH_FILE"
fi

# Keep only the required nav2 packages and remove all others to avoid bringing packages
# that are not yet ready for lyrical
KEEP_DIRS=("nav2_amcl" "nav2_map_server" "nav2_lifecycle_manager" "nav2_ros_common" "nav2_common" "nav2_msgs" "nav2_util")

for dir in */; do
  dir="${dir%/}"  # Remove trailing slash
  keep=false
  for keep_dir in "${KEEP_DIRS[@]}"; do
    if [[ "$dir" == "$keep_dir" ]]; then
      keep=true
      break
    fi
  done

  if [[ "$keep" == false ]]; then
    echo "Removing $dir"
    rm -rf "$dir"
  else
    echo "Keeping $dir"
  fi
done
