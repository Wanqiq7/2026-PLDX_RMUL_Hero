#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRESET="${1:-Debug}"
BUILD_JOBS="${BUILD_JOBS:-24}"

case "${PRESET}" in
  Debug|Release)
    ;;
  *)
    echo "Error: Unsupported preset '${PRESET}'. Use Debug or Release."
    exit 1
    ;;
esac

echo "--- Configuring project with CMake preset: ${PRESET} ---"
cmake --preset "${PRESET}" -S "${SCRIPT_DIR}"

echo "--- Building project with Ninja (${BUILD_JOBS} jobs) ---"
cmake --build --preset "${PRESET}" --parallel "${BUILD_JOBS}"

echo "--- Done! ---"
