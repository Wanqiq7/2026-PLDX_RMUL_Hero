#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRESET="${1:-Debug}"
BUILD_JOBS="${BUILD_JOBS:-24}"
OPENOCD_CFG="${OPENOCD_CFG:-openocd_dap.cfg}"
ELF_PATH="${SCRIPT_DIR}/build/${PRESET}/basic_framework.elf"

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

if [ ! -f "${ELF_PATH}" ]; then
  echo "Error: ELF not found at ${ELF_PATH}"
  exit 1
fi

echo "--- Flashing firmware (${ELF_PATH}) ---"
openocd -f "${OPENOCD_CFG}" -c "program ${ELF_PATH} verify reset exit"

echo "--- Done! ---"
