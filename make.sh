#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

# CMakePresets.json selects Ninja and the ARM GCC toolchain.
cmake --preset Debug
cmake --build --preset Debug --parallel "$(nproc)"
