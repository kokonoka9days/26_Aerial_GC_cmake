#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="$(basename "$ROOT_DIR")"
ELF_FILE="${ELF_FILE:-$ROOT_DIR/build/Debug/${PROJECT_NAME}.elf}"
OPENOCD_TARGET="${OPENOCD_TARGET:-target/stm32f4x.cfg}"
OPENOCD_SPEED="${OPENOCD_SPEED:-1000}"

if [[ ! -f "$ELF_FILE" ]]; then
    echo "ELF file not found: $ELF_FILE" >&2
    echo "Run make.sh first." >&2
    exit 1
fi

# jlink
openocd \
  -f interface/jlink.cfg \
  -c "transport select swd" \
  -f "$OPENOCD_TARGET" \
  -c "adapter speed $OPENOCD_SPEED" \
  -c "program $ELF_FILE verify reset exit"

# # daplink
# openocd \
#   -f interface/cmsis-dap.cfg \
#   -c "transport select swd" \
#   -f "$OPENOCD_TARGET" \
#   -c "adapter speed 100" \
#   -c "init; reset halt; targets; shutdown"
