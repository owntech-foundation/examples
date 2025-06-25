#!/bin/bash
set -euo pipefail

EXAMPLE_REPO_DIR=$(pwd)
CORE_DIR="core"
OUTPUT_DIR="$EXAMPLE_REPO_DIR/binaries"

if [ "$#" -eq 0 ]; then
  EXAMPLES=$(jq -r '.examples[].name' "$EXAMPLE_REPO_DIR/library.json")
else
  EXAMPLES="$@"
fi

cd "$CORE_DIR"
for EX in $EXAMPLES; do
  BASE=$(jq -r --arg ex "$EX" '.examples[] | select(.name==$ex) | .base' "$EXAMPLE_REPO_DIR/library.json")
  echo "Building $EX -> $BASE"
  platformio run -t "$EX"
  DEST="$OUTPUT_DIR/$BASE"
  mkdir -p "$DEST"
  cp .pio/build/*/firmware.mcuboot.bin "$DEST/firmware.mcuboot.bin"
  echo "Stored firmware for $EX at $DEST"
done
