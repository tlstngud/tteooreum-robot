#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_FILE="$ROOT_DIR/tools/serbot_runtime.py"
HOST="${1:-${SERBOT_HOST:-soda@192.168.0.7}}"
DEST_DIR="${2:-/home/soda/ros_ws/tools}"

SSH_OPTS=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
)

SSH_PREFIX=()
if command -v sshpass >/dev/null 2>&1 && [[ -n "${SERBOT_PASSWORD:-}" ]]; then
  SSH_PREFIX=(sshpass -p "$SERBOT_PASSWORD")
fi

"${SSH_PREFIX[@]}" ssh "${SSH_OPTS[@]}" "$HOST" "mkdir -p '$DEST_DIR'"
"${SSH_PREFIX[@]}" scp "${SSH_OPTS[@]}" "$SRC_FILE" "$HOST:$DEST_DIR/serbot_runtime.py"
"${SSH_PREFIX[@]}" ssh "${SSH_OPTS[@]}" "$HOST" "chmod +x '$DEST_DIR/serbot_runtime.py'"

echo "deployed=$HOST:$DEST_DIR/serbot_runtime.py"
