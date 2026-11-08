#!/bin/bash

echo "=============================="
echo "[GUARD] RAW ARGS:"
echo "$@"
echo "=============================="

TARGET="$1"
shift

ARGS=("$@")

GUARD_PROJECT_ROOT="$(dirname "$(dirname "$(realpath "$TARGET")")")"
source "$GUARD_PROJECT_ROOT/script/setup_ros_runtime.sh"
dts_setup_ros_runtime "$GUARD_PROJECT_ROOT" || exit 1

echo "[GUARD] TARGET EXE:"
echo "$TARGET"

echo "=============================="
echo "[GUARD] ARG COUNT: $#"
echo "=============================="

INDEX=0
for arg in "${ARGS[@]}"; do
    echo "[GUARD] ARG[$INDEX] = $arg"
    ((INDEX++))
done

echo "=============================="
echo "[GUARD] FULL COMMAND:"
echo "$TARGET ${ARGS[*]}"
echo "=============================="

echo "[GUARD] Starting monitor..."

while true; do
    echo "[GUARD] Launching program..."

    "$TARGET" "${ARGS[@]}"
    RET=$?

    echo "[GUARD] Process exited with code: $RET"

    if [ $RET -eq 0 ]; then
        echo "[GUARD] Program exited normally. Stopping guard."
        exit 0
    fi

    echo "[GUARD] Crash detected, restarting in 1 second..."
    sleep 1
done
