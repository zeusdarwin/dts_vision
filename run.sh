#!/bin/bash


WORK_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
BUILD_DIR="$WORK_DIR/build"
CONFIG_DIR="$WORK_DIR/config"
BIN_DIR="$WORK_DIR/bin"


export VISION_ROOT="$WORK_DIR"
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$WORK_DIR/lib:$LD_LIBRARY_PATH


blue="\033[1;34m"
yellow="\033[1;33m"
red="\033[1;31m"
reset="\033[0m"

if [ "$EUID" -eq 0 ]; then
    # systemd does not set SUDO_USER; use the service user in that case.
    RUN_USER="${SUDO_USER:-${USER:-root}}"
    USER_HOME=$(getent passwd "$RUN_USER" | cut -d: -f6)
    COPY_BASHRC="$WORK_DIR/output/user_bashrc_copy.bash"
    if [ -n "$USER_HOME" ] && [ -f "$USER_HOME/.bashrc" ]; then
        tail -n +11 "$USER_HOME/.bashrc" > "$COPY_BASHRC"
        chmod 644 "$COPY_BASHRC"
        chown "$RUN_USER:$RUN_USER" "$COPY_BASHRC"
        source "$COPY_BASHRC"
    elif [ -f "$COPY_BASHRC" ]; then
        source "$COPY_BASHRC"
    fi
else
    [ -f "$HOME/.bashrc" ] && source "$HOME/.bashrc"
fi


current_time=$(date +%s)
find "$WORK_DIR" -type f \
  ! -path "*/build/*" \
  ! -path "*/bin/*" \
  -newermt "$(date '+%Y-%m-%d %H:%M:%S')" \
  -exec touch {} \;
touch "$WORK_DIR"/tools/relink.cpp


do_build() {
    echo -e "${yellow}\n<--- Total Lines --->${reset}"
    total=$(find "$WORK_DIR" \
        -type d \( \
            -path "$BUILD_DIR" -o \
            -path "$WORK_DIR/model" -o \
            -path "$WORK_DIR/3rdparty" -o \
            -path "$WORK_DIR/.cache" \
        \) -prune -o \
        -type f \( \
            -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" \
            -o -name "*.py" -o -name "*.html" -o -name "*.sh" -o -name "*.md" \
            -o -name "*.yaml" -o -name "*.json" -o -name "*.css" -o -name "*.js" \
            -o -name "*.cu" -o -name "*.txt" \
        \) -exec wc -l {} + | awk 'END{print $1}')
    echo -e "${blue}        $total${reset}"
    mkdir -p "$BUILD_DIR"
    echo -e "${yellow}<--- Start CMake (Ninja) --->${reset}"
	# Prefer the pinned GCC 12 toolchain when available, but fall back to the
	# distribution compiler (for example GCC 11 on the Jetson image).
	C_COMPILER="${CC:-gcc-12}"
	CXX_COMPILER="${CXX:-g++-12}"
	if [ -z "${CC:-}" ] && ! command -v "$C_COMPILER" >/dev/null 2>&1; then
	    C_COMPILER="$(command -v gcc || true)"
	fi
	if [ -z "${CXX:-}" ] && ! command -v "$CXX_COMPILER" >/dev/null 2>&1; then
	    CXX_COMPILER="$(command -v g++ || true)"
	fi
	if [ -z "$C_COMPILER" ] || [ -z "$CXX_COMPILER" ]; then
	    echo -e "${red}No usable C/C++ compiler found. Install gcc and g++, or set CC/CXX.${reset}"
	    exit 1
	fi
	echo "Using C compiler: $C_COMPILER"
	echo "Using C++ compiler: $CXX_COMPILER"

	cmake -S "$WORK_DIR" -B "$BUILD_DIR" \
	    -G Ninja \
	    -DCMAKE_C_COMPILER="$C_COMPILER" \
	    -DCMAKE_CXX_COMPILER="$CXX_COMPILER"
    if [ $? -ne 0 ]; then
        echo -e "${red}--- CMake Failed ---${reset}"
        exit 1
    fi

    SECONDS=0
    echo -e "${yellow}<--- Start Ninja Build --->${reset}"
    ninja -C "$BUILD_DIR" -j$(($(nproc)-1)) -d explain
    if [ $? -ne 0 ]; then
        echo -e "${red}--- Ninja Build Failed ---${reset}"
        exit 1
    fi

    build_time=$SECONDS
    printf "${blue}<--- Build Time ---> %02d:%02d (mm:ss)\n${reset}" \
        $((build_time / 60)) $((build_time % 60))
}


if [ "$1" == "rebuild" ]; then
    echo -e "${yellow}<--- Rebuilding: Removing build directory --->${reset}"
    read -p "Are you sure? [y/N]: " confirm
    confirm=${confirm,,}
    if [[ "$confirm" != "y" && "$confirm" != "yes" ]]; then
        echo -e "${red}Rebuild cancelled.${reset}"
        exit 0
    fi
    # Carry an environment-provided point_msgs across the cache wipe. If it is
    # lost, CMake builds the bundled copy instead, and a second typesupport set
    # for the same type name keeps /auto_aim_plan out of the DDS graph.
    if [ -z "${point_msgs_DIR:-}" ] && [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
        cached_point_msgs=$(sed -n 's/^point_msgs_DIR:[^=]*=//p' "$BUILD_DIR/CMakeCache.txt" | head -n 1)
        case "$cached_point_msgs" in
            # Unset, or our own bundled copy: let CMake probe the environment again.
            ""|"$BUILD_DIR"/*) ;;
            *)
                if [ -f "$cached_point_msgs/point_msgsConfig.cmake" ]; then
                    export point_msgs_DIR="$cached_point_msgs"
                    echo -e "${yellow}Keeping external point_msgs: $point_msgs_DIR${reset}"
                fi
                ;;
        esac
    fi
    rm -rf "$BUILD_DIR"
    do_build
    exit 0
fi
if [ "$1" == "build" ]; then
    echo -e "${yellow}<--- building --->${reset}"
    do_build
    exit 0
fi

if [[ "$1" == "run" || "$1" == "debug" || "$1" == "race" ]]; then
    MODE="$1"
    shift
    if [ $# -lt 1 ]; then
        echo -e "${red}Please specify program to run.${reset}"
        exit 1
    fi

    if [[ "$MODE" != "race" ]]; then
        do_build
    fi

    RUN_PROGRAM="$BIN_DIR/$1"
    source "$WORK_DIR/script/setup_ros_runtime.sh"
    dts_setup_ros_runtime "$WORK_DIR" || exit 1
    shift
    ORIGINAL_ARGS=("$@")

    echo -e "${yellow}<--- Running dts_vision ($MODE) --->${reset}"

    if [[ "$MODE" == "debug" ]]; then
        echo -e "${yellow}Starting GDB with program: $RUN_PROGRAM ...${reset}"
        gdb --args "$RUN_PROGRAM" "${ORIGINAL_ARGS[@]}"
    else
        "$RUN_PROGRAM" "${ORIGINAL_ARGS[@]}"
        RET=$?

        if [ $RET -ne 0 ]; then
            echo -e "${red}--- Program crashed, running guard.sh ---${reset}"
            pkill "$(basename "$RUN_PROGRAM")"
            timeout=10
            while pgrep "$(basename "$RUN_PROGRAM")" > /dev/null; do
                sleep 0.5
                timeout=$((timeout - 1))
                if [ $timeout -le 0 ]; then
                    echo "$(basename "$RUN_PROGRAM") did not exit after 10 seconds, forcing kill"
                    pkill -9 "$(basename "$RUN_PROGRAM")"
                    break
                fi
            done

            GUARD_SCRIPT="$CONFIG_DIR/guard.sh"
            if [ ! -f "$GUARD_SCRIPT" ]; then
                echo -e "${red}guard.sh not found: $GUARD_SCRIPT${reset}"
                exit 1
            fi

            echo -e "${yellow}Starting guard.sh ...${reset}"
            exec "$GUARD_SCRIPT" "$RUN_PROGRAM" "${ORIGINAL_ARGS[@]}"
        fi
    fi
    exit 0
fi


echo -e "${yellow}Warning:${reset} Invalid argument '$1'."
echo -e "${yellow}Usage:${reset} $0 {build|rebuild|run <program> [args...]|debug <program> [args...]}"
exit 0
