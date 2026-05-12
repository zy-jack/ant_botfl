#!/usr/bin/env bash
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_ROOT="${NS3_ROOT:-${SCRIPT_ROOT}}"
ROS_ROOT="${ROS_ROOT:-${SCRIPT_ROOT}/ros}"
ROS_EXE_DIR="${ROS_EXE_DIR:-}"
NS3_TARGET="${NS3_TARGET:-scratch/sim/robot_fl_simulation}"
START_DELAY_SECONDS="${START_DELAY_SECONDS:-10}"

ROS_SCRIPTS=(
  s1.sh
  r1.sh
  r2.sh
  r3.sh
  r4.sh
  r5.sh
  r6.sh
  r7.sh
  r8.sh
  r9.sh
  r10.sh
)


print_usage() {
  cat <<EOF
Usage: ./run.sh [options]

Options:
  --ns3-root PATH       NS-3 project root. Default: ${SCRIPT_ROOT}
  --ros-root PATH       ROS project root. Default: ${SCRIPT_ROOT}/ros
  --ros-exe-dir PATH    Directory containing s1.sh and r1.sh...r10.sh. Default: ROS_ROOT/run/exe
  --ns3-target TARGET   NS-3 target passed to "./ns3 run". Default: scratch/sim/robot_fl_simulation
  --delay SECONDS       Delay between ROS terminal launches. Default: 10
  -h, --help            Show this help message.

Environment variables can also be used:
  NS3_ROOT=/path/to/ns3 ROS_ROOT=/path/to/ros ./run.sh
  ROS_EXE_DIR=/path/to/exe NS3_TARGET=scratch/sim/robot_fl_simulation START_DELAY_SECONDS=10 ./run.sh
EOF
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --ns3-root)
        if [[ $# -lt 2 ]]; then
          echo "Missing value for $1"
          exit 1
        fi
        NS3_ROOT="$2"
        shift 2
        ;;
      --ros-root)
        if [[ $# -lt 2 ]]; then
          echo "Missing value for $1"
          exit 1
        fi
        ROS_ROOT="$2"
        shift 2
        ;;
      --ros-exe-dir)
        if [[ $# -lt 2 ]]; then
          echo "Missing value for $1"
          exit 1
        fi
        ROS_EXE_DIR="$2"
        shift 2
        ;;
      --ns3-target)
        if [[ $# -lt 2 ]]; then
          echo "Missing value for $1"
          exit 1
        fi
        NS3_TARGET="$2"
        shift 2
        ;;
      --delay)
        if [[ $# -lt 2 ]]; then
          echo "Missing value for $1"
          exit 1
        fi
        START_DELAY_SECONDS="$2"
        shift 2
        ;;
      -h | --help)
        print_usage
        exit 0
        ;;
      *)
        echo "Unknown option: $1"
        print_usage
        exit 1
        ;;
    esac
  done

  if [[ -z "${ROS_EXE_DIR}" ]]; then
    ROS_EXE_DIR="${ROS_ROOT}/run/exe"
  fi
}

resolve_dir() {
  local path="$1"

  if [[ ! -d "${path}" ]]; then
    echo "Directory does not exist: ${path}"
    exit 1
  fi

  (cd "${path}" && pwd)
}

validate_config() {
  NS3_ROOT="$(resolve_dir "${NS3_ROOT}")"
  ROS_ROOT="$(resolve_dir "${ROS_ROOT}")"
  ROS_EXE_DIR="$(resolve_dir "${ROS_EXE_DIR}")"

  if [[ ! -x "${NS3_ROOT}/ns3" ]]; then
    echo "NS-3 launcher is missing or not executable: ${NS3_ROOT}/ns3"
    exit 1
  fi

  if ! [[ "${START_DELAY_SECONDS}" =~ ^[0-9]+$ ]]; then
    echo "Delay must be a non-negative integer: ${START_DELAY_SECONDS}"
    exit 1
  fi
}

find_terminal() {
  local terminal

  for terminal in xterm gnome-terminal konsole xfce4-terminal mate-terminal lxterminal terminator x-terminal-emulator; do
    if command -v "${terminal}" >/dev/null 2>&1; then
      printf '%s\n' "${terminal}"
      return 0
    fi
  done

  return 1
}

launch_ros_terminal() {
  local terminal="$1"
  local script="$2"
  local title="ros-${script%.sh}"
  local command

  printf -v command 'cd %q && %q; exec bash' "${ROS_ROOT}" "${ROS_EXE_DIR}/${script}"

  case "${terminal}" in
    gnome-terminal)
      "${terminal}" --title="${title}" -- bash -lc "${command}" &
      ;;
    konsole)
      "${terminal}" --new-tab -p "tabtitle=${title}" -e bash -lc "${command}" &
      ;;
    xfce4-terminal)
      "${terminal}" --title="${title}" -e "bash -lc '${command}'" &
      ;;
    mate-terminal)
      "${terminal}" --title="${title}" -- bash -lc "${command}" &
      ;;
    lxterminal)
      "${terminal}" -t "${title}" -e bash -lc "${command}" &
      ;;
    terminator)
      "${terminal}" -T "${title}" -x bash -lc "${command}" &
      ;;
    x-terminal-emulator)
      "${terminal}" -e bash -lc "${command}" &
      ;;
    *)
      "${terminal}" -T "${title}" -e bash -lc "${command}" &
      ;;
  esac
}

main() {
  local terminal
  local script

  parse_args "$@"
  validate_config

  clear

  echo "NS-3 root: ${NS3_ROOT}"
  echo "ROS root: ${ROS_ROOT}"
  echo "ROS executable scripts: ${ROS_EXE_DIR}"
  echo "NS-3 target: ${NS3_TARGET}"
  echo "ROS launch delay: ${START_DELAY_SECONDS}s"
  echo

  if ! terminal="$(find_terminal)"; then
    echo "No supported terminal emulator found. Please install xterm or another terminal emulator."
    exit 1
  fi

  for script in "${ROS_SCRIPTS[@]}"; do
    if [[ ! -x "${ROS_EXE_DIR}/${script}" ]]; then
      echo "Missing executable ROS script: ${ROS_EXE_DIR}/${script}"
      exit 1
    fi

    echo "Starting ${script} in ${terminal}..."
    launch_ros_terminal "${terminal}" "${script}"
    sleep "${START_DELAY_SECONDS}"
  done

  cd "${NS3_ROOT}"

  ./ns3 clean
  ./ns3 configure
  ./ns3
  ./ns3 run "${NS3_TARGET}"
}

main "$@"
