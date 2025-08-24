#!/usr/bin/env bash
# start_session.sh — Fresh DriftBot laptop-side test session
#
# Kills any previous tmux session, then starts all needed terminals:
#   - bringup:  micro-ROS agent + SLAM + bag recorder
#   - teleop:   interactive teleop_twist_keyboard
#   - monitor:  live topic rate/value monitor
#   - manual:   spare shell for ad-hoc ros2 commands
#
# Usage:
#   ./scripts/start_session.sh           # create session and attach
#   ./scripts/start_session.sh --noattach # create session, do not attach

set -euo pipefail

ATTACH=1
START_RVIZ=0

for arg in "$@"; do
    case "${arg}" in
        --noattach|--no-attach) ATTACH=0 ;;
        --rviz)                 START_RVIZ=1 ;;
        --help|-h)
            echo "Usage: $0 [--noattach] [--rviz]"
            echo "  --noattach  create the session but do not attach"
            echo "  --rviz      also open an RViz2 window with the saved config"
            exit 0
            ;;
    esac
done

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="${PROJECT_ROOT}/bags"
SESSION_NAME="driftbot"

# ── Ensure bag directory exists ───────────────────────────────────────────────
mkdir -p "${BAG_DIR}"

# ── Kill any stale DriftBot tmux sessions ─────────────────────────────────────
for stale in $(tmux ls 2>/dev/null | grep -E '^driftbot' | cut -d: -f1); do
    echo "[start_session] Killing stale tmux session '${stale}'"
    tmux kill-session -t "${stale}" || true
done

# ── Environment ───────────────────────────────────────────────────────────────
# ROS setup files may reference optional environment variables; disable nounset
# temporarily while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "${PROJECT_ROOT}/microros_ws/install/local_setup.bash" ]]; then
    source "${PROJECT_ROOT}/microros_ws/install/local_setup.bash"
elif [[ -f "${PROJECT_ROOT}/microros_ws/install/setup.bash" ]]; then
    source "${PROJECT_ROOT}/microros_ws/install/setup.bash"
fi
if [[ -f "${PROJECT_ROOT}/ros2_ws/install/local_setup.bash" ]]; then
    source "${PROJECT_ROOT}/ros2_ws/install/local_setup.bash"
elif [[ -f "${PROJECT_ROOT}/ros2_ws/install/setup.bash" ]]; then
    source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

# ── Base environment string for each pane ─────────────────────────────────────
# Force bash in each pane; some setups use zsh as the default tmux shell, and
# ROS *.bash setup scripts rely on BASH_SOURCE.
ENV_SOURCE="unset AMENT_CURRENT_PREFIX; source /opt/ros/jazzy/setup.bash"
if [[ -f "${PROJECT_ROOT}/microros_ws/install/local_setup.bash" ]]; then
    ENV_SOURCE="${ENV_SOURCE}; source ${PROJECT_ROOT}/microros_ws/install/local_setup.bash"
fi
ENV_SOURCE="${ENV_SOURCE}; source ${PROJECT_ROOT}/ros2_ws/install/local_setup.bash"

# ── Create tmux session ───────────────────────────────────────────────────────
echo "[start_session] Creating fresh tmux session '${SESSION_NAME}'"
tmux new-session -d -s "${SESSION_NAME}" -n bringup \
    "bash -c 'cd ${PROJECT_ROOT} && ${ENV_SOURCE} && exec ros2 launch driftbot_bringup bringup.launch.py bag_dir:=${BAG_DIR}'"

tmux new-window -t "${SESSION_NAME}" -n teleop \
    "bash -c '${ENV_SOURCE} && exec ros2 run teleop_twist_keyboard teleop_twist_keyboard'"

tmux new-window -t "${SESSION_NAME}" -n monitor \
    "bash -c '${ENV_SOURCE} && exec ros2 run driftbot_bringup topic_monitor'"

tmux new-window -t "${SESSION_NAME}" -n manual \
    "bash -c '${ENV_SOURCE} && exec bash'"

if [[ ${START_RVIZ} -eq 1 ]]; then
    tmux new-window -t "${SESSION_NAME}" -n rviz \
        "bash -c '${ENV_SOURCE} && exec rviz2 -d ${PROJECT_ROOT}/ros2_ws/src/driftbot_bringup/config/driftbot.rviz'"
fi

# ── Print instructions ────────────────────────────────────────────────────────
cat <<EOF
[start_session] Session '${SESSION_NAME}' is running.

Windows:
  1:bringup  micro-ROS agent + SLAM + bag recorder
  2:teleop   teleop_twist_keyboard (use i/k/j/l/, keys)
  3:monitor  live topic monitor
  4:manual   spare shell
EOF

if [[ ${START_RVIZ} -eq 1 ]]; then
    echo "  5:rviz     RViz2 with LaserScan + Map + TF + Odometry"
fi

cat <<EOF
Attach later with: tmux attach -t ${SESSION_NAME}
Switch windows:    Ctrl-b + <window-number>
Stop everything:   ./scripts/stop_session.sh
EOF

if [[ ${ATTACH} -eq 1 ]]; then
    tmux attach -t "${SESSION_NAME}"
fi
