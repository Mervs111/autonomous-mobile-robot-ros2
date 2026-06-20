#!/usr/bin/env bash
# run_amr_demo.sh
# ===============
# Bringup lengkap AMR demo dalam satu sesi tmux 4-pane.
#
# Layout:
#   +---------------------+---------------------+
#   |  T1 Sensor          |  T2 Localization    |
#   |  (kamera+lidar+odom)|  (RTAB-Map lock)    |
#   +---------------------+---------------------+
#   |  T4 Command         |  T3 Nav2 stack      |
#   |  (reset + goal)     |                     |
#   +---------------------+---------------------+
#
# Tiap pane di-env (DOMAIN 42 + Cyclone). T2 nunggu 8s biar T1 sensor siap;
# T3 nunggu 15s biar T2 lokalisasi siap.
#
# Usage:
#   bash run_amr_demo.sh             # bringup default (peta lab_demo_20jun.db)
#   bash run_amr_demo.sh peta.db     # bringup dengan peta lain
#   bash run_amr_demo.sh kill        # bunuh sesi tmux & semua node
#
# Setelah bringup, di pane T4 (kanan-bawah):
#   bash $(ros2 pkg prefix amr_slam)/lib/amr_slam/reset_odom.sh
#   ros2 param set /stm32_bridge autonomous_enabled true
#   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
#     "{pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

set -e
SESSION="amr"
MAP="${1:-$HOME/maps/lab_demo_20jun.db}"

if [ "$1" = "kill" ]; then
  echo "[demo] killing tmux session $SESSION + Nav2/RTAB nodes..."
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  pkill -f rtabmap || true
  pkill -f nav2 || true
  pkill -f rgbd_odometry || true
  pkill -f stm32_bridge || true
  echo "[demo] done."
  exit 0
fi

# cek dependencies
command -v tmux >/dev/null 2>&1 || { echo "[ERR] tmux belum terinstall. sudo apt install tmux"; exit 1; }
[ -f "$MAP" ] || { echo "[ERR] peta tidak ditemukan: $MAP"; exit 1; }

ENV='export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source /opt/ros/humble/setup.bash; cd ~/amr_starter && source install/setup.bash'

echo "[demo] kill sesi lama (kalau ada)..."
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "[demo] start tmux sesi '$SESSION'..."
tmux new-session -d -s "$SESSION" -x 220 -y 50

# Pane 0 = T1 sensor
tmux send-keys -t "$SESSION:0.0" \
  "$ENV; echo '=== T1 SENSOR ==='; ros2 launch amr_bringup amr_full.launch.py use_slam:=false use_nav2:=false use_rtabmap:=false use_vr:=false use_failover:=false" C-m

# Split horizontal -> pane 1 = T2 localization
tmux split-window -h -t "$SESSION:0.0"
tmux send-keys -t "$SESSION:0.1" \
  "$ENV; echo '=== T2 LOCALIZATION (tunggu 8s biar T1 sensor siap) ==='; sleep 8; ros2 launch amr_3d_mapping rtabmap_localization.launch.py database_path:=$MAP" C-m

# Split vertical pane 0 -> pane 2 = T4 command (kanan-bawah jadi command)
tmux split-window -v -t "$SESSION:0.0"
tmux send-keys -t "$SESSION:0.2" \
  "$ENV; echo '=== T4 COMMAND ==='; echo; echo 'Tunggu Nav2 active, lalu:'; echo '  bash \$(ros2 pkg prefix amr_slam)/lib/amr_slam/reset_odom.sh'; echo '  ros2 param set /stm32_bridge autonomous_enabled true'; echo '  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: \\\"base_link\\\"}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}\"'; echo" C-m

# Split vertical pane 1 -> pane 3 = T3 Nav2
tmux split-window -v -t "$SESSION:0.1"
tmux send-keys -t "$SESSION:0.3" \
  "$ENV; echo '=== T3 NAV2 (tunggu 15s biar T2 localization siap) ==='; sleep 15; ros2 launch amr_slam nav2.launch.py" C-m

# Fokus ke pane command
tmux select-pane -t "$SESSION:0.2"
echo "[demo] attaching to tmux..."
echo "[demo] keluar dari tmux: Ctrl-b lalu d  (detach, sesi tetap jalan)"
echo "[demo] bunuh semua: bash $0 kill"
sleep 1
tmux attach -t "$SESSION"
