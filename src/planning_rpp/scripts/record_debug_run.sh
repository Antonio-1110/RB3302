#!/usr/bin/env bash
# Record everything needed to debug a navigation run.
#
# Usage (on the robot, with robot.launch.py and planning.launch.py already
# running):
#     bash ~/rb3302_final_proj_code/student_ws/src/planning_rpp/scripts/record_debug_run.sh [label]
#
# Then start go_letters in another terminal. When the run is over (a few
# seconds after the robot hugs a wall / stops), press Ctrl+C here ONCE and
# wait for "done". Everything is saved to ~/debug_runs/<label>_<time>/:
#     system.txt   clock, CPU/RAM, serial ports, udev rules, nodes/topics
#     top.txt      CPU usage per process, every second
#     bag/         rosbag of TF, odometry, IMU, scan, AMCL, plan, all
#                  velocity-chain stages, debug topics, diagnostics, /rosout
#     log_*/       the three most recent ROS launch log folders
#
# Copy to the Mac afterwards with:
#     rsync -avz ubuntu@192.168.50.13:~/debug_runs/ \
#       ~/Downloads/RB3302/robot_code_3302/debug_runs/


LABEL="${1:-run}"
RUN="$HOME/debug_runs/${LABEL}_$(date +%m%d_%H%M%S)"
mkdir -p "$RUN"

# ROS environment (harmless if already sourced).
source /opt/ros/jazzy/setup.bash
WS="$HOME/rb3302_final_proj_code/student_ws/install/setup.bash"
[ -f "$WS" ] && source "$WS"

echo "[debug] saving to $RUN"

{
    echo "=== date (robot clock)"; date
    echo; echo "=== timedatectl"; timedatectl
    echo; echo "=== uptime / load"; uptime
    echo; echo "=== CPU cores"; nproc
    echo; echo "=== memory"; free -h
    echo; echo "=== serial ports"
    ls -l /dev/rplidar /dev/waverover /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/ 2>&1
    echo; echo "=== udev rules"
    for f in /etc/udev/rules.d/*.rules; do echo "--- $f"; cat "$f"; done
    echo; echo "=== ROS nodes"; ros2 node list
    echo; echo "=== ROS topics"; ros2 topic list -t
} > "$RUN/system.txt" 2>&1

# CPU usage every second, in the background until the recording stops.
top -b -d 1 > "$RUN/top.txt" 2>&1 &
TOP_PID=$!

# Ctrl+C stops `ros2 bag record` (which saves the bag cleanly); this trap
# keeps the script alive afterwards so the cleanup below still runs.
trap 'echo; echo "[debug] stopping recording, please wait..."' INT

echo "[debug] recording. Start go_letters now; press Ctrl+C here ONCE when the run is over."
ros2 bag record -o "$RUN/bag" \
    /tf /tf_static \
    /scan \
    /odom /odom/raw /odom/raw_signed /imu_data_ros \
    /amcl_pose /initialpose \
    /plan /lookahead_point \
    /cmd_vel_nav_raw /cmd_vel_nav_smoothed /cmd_vel_safe /cmd_vel_repelled /cmd_vel \
    /collision_monitor_state \
    /turn_rate_controller/debug /wall_repulsion/debug \
    /diagnostics /rosout

trap - INT
kill "$TOP_PID" 2>/dev/null
wait "$TOP_PID" 2>/dev/null

# Launch logs (node output, warnings, errors) from the latest runs.
i=0
for d in $(ls -td "$HOME"/.ros/log/*/ 2>/dev/null | head -3); do
    i=$((i + 1))
    cp -r "$d" "$RUN/log_${i}_$(basename "$d")"
done

echo "[debug] done: $RUN"
du -sh "$RUN"
echo "[debug] on the Mac, run:"
echo "  rsync -avz ubuntu@192.168.50.13:~/debug_runs/ ~/Downloads/RB3302/robot_code_3302/debug_runs/"
