#!/bin/bash
OUT=/home/agi/x2_deploy_workspace/competition/inspect_walk.txt
mkdir -p /home/agi/x2_deploy_workspace/competition
: > "$OUT"

echo "===== pwd / ls =====" >> "$OUT"
pwd >> "$OUT"
ls -la >> "$OUT"

echo "===== important example files =====" >> "$OUT"

for f in \
/home/agi/x2_deploy_workspace/aimdk-aarch64-1bde262f-artifacts/src/py_examples/py_examples/mc_locomotion_velocity.py \
/home/agi/x2_deploy_workspace/aimdk-aarch64-1bde262f-artifacts/src/py_examples/py_examples/navigation.py \
/home/agi/x2_deploy_workspace/aimdk-aarch64-1bde262f-artifacts/src/py_examples/py_examples/motocontrol.py \
/home/agi/x2_deploy_workspace/aimdk-aarch64-1bde262f-artifacts/src/py_examples/py_examples/keyboard.py \
/home/agi/x2_deploy_workspace/aimdk-aarch64-1bde262f-artifacts/src/py_examples/py_examples/set_mc_input_source.py \
/home/agi/x2_deploy_workspace/aimdk-aarch64-1bde262f-artifacts/src/py_examples/py_examples/set_mc_action.py
do
  echo "" >> "$OUT"
  echo "========== $f ==========" >> "$OUT"
  if [ -f "$f" ]; then
    sed -n '1,260p' "$f" >> "$OUT"
  else
    echo "NOT FOUND" >> "$OUT"
  fi
done

echo "" >> "$OUT"
echo "===== ros2 interface show =====" >> "$OUT"

for i in \
aimdk_msgs/msg/McLocomotionVelocity \
aimdk_msgs/msg/McLocomotionSpeedMode \
aimdk_msgs/msg/McLocomotionSpeedStatus \
aimdk_msgs/srv/SetMcLocomotionSpeedMode \
aimdk_msgs/srv/GetMcLocomotionSpeedStatus \
aimdk_msgs/srv/SetMcInputSource \
aimdk_msgs/msg/McInputSource \
nav_msgs/msg/Odometry
do
  echo "" >> "$OUT"
  echo "========== $i ==========" >> "$OUT"
  ros2 interface show "$i" >> "$OUT" 2>&1
done

echo "" >> "$OUT"
echo "===== odom once =====" >> "$OUT"
timeout 3 ros2 topic echo /aima/hal/odom/state --once >> "$OUT" 2>&1

echo "DONE: $OUT"
