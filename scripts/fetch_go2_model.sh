#!/usr/bin/env bash
set -e
BASE=https://raw.githubusercontent.com/unitreerobotics/unitree_mujoco/main/unitree_robots/go2
DST=/home/tc63/mujoco/mjx-go2-mujoco2ros2/sim_env/models/go2
mkdir -p $DST/assets
for f in go2.xml scene.xml; do
  curl -sSfL -o $DST/$f $BASE/$f && echo "ok $f"
done
for f in base_0 base_1 base_2 base_3 base_4 hip_0 hip_1 thigh_0 thigh_1 thigh_mirror_0 thigh_mirror_1 calf_0 calf_1 calf_mirror_0 calf_mirror_1 foot; do
  curl -sSfL -o $DST/assets/$f.obj $BASE/assets/$f.obj && echo "ok $f.obj"
done
du -sh $DST
echo "=== MODEL DOWNLOAD DONE ==="
