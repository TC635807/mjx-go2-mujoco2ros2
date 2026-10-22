#!/usr/bin/env bash
# 【可选】给 dds 后端（unitree_sdk2py）准备 CycloneDDS。
#
# 结论先说（本机实测，详见 docs/backend-notes.md）:
#   * CycloneDDS 的 C 库能编出来；
#   * 但 cyclonedds 的 **Python 绑定在 Python 3.14 上 import 就崩**
#     （PEP 649 让它的 IDL 注解机制失效），unitree_sdk2py 因此用不了；
#   * 所以本仓库的 ROS 2 路径走的是 rclpy + rmw_cyclonedds_cpp，见 docs/ros2-setup.md。
#
# 下面这套只有在 Python 3.12/3.13 环境下才有意义。
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/.venv/bin/python"
PREFIX="$HOME/.local/cyclonedds"

# 1) CycloneDDS C 库（0.10.5，与 Python 绑定同版本）
mkdir -p "$ROOT/third_party" && cd "$ROOT/third_party"
[ -d cyclonedds ] || git clone --depth 1 --branch 0.10.5 https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds
"$ROOT/.venv/bin/cmake" -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" -DBUILD_IDLC=OFF -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF -DENABLE_SECURITY=OFF
"$ROOT/.venv/bin/cmake" --build build --target install -j"$(nproc)"

# 2) Python 绑定（注意：Python 3.14 下需要下面这行兼容补丁，且装了也用不了）
cd "$ROOT/third_party"
[ -d cyclonedds-python ] || git clone --depth 1 --branch 0.10.5 https://github.com/eclipse-cyclonedds/cyclonedds-python.git
sed -i 's/_Py_IsFinalizing()/Py_IsFinalizing()/g' cyclonedds-python/clayer/pysertype.c
# cyclonedds-python 的 find_cyclonedds() 要求 prefix 下同时存在 include/ 和 bin/
mkdir -p "$PREFIX/bin"
export CYCLONEDDS_HOME="$PREFIX" CMAKE_PREFIX_PATH="$PREFIX" LD_LIBRARY_PATH="$PREFIX/lib"
"$PY" -m pip install --no-build-isolation "$ROOT/third_party/cyclonedds-python"

# 3) unitree_sdk2py
[ -d unitree_sdk2_python ] || git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git
"$PY" -m pip install --no-build-isolation -e "$ROOT/third_party/unitree_sdk2_python"

echo "=== 做完上面这些，在 Python<=3.13 上可以用 --transport dds ==="
