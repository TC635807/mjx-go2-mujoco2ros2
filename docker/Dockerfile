# syntax=docker/dockerfile:1
#
# =============================================================================
# 宇树官方推荐栈的 Docker 环境（Go2 二次开发 / ROS 2 路径）
# =============================================================================
# 依据（官方文档）：
#   * support.unitree.com Quick_start（中文）:
#       推荐 Ubuntu 20.04 或 Ubuntu 22.04；不支持 Mac/Windows
#   * unitree_ros2 README:
#       tested  Ubuntu 20.04 + ROS 2 Foxy / Ubuntu 22.04 + ROS 2 Humble (recommend)
#       Docker 环境见其 .devcontainer/Dockerfile-humble
#   * unitree_ros2 README / Basic_services: 机器人固件 CycloneDDS 版本 = 0.10.2
#   * unitree_sdk2_python README / setup.py: Python >= 3.8, cyclonedds == 0.10.2
#   * unitree_sdk2 README: Ubuntu 20.04 / gcc 9.4 / cmake >= 3.10
#
# 本镜像 = Ubuntu 22.04 + ROS 2 Humble + CycloneDDS(0.10.x) + 官方 unitree_ros2
#          + 官方 unitree_sdk2(C++) + 官方 unitree_sdk2_python(cyclonedds 0.10.2)
#          + 本仓 mjx-go2-mujoco2ros2 的 python venv（numpy/mujoco/matplotlib）
# 与官方 .devcontainer/Dockerfile-humble 的差别（不影响兼容性）：
#   - 基座用官方 ROS 镜像 ros:humble-ros-base（同为 Ubuntu 22.04 + Humble），
#     不装 zsh / llvm / clang 等编辑器工具；
#   - boost 只装构建 SDK 实际用到的子集；
#   - 额外装 libosmesa6/GL/X11 运行库，让无显示器环境也能跑可视化测试。
# =============================================================================

FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- 构建期代理（可选；只在 build 时需要，不会写入镜像）---------------------
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG NO_PROXY

# --- 上游版本（可覆盖）-------------------------------------------------------
ARG UNITREE_ROS2_REF=v0.3.0
ARG UNITREE_SDK2_REF=2.0.2
ARG UNITREE_SDK2_PY_REF=814556d15970dd2ecf1c9984e845ca02ab07e206
ARG CYCLONEDDS_PY_VER=0.10.2

# --- 运行期路径 -------------------------------------------------------------
ARG UNITREE_ROS2_DIR=/opt/unitree_ros2
ARG UNITREE_SDK2_SRC=/opt/unitree_sdk2
ARG UNITREE_SDK2_PREFIX=/opt/unitree_robotics
ARG VENV=/opt/venv

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Shanghai \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    UNITREE_ROS2_DIR=${UNITREE_ROS2_DIR} \
    UNITREE_SDK2_PREFIX=${UNITREE_SDK2_PREFIX} \
    VENV=${VENV} \
    PYTHONUNBUFFERED=1 \
    LANG=C.UTF-8

# =============================================================================
# 1) 系统依赖 —— 官方 unitree_sdk2 依赖 + ROS 2 Humble 的 CycloneDDS 工具链
#    （Quick_start 的 apt 列表: cmake g++ build-essential libyaml-cpp-dev
#      libeigen3-dev libboost-all-dev libfmt-dev；这里只装构建 SDK 实际用到的
#      boost 子集，功能等价、镜像更小）
# =============================================================================
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential cmake git curl wget ca-certificates gnupg \
      libyaml-cpp-dev libeigen3-dev libfmt-dev \
      libboost-dev libboost-system-dev libboost-thread-dev \
      python3-pip python3-venv python3-dev python3-yaml \
      python3-colcon-common-extensions \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rosidl-generator-dds-idl \
      iproute2 iputils-ping net-tools less vim \
      libosmesa6 libgl1 libegl1 libglfw3 \
      libx11-6 libxext6 libxrandr2 libxi6 libxcursor1 libxinerama1 libxxf86vm1 \
 && rm -rf /var/lib/apt/lists/*

# =============================================================================
# 2) 官方 unitree_ros2 —— unitree_go / unitree_api / unitree_hg 消息包
#    与官方 CI（.github/workflows/build-humble.yml）一致：
#    Humble 不需要再编译 cyclonedds（apt 提供的已是 0.10.x），直接 colcon build。
# =============================================================================
RUN git clone --depth 1 --branch "${UNITREE_ROS2_REF}" \
        https://github.com/unitreerobotics/unitree_ros2 "${UNITREE_ROS2_DIR}" \
 && cd "${UNITREE_ROS2_DIR}/cyclonedds_ws" \
 && source /opt/ros/humble/setup.bash \
 && colcon build --packages-select unitree_go unitree_api unitree_hg \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf build log

# =============================================================================
# 3) 官方 unitree_sdk2（C++ SDK）—— 自带 x86_64/aarch64 预编译 CycloneDDS
#    安装到 /opt/unitree_robotics（官方文档示例路径）
# =============================================================================
RUN git clone --depth 1 --branch "${UNITREE_SDK2_REF}" \
        https://github.com/unitreerobotics/unitree_sdk2 "${UNITREE_SDK2_SRC}" \
 && cmake -S "${UNITREE_SDK2_SRC}" -B "${UNITREE_SDK2_SRC}/build" \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON \
      -DCMAKE_INSTALL_PREFIX="${UNITREE_SDK2_PREFIX}" \
 && cmake --build "${UNITREE_SDK2_SRC}/build" -j"$(nproc)" \
 && cmake --install "${UNITREE_SDK2_SRC}/build" \
 && mkdir -p "${UNITREE_SDK2_PREFIX}/bin" \
 && cp -a "${UNITREE_SDK2_SRC}/build/bin/." "${UNITREE_SDK2_PREFIX}/bin/" \
 && rm -rf "${UNITREE_SDK2_SRC}/build"

# =============================================================================
# 4) 官方 unitree_sdk2_python —— 依赖 cyclonedds == 0.10.2（= 机器人固件同版本）
#    用 PyPI 预编译 wheel（官方 README 认可的安装方式），wheel 自带 libddsc 0.10.2。
# =============================================================================
RUN python3 -m pip install --no-cache-dir "cyclonedds==${CYCLONEDDS_PY_VER}" \
 && git init -q /opt/unitree_sdk2_python \
 && cd /opt/unitree_sdk2_python \
 && git remote add origin https://github.com/unitreerobotics/unitree_sdk2_python \
 && git fetch -q --depth 1 origin "${UNITREE_SDK2_PY_REF}" \
 && git checkout -q FETCH_HEAD \
 && python3 -m pip install --no-cache-dir -e . \
 && python3 -c "from unitree_sdk2py.utils.crc import CRC; from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_; print('unitree_sdk2py OK, official CRC =', CRC())"

# =============================================================================
# 5) 本仓的 python venv（numpy + mujoco；--system-site-packages 让 rclpy /
#    unitree_sdk2py（装在系统 python）对 venv 可见）
# =============================================================================
RUN python3 -m venv --system-site-packages "${VENV}" \
 && "${VENV}/bin/pip" install --no-cache-dir --upgrade pip \
 && "${VENV}/bin/pip" install --no-cache-dir "numpy>=1.26" "mujoco>=3.2,<4" matplotlib \
 && "${VENV}/bin/python" -c "import numpy, mujoco; print('venv:', numpy.__version__, mujoco.__version__)"

# =============================================================================
# 6) 非 root 用户（与宿主 uid/gid 对齐，避免 /workspace 绑定挂载产生 root 文件）
# =============================================================================
ARG USER_NAME=unitree
ARG USER_UID=1000
ARG USER_GID=1000
RUN if ! getent group "${USER_GID}" >/dev/null; then groupadd -g "${USER_GID}" "${USER_NAME}"; fi \
 && if ! getent passwd "${USER_UID}" >/dev/null; then \
      useradd -m -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash "${USER_NAME}"; \
    fi

# =============================================================================
# 7) 环境脚本 / entrypoint
# =============================================================================
COPY setup_unitree_humble.sh /opt/setup_unitree_humble.sh
COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod 0755 /opt/setup_unitree_humble.sh /opt/entrypoint.sh

RUN USER_HOME="$(getent passwd "${USER_UID}" | cut -d: -f6)" \
 && echo 'source /opt/setup_unitree_humble.sh' >> "${USER_HOME}/.bashrc"

USER ${USER_UID}:${USER_GID}
WORKDIR /workspace
ENV BASH_ENV=/opt/setup_unitree_humble.sh
ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["bash"]
