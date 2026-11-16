#!/usr/bin/env bash
# 一键安装 ROS 2，让本机 `import rclpy` 可用。
# Ubuntu 24.04 → Jazzy（ros-jazzy-ros-base + rclpy）
# Ubuntu 22.04 → Humble（ros-humble-ros-base + rclpy）
# 幂等：已经能导入 rclpy 时只补 ldconfig，不重复下载。
set -euo pipefail

cd "$(dirname "$0")/.."

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi
export DEBIAN_FRONTEND=noninteractive

# shellcheck disable=SC1091
. /etc/os-release
case "${VERSION_ID}" in
  24.04) DISTRO=jazzy ;;
  22.04) DISTRO=humble ;;
  *)
    echo "setup_ros2.sh 失败：只支持 Ubuntu 24.04（Jazzy）或 22.04（Humble），当前是 ${VERSION_ID:-未知}。" >&2
    exit 1
    ;;
esac

fail() {
  echo "setup_ros2.sh 失败：$1" >&2
  exit 1
}

retry() {
  local n=0
  until "$@"; do
    n=$((n + 1))
    if [[ "$n" -ge 4 ]]; then
      return 1
    fi
    echo "重试 ${n}/3：$*" >&2
    sleep $((n * 4))
  done
}

ldconfig_ros() {
  echo "/opt/ros/${DISTRO}/lib" | "${SUDO[@]}" tee "/etc/ld.so.conf.d/ros2-${DISTRO}.conf" >/dev/null
  "${SUDO[@]}" ldconfig
}

rclpy_imports() {
  [[ -f "/opt/ros/${DISTRO}/setup.bash" ]] || return 1
  local sys_py="/usr/bin/python3"
  [[ -x "${sys_py}" ]] || sys_py="python3"
  bash -c "set +u; source '/opt/ros/${DISTRO}/setup.bash'; '${sys_py}' -c 'import rclpy; from rosgraph_msgs.msg import Clock; from std_msgs.msg import String'"
}

venv_rclpy() {
  [[ -x .venv/bin/python ]] || return 0
  if .venv/bin/python -c 'from yopo_l0.ros2_env import ensure_ros_paths; ensure_ros_paths(); import rclpy'; then
    echo ".venv 可以 import rclpy。"
  else
    echo "警告：系统 python3 能导入 rclpy，但 .venv 不能。先 make venv；启动器会补 ROS 的模块路径。" >&2
  fi
}

if rclpy_imports; then
  ldconfig_ros
  echo "ROS 2 ${DISTRO} 已安装，rclpy 可导入。动态库已登记到 ldconfig。"
  venv_rclpy
  echo "运行：.venv/bin/python -m yopo_l0.ros2_run --scene scenes/open-3drone.json --seed 0 --policy hover"
  exit 0
fi

retry "${SUDO[@]}" apt-get update -y || fail "apt-get update 失败"
if ! command -v add-apt-repository >/dev/null 2>&1; then
  retry "${SUDO[@]}" apt-get install -y software-properties-common ca-certificates curl || fail "安装 curl / software-properties-common 失败"
fi
retry "${SUDO[@]}" add-apt-repository -y universe || fail "启用 universe 失败"
retry "${SUDO[@]}" apt-get install -y ca-certificates curl || fail "安装 curl 失败"

if [[ ! -f /etc/apt/sources.list.d/ros2.sources && ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  # 官方 ros-apt-source。GitHub API 不可用时退回 packages.ros.org 的 keyring 源。
  if ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p' | head -1)" \
    && [[ -n "${ROS_APT_SOURCE_VERSION}" ]]; then
    DEB="/tmp/ros2-apt-source.deb"
    COD="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
    URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${COD}_all.deb"
    retry curl -fL --retry 3 -o "${DEB}" "${URL}" || fail "下载 ${URL} 失败"
    "${SUDO[@]}" dpkg -i "${DEB}" || fail "安装 ros2-apt-source 失败"
  else
    echo "GitHub 上的 ros-apt-source 不可用，改用 packages.ros.org 的 keyring 源。" >&2
    retry "${SUDO[@]}" curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
      || fail "下载 ROS key 失败"
    ARCH="$(dpkg --print-architecture)"
    COD="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
    echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${COD} main" \
      | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi
fi

retry "${SUDO[@]}" apt-get update -y || fail "添加 ROS 源之后 apt-get update 失败"

PKGS=(
  "ros-${DISTRO}-ros-base"
  "ros-${DISTRO}-rclpy"
  "ros-${DISTRO}-std-msgs"
  "ros-${DISTRO}-rosgraph-msgs"
  "python3-yaml"  # rclpy 在系统 site-packages 里找 yaml；.venv 不带它
)
if [[ "${DISTRO}" == "humble" ]]; then
  PKGS+=("ros-humble-rmw-cyclonedds-cpp")
fi
if apt-cache show python3-rclpy >/dev/null 2>&1; then
  PKGS+=(python3-rclpy)
else
  echo "apt 里没有名为 python3-rclpy 的包。rclpy 模块由 ros-${DISTRO}-rclpy 提供，已列入安装列表。"
fi

retry "${SUDO[@]}" apt-get install -y \
  -o Dpkg::Options::=--force-confdef \
  -o Dpkg::Options::=--force-confold \
  "${PKGS[@]}" || fail "安装 ${PKGS[*]} 失败"

ldconfig_ros
rclpy_imports || fail "包已安装，但 source /opt/ros/${DISTRO}/setup.bash 之后 python3 仍不能 import rclpy"
venv_rclpy
echo "ROS 2 ${DISTRO} 安装完成（ros-${DISTRO}-ros-base + rclpy）。"
echo "动态库路径已写入 /etc/ld.so.conf.d/ros2-${DISTRO}.conf。"
echo "运行：.venv/bin/python -m yopo_l0.ros2_run --scene scenes/open-3drone.json --seed 0 --policy hover"
echo "需要自己起解释器时再：source /opt/ros/${DISTRO}/setup.bash"
