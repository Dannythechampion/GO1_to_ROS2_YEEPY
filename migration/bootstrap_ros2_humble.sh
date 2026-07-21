#!/usr/bin/env bash
set -euo pipefail

expected_ubuntu="22.04"
expected_codename="jammy"
expected_arch="arm64"
ros_distro="humble"

if [[ ! -r /etc/os-release ]]; then
  printf 'ERROR: /etc/os-release is unavailable.\n' >&2
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release
actual_arch="$(dpkg --print-architecture)"

if [[ "${VERSION_ID:-}" != "$expected_ubuntu" \
   || "${VERSION_CODENAME:-}" != "$expected_codename" \
   || "$actual_arch" != "$expected_arch" ]]; then
  printf 'ERROR: expected Ubuntu %s (%s) on %s.\n' \
    "$expected_ubuntu" "$expected_codename" "$expected_arch" >&2
  printf 'Detected VERSION_ID=%s VERSION_CODENAME=%s ARCH=%s\n' \
    "${VERSION_ID:-unknown}" "${VERSION_CODENAME:-unknown}" "$actual_arch" >&2
  exit 1
fi

printf 'Installing ROS 2 %s prerequisites on Ubuntu %s %s.\n' \
  "$ros_distro" "$VERSION_ID" "$actual_arch"

sudo apt-get update
sudo apt-get install -y \
  locales \
  software-properties-common \
  curl \
  ca-certificates \
  file \
  git \
  git-lfs

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y

ros_apt_source_version="$({
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p'
} | head -n 1)"

if [[ -z "$ros_apt_source_version" ]]; then
  printf 'ERROR: could not determine the latest ros2-apt-source release.\n' >&2
  exit 1
fi

ros_apt_source_deb="/tmp/ros2-apt-source.deb"
curl -fsSL -o "$ros_apt_source_deb" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${expected_codename}_all.deb"
sudo dpkg -i "$ros_apt_source_deb"

sudo apt-get update
sudo apt-get install -y \
  "ros-${ros_distro}-desktop" \
  "ros-${ros_distro}-rmw-cyclonedds-cpp" \
  "ros-${ros_distro}-navigation2" \
  "ros-${ros_distro}-nav2-bringup" \
  "ros-${ros_distro}-slam-toolbox" \
  "ros-${ros_distro}-pointcloud-to-laserscan" \
  "ros-${ros_distro}-tf2-tools" \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  python3-dev \
  pybind11-dev \
  build-essential \
  cmake \
  libboost-all-dev \
  libeigen3-dev \
  libmsgpack-dev \
  libpcl-dev \
  libyaml-cpp-dev

git lfs install

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

env_file="$HOME/go1_ros2_env.bash"
cat > "$env_file" <<EOF
#!/usr/bin/env bash
source /opt/ros/${ros_distro}/setup.bash
if [[ -r \"\$HOME/go1_ros2_ws/install/setup.bash\" ]]; then
  source \"\$HOME/go1_ros2_ws/install/setup.bash\"
fi
EOF
chmod +x "$env_file"

# shellcheck disable=SC1090
source "/opt/ros/${ros_distro}/setup.bash"
printf '\nROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
printf 'ros2=%s\n' "$(command -v ros2)"
printf 'colcon=%s\n' "$(command -v colcon)"
printf 'vcs=%s\n' "$(command -v vcs)"
printf 'Environment helper: %s\n' "$env_file"
printf 'Bootstrap completed. Reboot is not required by this script.\n'
