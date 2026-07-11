#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: bash build.sh <version> [threads]"
    echo "Example: bash build.sh v0.3.2 4"
}

version=${1:-}
threads=${2:-1}
package_version=${version#v}

if [[ -z "${version}" ]]; then
    echo "No version provided."
    usage
    exit 1
fi

if [[ ! ${package_version} =~ ^[0-9]+(\.[0-9]+){1,3}([~+.-][0-9A-Za-z.+:~_-]+)?$ ]]; then
    echo "Invalid package version: ${version}"
    echo "Use a version such as v0.3.2 or 0.3.2."
    exit 1
fi

if [[ ! ${threads} =~ ^[1-9][0-9]*$ ]]; then
    echo "Invalid thread count: ${threads}"
    exit 1
fi

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ros_package_dir="${script_dir}"

if [[ ! -f "${ros_package_dir}/package.xml" || ! -f "${ros_package_dir}/CMakeLists.txt" ]]; then
    echo "ROS package metadata was not found in ${ros_package_dir}."
    exit 1
fi

# GitLab ROS images normally source their only ROS installation before invoking this script.
if [[ -z "${ROS_DISTRO:-}" ]]; then
    mapfile -t ros_installations < <(find /opt/ros -mindepth 1 -maxdepth 1 -type d 2>/dev/null | sort)
    if [[ ${#ros_installations[@]} -ne 1 ]]; then
        echo "ROS_DISTRO is unset and a unique ROS installation could not be selected."
        exit 1
    fi
    # shellcheck disable=SC1090
    source "${ros_installations[0]}/setup.bash"
fi

case "${ROS_DISTRO}" in
    noetic|melodic) ;;
    *)
        echo "Unsupported ROS distribution: ${ROS_DISTRO}"
        exit 1
        ;;
esac

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt install -y udx-ads-chcnav-msgs unity-drive-udi-msgs 


# Keep this list limited to dependencies used by the robot_loc ROS1 targets.
system_packages=(
    build-essential
    ca-certificates
    cmake
    curl
    libgeographic-dev
    libeigen3-dev
    libyaml-cpp-dev
)

if [[ "${ROS_DISTRO}" == "noetic" ]]; then
    system_packages+=(python3-catkin-pkg)
else
    system_packages+=(python-catkin-pkg)
fi

ros_packages=(
    "ros-${ROS_DISTRO}-angles"
    "ros-${ROS_DISTRO}-catkin"
    "ros-${ROS_DISTRO}-cmake-modules"
    "ros-${ROS_DISTRO}-diagnostic-msgs"
    "ros-${ROS_DISTRO}-diagnostic-updater"
    "ros-${ROS_DISTRO}-eigen-conversions"
    "ros-${ROS_DISTRO}-geographic-msgs"
    "ros-${ROS_DISTRO}-geometry-msgs"
    "ros-${ROS_DISTRO}-message-filters"
    "ros-${ROS_DISTRO}-message-generation"
    "ros-${ROS_DISTRO}-message-runtime"
    "ros-${ROS_DISTRO}-nav-msgs"
    "ros-${ROS_DISTRO}-nodelet"
    "ros-${ROS_DISTRO}-rosbag"
    "ros-${ROS_DISTRO}-roscpp"
    "ros-${ROS_DISTRO}-roslint"
    "ros-${ROS_DISTRO}-rostest"
    "ros-${ROS_DISTRO}-rosunit"
    "ros-${ROS_DISTRO}-sensor-msgs"
    "ros-${ROS_DISTRO}-std-msgs"
    "ros-${ROS_DISTRO}-std-srvs"
    "ros-${ROS_DISTRO}-tf2"
    "ros-${ROS_DISTRO}-tf2-geometry-msgs"
    "ros-${ROS_DISTRO}-tf2-ros"
)

apt-get install -y --no-install-recommends "${system_packages[@]}" "${ros_packages[@]}"

builder_url="https://oss.ud-x.com:30080/devops/builder/ros/build.sh"
echo "Building robot_loc ${package_version} for ROS ${ROS_DISTRO} with ${threads} thread(s)."
(
    cd "${ros_package_dir}"
    curl -fsSL "${builder_url}" | bash -s -- -o udi-zpmc -v "${package_version}" -j "${threads}"
)

shopt -s nullglob
artifacts=("${script_dir}"/*.deb "${script_dir}/../"*.deb)
if [[ ${#artifacts[@]} -eq 0 ]]; then
    echo "The ROS builder completed but no deb package was produced in ${script_dir} or ${script_dir}/../."
    exit 1
fi

for artifact in "${artifacts[@]}"; do
    artifact_dir=$(cd "$(dirname "${artifact}")" && pwd)
    output_dir=$(cd "${script_dir}/../" && pwd)
    if [[ "${artifact_dir}" != "${output_dir}" ]]; then
        mv -f "${artifact}" "${output_dir}/"
    fi
done

echo "Generated package(s):"
printf '  %s\n' "${artifacts[@]##*/}"
