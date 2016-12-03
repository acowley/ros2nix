{ catkin, cmake, gtest, pkgconfig, pyEnv
, stdenv, fetchurl, message_generation, message_runtime
, std_msgs, geometry_msgs, sensor_msgs, trajectory_msgs
, std_srvs, mkRosCmakePackage
}:
mkRosCmakePackage rec {
  name = "gazebo_msgs-${version}";
  version = "2.5.7";
  src = fetchurl {
    url = "https://github.com/ros-simulation/gazebo_ros_pkgs/archive/2.5.7.tar.gz";
    sha256 = "19xaq9kq1fdjf5fr1k8n8id82kxq2irsr2k0sk2wgmmlshlw2h3a";
  };
  unpackPhase = ''
    tar --strip-components=1 -xf ${src} gazebo_ros_pkgs-2.5.7/gazebo_msgs
    export sourceRoot="$PWD"/gazebo_msgs
  '';
  configurePhase = ''
    cmakeConfigurePhase
  '';
  propagatedBuildInputs = [
    cmake
    pkgconfig
    gtest
    pyEnv
    catkin
    message_generation
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    std_srvs
  ];
}
