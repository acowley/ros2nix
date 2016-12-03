{ catkin, cmake, gtest, pkgconfig, pyEnv, mkRosPythonPackage
, fetchurl, message_generation, dynamic_reconfigure
, roslib, roscpp, tf
, std_msgs, geometry_msgs, rosgraph_msgs, std_srvs
, gazebo_msgs, gazebo7, tinyxml, protobuf, sdformat, ignition
, freeimage, libxml2, tbb
}:
mkRosPythonPackage rec {
  name = "gazebo_ros-${version}";
  version = "2.5.7";
  src = fetchurl {
    url = "https://github.com/ros-simulation/gazebo_ros_pkgs/archive/2.5.7.tar.gz";
    sha256 = "19xaq9kq1fdjf5fr1k8n8id82kxq2irsr2k0sk2wgmmlshlw2h3a";
  };
  unpackPhase = ''
    tar --strip-components=1 -xf ${src} gazebo_ros_pkgs-2.5.7/gazebo_ros
    export sourceRoot="$PWD"/gazebo_ros
  '';
  preConfigure = ''
    export SOURCE_DATE_EPOCH=315532800
  '';
  propagatedBuildInputs = [
    gazebo_msgs
    gazebo7
    roslib
    roscpp
    tf
    std_srvs
    rosgraph_msgs
    dynamic_reconfigure
    std_msgs
    geometry_msgs
    message_generation
    tinyxml
    protobuf
    sdformat
    ignition.math2
    freeimage
    libxml2
    tbb
  ];
}
