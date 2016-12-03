{ catkin, cmake, gtest, pkgconfig, pyEnv
, stdenv, fetchurl, message_generation, message_runtime
, std_msgs, rospy, rostest
, mkRosCmakePackage
}:
mkRosCmakePackage rec {
  name = "rospy_tutorials";
  version = "0.7.1";
  src = fetchurl {
    url = "https://github.com/ros/ros_tutorials/archive/${version}.tar.gz";
    sha256 = "18iifdsjwbs5csg60rinkdlmbmhpbxsk9d7f7zp5v29bwcmdrxjm";
  };
  unpackPhase = ''
    tar --strip-components=1 -xf ${src} ros_tutorials-0.7.1/rospy_tutorials
    export sourceRoot="$PWD"/rospy_tutorials
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
    rospy
    rostest
  ];
}
