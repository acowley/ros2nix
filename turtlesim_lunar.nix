{ catkin, cmake, gtest, pkgconfig, pyEnv
, stdenv, fetchurl, geometry_msgs, message_generation, message_runtime
, rosconsole, roscpp, roscpp_serialization, roslib, rostime, std_msgs, std_srvs
, mkRosCmakePackage
, qt5
}:
mkRosCmakePackage rec {
  name = "turtlesim";
  version = "0.8.0";
  src = fetchurl {
    url = "https://github.com/ros/ros_tutorials/archive/${version}.tar.gz";
    sha256 = "1x2h9psznfp414vkphijnki15h93ll6qf11pjfzdjbnbm3b4clni";
  };
  unpackPhase = ''
    tar --strip-components=1 -xf ${src} ros_tutorials-0.8.0/turtlesim
    export sourceRoot="$PWD"/turtlesim
  '';
  # See https://trac.macports.org/ticket/53201 for a discussion of why
  # we need to bump the minimum CMake version: without doing so, a
  # custom Clang breaks compiler identification and feature support
  # tests.
  preConfigure = ''
    sed 's/cmake_minimum_required(VERSION 2.8.3)/cmake_minimum_required(VERSION 3.0.0)/' -i ./CMakeLists.txt
  '' + stdenv.lib.optionalString stdenv.isDarwin ''
    export NIX_CFLAGS_COMPILE="$NIX_CFLAGS_COMPILE -F${qt5.qtbase}/lib"
  '';
  configurePhase = ''
    cmakeConfigurePhase
  '';
  cmakeFlags = ["-DCMAKE_OSX_DEPLOYMENT_TARGET=" "-DCMAKE_OSX_SYSROOT="];
  buildInputs = with qt5; [ qtbase makeQtWrapper qmakeHook ];
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
    rosconsole
    roscpp
    std_srvs
  ];
  postInstall = ''
    mkdir -p $out/bin
    makeQtWrapper $out/lib/turtlesim/turtlesim_node $out/bin/turtlesim_node
  '';
}
