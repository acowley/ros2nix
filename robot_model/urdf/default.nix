{ fetchurl, catkin, cmake, rostest, gtest, pkgconfig, roscpp, pluginlib
, rosconsole_bridge, mkRosCmakePackage
, urdfdom, urdfdom-headers, urdf_parser_plugin }:
mkRosCmakePackage rec {
  name = "urdf_parser_plugin-${version}";
  version = "1.12.5";
  src = fetchurl {
    url = "https://github.com/ros/robot_model/archive/${version}.tar.gz";
    sha256 = "1k0p0as2152cihp6xm304khsf2imymk6w3vspz4kivxmykmlqy60";
  };
  unpackPhase = ''
    tar --strip-components=1 -xf ${src} robot_model-1.12.5/urdf
    export sourceRoot="$PWD"/urdf
  '';
  configurePhase = ''
    cmakeConfigurePhase
  '';
  propagatedBuildInputs = [
    cmake
    pkgconfig
    gtest
    catkin
    urdfdom-headers
    roscpp
    rostest
    pluginlib
    rosconsole_bridge
    urdfdom
    urdfdom-headers
    urdf_parser_plugin
  ];
}
