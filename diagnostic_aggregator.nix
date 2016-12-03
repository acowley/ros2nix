{ catkin, cmake, gtest, pkgconfig, pyEnv, mkRosCmakePackage
, fetchurl, roscpp, rospy, rostest, diagnostic_msgs
, pluginlib, xmlrpcpp, bondcpp, bondpy
}:
mkRosCmakePackage rec {
  name = "diagnostic_aggregator-${version}";
  version = "1.8.10";
  src = fetchurl {
    url = "https://github.com/ros/diagnostics/archive/1.8.10.tar.gz";
    sha256 = "1sjd8nmvsrhr1wx25pykm6fg15ndkfy8vl52bdryfm8fyq960b4z";
  };
  unpackPhase = ''
    tar --strip-components=1 -xf ${src} diagnostics-1.8.10/diagnostic_aggregator
    export sourceRoot="$PWD"/diagnostic_aggregator
  '';
  buildInputs = [ rostest ];
  propagatedBuildInputs = [ roscpp rospy diagnostic_msgs pluginlib 
    diagnostic_msgs xmlrpcpp bondcpp bondpy ];
}
