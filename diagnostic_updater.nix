{ catkin, cmake, gtest, pkgconfig, pyEnv, mkRosPythonPackage
, fetchurl, roscpp, rostest, std_msgs, diagnostic_msgs
}:
mkRosPythonPackage rec {
  name = "diagnostic_updater-${version}";
  version = "1.8.10";
  src = fetchurl {
    url = "https://github.com/ros/diagnostics/archive/1.8.10.tar.gz";
    sha256 = "1sjd8nmvsrhr1wx25pykm6fg15ndkfy8vl52bdryfm8fyq960b4z";
  };
  unpackPhase = ''
    tar --strip-components=1 -xf ${src} diagnostics-1.8.10/diagnostic_updater
    export sourceRoot="$PWD"/diagnostic_updater
  '';
  preConfigure = ''
    export SOURCE_DATE_EPOCH=315532800
  '';
  propagatedBuildInputs = [ roscpp std_msgs diagnostic_msgs rostest ];
}
