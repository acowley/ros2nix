{ stdenv, pkgs, fetchFromGitHub }:
let pip2nix = import (fetchFromGitHub {
      owner = "acowley";
      repo = "pip2nix";
      rev = "388b76bddf2f9f968052bba480929d69453d55c7";
      sha256 = "0fms3bnr9h4rzzj77x24nd6hvi6gbksddfgiim8j9vj5hn78zfjz";
    }) {
       inherit (pkgs.pythonPackages) buildPythonApplication pip;
       inherit (pkgs) nix cacert;
       inherit stdenv;
    };
    pipDeps = [ "rosdep" "rosinstall_generator" "wstool" "rosinstall"
                "markerlib" "catkin_tools" "catkin_pkg" "bloom" ];
in stdenv.mkDerivation {
  name = "ros-python-packages";
  src = "";
  buildInputs = [pip2nix];
  phases = ["installPhase"];
  installPhase = ''
    mkdir -p $out
    echo "${stdenv.lib.concatStringsSep "\n" pipDeps}" | pip2nix > $out/ros-python-packages.nix
  '';
}
