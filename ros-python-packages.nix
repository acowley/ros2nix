{ stdenv, pkgs, fetchFromGitHub }:
let pip2nix = import (fetchFromGitHub {
      owner = "acowley";
      repo = "pip2nix";
      rev = "d2fc2136199ca33e669d8e1cc4d4a26a6c33db64";
      sha256 = "1pwq8yidyl28mdnxy1hxr0jgd82w9s0m3mfbw59pflbj0w8g9i94";
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
