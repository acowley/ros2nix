{ nixpkgs ? (import ./nixpkgs {}) }:
with nixpkgs;
let localPackages = rec {
      console-bridge = callPackage ./console-bridge.nix {};
      poco = callPackage ./poco.nix {};
      collada-dom = callPackage ./collada-dom.nix {};
      urdfdom-headers = callPackage ./urdfdom-headers.nix {};
      urdfdom = callPackage ./urdfdom.nix { inherit urdfdom-headers console-bridge; };
    };
  distroParams = {
    inherit (nixpkgs) boost opencv3 qt5;
    uuid = if nixpkgs ? uuid then nixpkgs.uuid else null;
    inherit (localPackages) console-bridge poco;
    inherit (darwin) libobjc;
    inherit (darwin.apple_sdk.frameworks) Cocoa;
  };
  perception = callPackage ./kinetic_perception.nix (distroParams // {
    extraPackages = {
      turtlesim = import ./turtlesim.nix;
    };
  } // callPackage ./ros-build-env.nix {} perception.packages);
  comm = callPackage ./kinetic_comm.nix (distroParams // {
    extraPackages = {
      turtlesim = import ./turtlesim.nix;
      inherit (perception.definitions) geometry_msgs;
    };
  } // callPackage ./ros-build-env.nix {} comm.packages);
in if lib.inNixShell then comm.shell else comm.packages
# in if lib.inNixShell then perception.shell else perception.packages
