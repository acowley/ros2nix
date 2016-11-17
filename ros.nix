{ nixpkgs ? (import ./nixpkgs {}) }:
with nixpkgs;
let localPackages = rec {
      console-bridge = callPackage ./console-bridge.nix {};
      poco = callPackage ./poco.nix {};
      collada-dom = callPackage ./collada-dom.nix {};
      urdfdom-headers = callPackage ./urdfdom-headers.nix {};
      urdfdom = callPackage ./urdfdom.nix { inherit urdfdom-headers console-bridge; };
    };
  # rosPackageSet = callPackage ./kinetic_comm.nix ({
  rosPackageSet = callPackage ./kinetic_perception.nix ({
    inherit (nixpkgs) boost opencv3 qt5;
    uuid = null;
    inherit (localPackages) console-bridge poco;
    inherit (darwin) libobjc;
    inherit (darwin.apple_sdk.frameworks) Cocoa;
    extraPackages = { turtlesim = import ./turtlesim.nix; };
  } // (callPackage ./ros-build-env.nix {} rosPackageSet.packages));
in if lib.inNixShell then rosPackageSet.shell else rosPackageSet.packages
