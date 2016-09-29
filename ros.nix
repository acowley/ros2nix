{ nixpkgs ? (import <nixpkgs> {}) }:
with nixpkgs;
let localPackages = rec {
      console-bridge = callPackage ./console-bridge.nix {};
      poco = callPackage ./poco.nix {};
      collada-dom = callPackage ./collada-dom.nix {};
      urdfdom-headers = callPackage ./urdfdom-headers.nix {};
      urdfdom = callPackage ./urdfdom.nix { inherit urdfdom-headers console-bridge; };
    };
# in callPackage ./indigo_perception.nix {
# in callPackage ./indigo_core.nix {
in callPackage ./kinetic_perception.nix {
    inherit (nixpkgs) boost opencv3;
# libpng12-dev libtiff-dev libv4l-dev libvtk-qt protobuf-dev python-defusedxml;
    uuid = null;
    inherit (localPackages) console-bridge;
    inherit (localPackages) poco;
    inherit (darwin) libobjc;
    inherit (darwin.apple_sdk.frameworks) Cocoa;
   }
