{ nixpkgs ? import <nixpkgs> {}, compiler ? "default" }:

let

  inherit (nixpkgs) pkgs;

  f = { mkDerivation, stdenv, base, containers, hxt, text
      , lens, yaml-light, yaml-light-lens }:
      mkDerivation {
        pname = "ros2nix";
        version = "0.1.0.0";
        src = ./.;
        isLibrary = false;
        isExecutable = true;
        executableHaskellDepends = [ base hxt containers text
                                     lens yaml-light yaml-light-lens ];
        description = "Build Nix definitions of a ROS distribution";
        license = stdenv.lib.licenses.bsd3;
      };

  haskellPackages = if compiler == "default"
                       then pkgs.haskell.packages.lts-5_11
                       else pkgs.haskell.packages.${compiler};

  drv = haskellPackages.callPackage f {};

in

  if pkgs.lib.inNixShell then drv.env else drv
