{ nixpkgs ? import <nixpkgs> {}, compiler ? "default" }:

let

  inherit (nixpkgs) pkgs;

  f = { mkDerivation, stdenv, base, containers, hxt, text, hnix, directory, lens
      , process, yaml-light, yaml-light-lens, temporary, filepath, logging
      , optparse-applicative, intero, cabal-install }:
      mkDerivation {
        pname = "ros2nix";
        version = "0.1.0.0";
        src = ./.;
        isLibrary = false;
        isExecutable = true;
        executableHaskellDepends = [ base hxt containers text hnix process
                                     lens yaml-light yaml-light-lens
                                     temporary directory filepath logging
                                     optparse-applicative intero cabal-install ];
        description = "Build Nix definitions of a ROS distribution";
        license = stdenv.lib.licenses.bsd3;
      };

  haskellPackages = if compiler == "default"
                       then pkgs.haskell.packages.ghc801;
                       else pkgs.haskell.packages.${compiler};

  drv = haskellPackages.callPackage f {};

in

  if pkgs.lib.inNixShell then drv.env else drv
