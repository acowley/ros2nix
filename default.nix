{ nixpkgs ? import <nixpkgs> {}, compiler ? "ghc822" }:
nixpkgs.pkgs.haskell.packages.${compiler}.callPackage ./ros2nix.nix {
  inherit (nixpkgs.stdenv.lib) sourceByRegex;
}
