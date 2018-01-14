# { nixpkgs ? import <nixpkgs> {}, compiler ? "default" }:
# with (import <nixpkgs> {});
# haskell.lib.buildStackProject {
#   name = "ros2nix";
#   buildInputs = [ ncurses ];
#   ghc = ghc;
# }
{ compiler ? "ghc822" }:
let nixpkgs = import <nixpkgs> {};
in (import ./default.nix { inherit nixpkgs compiler; }).env
