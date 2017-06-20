{ nixpkgs ? import <nixpkgs> {}, compiler ? "default" }:
with (import <nixpkgs> {});
haskell.lib.buildStackProject {
  name = "ros2nix";
  buildInputs = [ ncurses ];
  ghc = ghc;
}
