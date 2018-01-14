{ stdenv, mkDerivation, sourceByRegex, base, containers, data-fix,
  directory, filepath, hnix, hxt, lens, logging, optparse-applicative,
  process, temporary, text, yaml-light, yaml-light-lens,
  haskell-ide-engine }:
mkDerivation {
  pname = "ros2nix";
  version = "0.1.0.0";
  src = sourceByRegex ./. [".*\.hs$" "src" "ros2nix\.cabal" "LICENSE"];
  isLibrary = false;
  isExecutable = true;

  executableHaskellDepends = [
    base containers data-fix directory filepath hnix hxt lens logging
    optparse-applicative process temporary text yaml-light
    yaml-light-lens # haskell-ide-engine
  ];
  homepage = "https://github.com/acowley/ros2nix#readme";
  license = stdenv.lib.licenses.bsd3;
}
