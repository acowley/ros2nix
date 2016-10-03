let sysPkgs = import <nixpkgs> {};
    pinnedPkgs = sysPkgs.fetchFromGitHub {
      owner = "NixOS";
      repo = "nixpkgs-channels";
      # 2016-10-01
      rev = "1bd4c08606475f2ed2ee76dbee7bc996e87e6804";
      sha256 = "1wfsay7csw2fx8qaxlpgnbggx4p5riaycfjsmijjl2cqmkraggr1";
    };
# in import pinnedPkgs
in (_: sysPkgs)
