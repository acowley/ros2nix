{ stdenv, fetchFromGitHub, cmake, pkgconfig, tinyxml, boost
, urdfdom-headers, console-bridge }:
stdenv.mkDerivation {
  name = "urdfdom";
  src = fetchFromGitHub {
    owner = "ros";
    repo = "urdfdom";
    rev = "0.4.1";
    sha256 = "0067x4gj6j0k1inhxvyffnnnlrsks2dzgmiql4cxj7jncm8agbyl";
  };
  nativeBuildInputs = [ cmake pkgconfig tinyxml urdfdom-headers console-bridge
                        boost ];
}
