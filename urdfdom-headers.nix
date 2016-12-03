{ stdenv, fetchFromGitHub, cmake }:
stdenv.mkDerivation {
  name = "urdfdom-headers";
  src = fetchFromGitHub {
    owner = "ros";
    repo = "urdfdom_headers";
    rev = "0.4.1";
    sha256 = "13qp5bbsjmgyq0r008x6arrv5npk4g2y2vgz4xw2jyf3b81cx7gk";
  };
  nativeBuildInputs = [ cmake ];
}
