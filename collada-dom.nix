{ stdenv, fetchzip, cmake, pkgconfig
, boost, zlib, libxml2, minizip, uriparser, pcre }:
stdenv.mkDerivation {
  name = "collada-dom";
  version = "2.4.4";
  src =
    let owner = "rdiankov";
        repo = "collada-dom";
        rev = "2.4.4";
    in fetchzip {
      url = "https://github.com/${owner}/${repo}/archive/v${rev}.tar.gz";
      name = "${repo}-${rev}-src";
      sha256 = "19wx50y5ax9jn5s7g39b5s9my2x6vq99dv4jsx6zfy6c4c28mx8d";
  } // { inherit rev; };
  nativeBuildInputs = [ cmake pkgconfig boost ];
  buildInputs = [ zlib libxml2 minizip uriparser pcre ];
}
