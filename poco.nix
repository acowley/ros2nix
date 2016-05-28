{ stdenv, fetchzip, cmake, sqlite, zlib, pcre, expat, openssl }:
stdenv.mkDerivation {
  name = "poco";
  src =
    let owner = "pocoproject";
        repo = "poco";
        rev = "1.7.2";
    in fetchzip {
      url = "https://github.com/${owner}/${repo}/archive/${repo}-${rev}-release.tar.gz";
      name = "${repo}-${rev}-src";
      sha256 = "1l2zv20ymsfc7901r4vkmq4zdin03j1qnddibdbmfi2006rgf0kq";
  } // { inherit rev; };
  nativeBuildInputs = [ cmake ];
  buildInputs = [ sqlite zlib pcre expat openssl ];
  cmakeFlags = [ "-DPOCO_UNBUNDLED=ON" ];
}
