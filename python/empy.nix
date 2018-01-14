{ buildPythonPackage, fetchurl }:
buildPythonPackage rec {
  pname = "EmPy";
  version = "3.3.3";
  name = "${pname}-${version}";
  src = fetchurl {
    url = http://www.alcyone.com/software/empy/empy-3.3.3.tar.gz;
    sha256 = "1mxfy5mgp473ga1pgz2nvm8ds6z4g3hdky8523z6jzvcs9ny6hcq";
  };
}
