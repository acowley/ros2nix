{ buildPythonPackage, fetchPypi, vcstools, rosdistro, rosdep, empy }:
buildPythonPackage rec {
  pname = "bloom";
  version = "0.6.2";
  name = "${pname}-${version}";
  propagatedBuildInputs = [ vcstools rosdistro rosdep empy ];
  doCheck = false;
  src = fetchPypi {
    inherit pname version;
    sha256 = "0snpah8zsr5bw00c8sv8dwf9y38iw9kxziv154k6m15fsryj0fxp";
  };
}
