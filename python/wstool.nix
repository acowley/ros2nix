{ buildPythonPackage, fetchPypi, vcstools }:
buildPythonPackage rec {
  pname = "wstool";
  version = "0.1.17";
  name = "${pname}-${version}";
  propagatedBuildInputs = [ vcstools ];
  src = fetchPypi {
    inherit pname version;
    sha256 = "0dz2gn2qx919s1z5wa94nkvb01pnqp945mvj97108w7i1q8lz6y7";
  };
}
