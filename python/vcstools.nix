{ buildPythonPackage, fetchPypi, python-dateutil, pyyaml }:
buildPythonPackage rec {
  pname = "vcstools";
  version = "0.1.40";
  name = "${pname}-${version}";
  propagatedBuildInputs = [python-dateutil pyyaml];
  doCheck = false;
  src = fetchPypi {
    inherit pname version;
    sha256 = "1mfasip71ky1g968n1zlramgn3fjxk4c922d0x9cs0nwm2snln4m";
  };
}
