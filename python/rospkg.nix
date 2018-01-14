{ buildPythonPackage, fetchPypi, pyyaml }:
buildPythonPackage rec {
  pname = "rospkg";
  version = "1.1.4";
  name = "${pname}-${version}";
  propagatedBuildInputs = [pyyaml];
  src = fetchPypi {
    inherit pname version;
    sha256 = "0g2hjqia34l57zkyq2n4lzxhnjv7j63wcjv8m7d7rf35yq2xzvs2";
  };
}
