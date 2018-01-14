{ buildPythonPackage, fetchPypi, pyyaml, rospkg, catkin_pkg }:
buildPythonPackage rec {
  pname = "rosdistro";
  version = "0.6.4";
  name = "${pname}-${version}";
  propagatedBuildInputs = [catkin_pkg pyyaml rospkg];
  src = fetchPypi {
    inherit pname version;
    sha256 = "1pj4vjlvmw9480qsy983al44yp53z9pi7pmnhmhqhmnm4kcibz5c";
  };
}
