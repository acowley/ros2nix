{ buildPythonPackage, fetchPypi, catkin_pkg, wstool, rosdistro }:
buildPythonPackage rec {
  pname = "rosinstall";
  version = "0.7.8";
  name = "${pname}-${version}";
  propagatedBuildInputs = [ catkin_pkg wstool rosdistro ];
  src = fetchPypi {
    inherit pname version;
    sha256 = "0h7d8ynv44c68sbfn28xw4k18k3ip6252x7r7bqw6b5cifzhia1b";
  };
}
