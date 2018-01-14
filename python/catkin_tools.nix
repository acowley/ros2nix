{ buildPythonPackage, fetchFromGitHub, catkin_pkg, pyyaml, osrf-pycommon }:
buildPythonPackage rec {
  pname = "catkin_tools";
  version = "0.4.4";
  name = "${pname}-${version}";
  propagatedBuildInputs = [ catkin_pkg pyyaml osrf-pycommon ];
  doCheck = false;
  src = fetchFromGitHub {
    owner = "catkin";
    repo = "catkin_tools";
    rev = "0.4.4";
    sha256 = "0mfbxyl9f6r4sj9zcpgk9idf0jkl0n1vpsqcp8p55psp4gj1vvxr";
  };
}
