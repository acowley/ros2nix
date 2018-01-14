{ buildPythonPackage, fetchPypi, nose, catkin_pkg, pyyaml, rosdistro, rospkg }:
buildPythonPackage rec {
  pname = "rosdep";
  version = "0.11.8";
  name = "${pname}-${version}";
  buildInputs = [nose];
  propagatedBuildInputs = [catkin_pkg pyyaml rosdistro rospkg];
  doCheck = false;
  src = fetchPypi {
    inherit pname version;
    sha256 = "1bl7p37bc0zznx5m5sxnzgfb5d8f3ghi3wy9f8318q7q2i0yx1c1";
  };
}
