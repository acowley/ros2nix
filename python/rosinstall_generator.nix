{ buildPythonApplication, fetchPypi, rosdistro }:
buildPythonApplication rec {
  pname = "rosinstall_generator";
  version = "0.1.13";
  name = "${pname}-${version}";
  propagatedBuildInputs = [ rosdistro ];
  patchPhase = ''
    sed "s/'argparse', //" -i setup.py
  '';
  src = fetchPypi {
    inherit pname version;
    sha256 = "16nyijzrq8y6xk2ph5niylc72ry8zs7swxf8qicblx6fnfi7wv05";
  };
}
