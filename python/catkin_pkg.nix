{ buildPythonPackage, fetchFromGitHub,
  mock, docutils, python-dateutil, pyparsing }:
buildPythonPackage rec {
  pname = "catkin_pkg";
  version = "0.3.9";
  name = "${pname}-${version}";
  buildInputs = [mock];
  propagatedBuildInputs = [docutils python-dateutil pyparsing];
  patchPhase = ''
    sed 's/argparse//' -i setup.py
  '';
  src = fetchFromGitHub {
    owner = "ros-infrastructure";
    repo = "catkin_pkg";
    rev = "0.3.9";
    sha256 = "12zzy30wsml2s7hl8f06ras8sdr4vpaa5i9iy3pc76462fwmhkvv";
  };
}
