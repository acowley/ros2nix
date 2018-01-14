{ buildPythonPackage, fetchFromGitHub, trollius }:
buildPythonPackage rec {
  pname = "osrf-pycommon";
  version = "0.1.4";
  name = "${pname}-${version}";
  propagatedBuildInputs = [trollius];
  doCheck = false;
  src = fetchFromGitHub {
    owner = "osrf";
    repo = "osrf_pycommon";
    rev = "0.1.4";
    sha256 = "1qgyfgybqid9qbzk5q31xpp477dqzwq7j095gbw2dv4flifnvp3s";
  };
}
