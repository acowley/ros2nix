with (import <nixpkgs> {}).pkgs;
let
  basePythonPackages = python27Packages;
  sip = callPackage ./sip.nix {
      inherit (basePythonPackages) buildPythonPackage;
  };
  rosLocalPythonPackages = callPackage "${callPackage ./ros-python-packages.nix {}}/ros-python-packages.nix" {
    inherit (basePythonPackages) buildPythonPackage;
    inherit (pkgs) fetchurl;
    extradeps = { inherit (basePythonPackages) setuptools; };
  };
  rosPythonPackages = basePythonPackages // rosLocalPythonPackages;
  rosDistributions = {
    indigo = "2014-07-22-indigo";
    jade = "2015-05-23-jade";
    kinetic = "2016-05-23-kinetic";
    lunar = "2017-05-23-lunar";
  };
  srcPyEnv = python27.buildEnv.override {
    extraLibs = with rosPythonPackages; [
      setuptools rosdep rosinstall-generator wstool rosinstall
      catkin_tools catkin_pkg pyyaml rospkg rosdistro
    ];
  };

  # Builds a rosinstall file for a particular variant of a given
  # distro and initializes a workspace for that variant.
  mkRosSrcDerivation = distro: variant:
    stdenv.mkDerivation {
      name = "ros-"+distro+"-"+variant+"-src";
      version = lib.getAttr distro rosDistributions;
      srcs = [];
      buildInputs = [ srcPyEnv cacert which ];
      buildCommand = ''
        source $stdenv/setup
        mkdir -p $out
        which python
        export SSL_CERT_FILE="${cacert}/etc/ssl/certs/ca-bundle.crt"
        rosinstall_generator ${variant} --rosdistro ${distro} --deps --tar > $out/${distro}_${variant}.rosinstall
      '';
    };

  # See http://www.ros.org/reps/rep-0131.html#variants for a list of
  # ROS variants.
  mkDistro = distro:
    let mkSrcVariant = vname: {
          name = vname + "-src";
          value = mkRosSrcDerivation distro vname;
        };
        variants = [ "ros-core" "ros-base" "ros-full" "robot" "perception"
                     "simulators" "viz" "desktop" "desktop-full" "ros_comm" ];
    in builtins.listToAttrs (map mkSrcVariant variants);
in
{
  indigo = mkDistro "indigo";
  jade = mkDistro "jade";
  kinetic = mkDistro "kinetic";
  lunar = mkDistro "lunar";
}
