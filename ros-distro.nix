with (import <nixpkgs> {}).pkgs;
let
  rosDistributions = {
    indigo = "2014-07-22-indigo";
    jade = "2015-05-23-jade";
    kinetic = "2016-05-23-kinetic";
    lunar = "2017-05-23-lunar";
  };
  rosPython = python27.override {
    packageOverrides = self: super: {
      rosdep = self.callPackage ./python/rosdep.nix {};
      rosinstall_generator = self.callPackage ./python/rosinstall_generator.nix {};
      catkin_pkg = super.callPackage ./python/catkin_pkg.nix {};
      catkin_tools = self.callPackage ./python/catkin_tools.nix {};
      osrf-pycommon = super.callPackage ./python/osrf-pycommon.nix {};
      rospkg = super.callPackage ./python/rospkg.nix {};
      rosdistro = self.callPackage ./python/rosdistro.nix {};
      sip = super.callPackage ./sip.nix {};
    };
  };

  # Builds a rosinstall file for a particular variant of a given
  # distro and initializes a workspace for that variant.
  mkRosSrcDerivation = distro: variant:
    stdenv.mkDerivation {
      name = "ros-"+distro+"-"+variant+"-src";
      version = lib.getAttr distro rosDistributions;
      srcs = [];
      buildInputs = [rosPython.pkgs.rosinstall_generator cacert ];
      buildCommand = ''
        source $stdenv/setup
        mkdir -p $out
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
  rospy = rosLocalPythonPackages;
  inherit rosPython;
}
