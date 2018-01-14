let
ov = self: super: {
  boost = super.boost163;
  console-bridge = super.callPackage ./console-bridge.nix {};
  poco = super.callPackage ./poco.nix {};
  collada-dom = super.callPackage ./collada-dom.nix {};
  urdfdom-headers = super.callPackage ./urdfdom-headers.nix {};
  urdfdom = super.callPackage ./urdfdom.nix {};
  uuid = if super ? uuid then super.uuid else null;
  inherit (super.darwin) libobjc;
  inherit (super.darwin.apple_sdk.frameworks) Cocoa;
  protobuf = super.protobuf3_1;
  flann = super.callPackage ./flann.nix {};
  pythonPackages = self.rosPython.pkgs;
  python27Packages = self.rosPython.pkgs;
  opencv3 = super.opencv3.override { pythonPackages = self.rosPython.pkgs; };
  rosPython = super.python27.override {
    packageOverrides = self: super: {
      rosdep = self.callPackage ./python/rosdep.nix {};
      rosinstall_generator = self.callPackage ./python/rosinstall_generator.nix {};
      catkin_pkg = super.callPackage ./python/catkin_pkg.nix {};
      catkin_tools = self.callPackage ./python/catkin_tools.nix {};
      osrf-pycommon = super.callPackage ./python/osrf-pycommon.nix {};
      rospkg = super.callPackage ./python/rospkg.nix {};
      rosdistro = self.callPackage ./python/rosdistro.nix {};
      wstool = super.callPackage ./python/wstool.nix {};
      rosinstall = self.callPackage ./python/rosinstall.nix {};
      empy = super.callPackage ./python/empy.nix {};
      bloom = super.callPackage ./python/bloom.nix {};
      vcstools = super.callPackage ./python/vcstools.nix {};
      sip = super.callPackage ./sip.nix {};
    };
  };
};
nixpkgs = (import ((import <nixpkgs> { }).fetchFromGitHub {
  owner = "NixOS";
  repo = "nixpkgs";
  rev = "b17ec549a14a4b3fb8cdfff914afddd65fee942e";
  sha256 = "0x969dk2690bd7hvyn3yykws6mnn434a88l7z2xsn4dk9bvi5kp2";
}) {
    overlays = [ov];
  }); in
# nixpkgs = import <nixpkgs> { overlays = [ov]; }; in
with nixpkgs;
let localPackages = rec {
      console-bridge = import ./console-bridge.nix;
      poco = import ./poco.nix;
      collada-dom = import ./collada-dom.nix;
      urdfdom-headers = import ./urdfdom-headers.nix;
      urdfdom = import ./urdfdom.nix;
    };
  distroParams = {
    inherit (nixpkgs) opencv3 qt5;
    boost = nixpkgs.boost164;
    uuid = if nixpkgs ? uuid then nixpkgs.uuid else null;
    inherit (localPackages) console-bridge poco;
    inherit (darwin) libobjc;
    inherit (darwin.apple_sdk.frameworks) Cocoa;
    protobuf = nixpkgs.protobuf3_1;
  };
  perception = callPackage ./kinetic_perception.nix (distroParams // {
    extraPackages = {
      turtlesim = import ./turtlesim.nix;
    };
  } // callPackage ./ros-build-env.nix {} perception.packageSet);
  comm = callPackage ./kinetic_comm.nix (distroParams // {
    extraPackages = {
      turtlesim = import ./turtlesim.nix;
      inherit (perception.definitions) geometry_msgs;
    };
  } // callPackage ./ros-build-env.nix {} comm.packageSet);
  lunar_comm = callPackage ./lunar_comm.nix ({
    extraPackages = {
    # turtlesim = import ./turtlesim_lunar.nix;
    inherit (lunar_perception.definitions) geometry_msgs sensor_msgs tf2_msgs nav_msgs actionlib_msgs cv_bridge image_transport pluginlib class_loader tf2 tf2_ros actionlib tf2_py angles tf;
    };
  } // callPackage ./ros-build-env.nix {} lunar_comm.packageSet);
  lunar_perception = qt56.callPackage ./lunar_perception.nix ({
    extraPackages = {
      turtlesim = import ./turtlesim_lunar.nix;
    };
  } // callPackage ./ros-build-env.nix {} lunar_perception.packageSet);
  # The comm package set extended with Gazebo support
  sim = callPackage ./kinetic_sim.nix (distroParams // {
    extraPackages = {
      turtlesim = import ./turtlesim.nix;
      gazebo_msgs = import ./gazebo_ros_pkgs/gazebo_msgs/default.nix;
      gazebo_ros = import ./gazebo_ros_pkgs/gazebo_ros/default.nix;
      gazebo_plugins = import ./gazebo_ros_pkgs/gazebo_plugins/default.nix;
      diagnostic_updater = import ./diagnostic_updater.nix;
      diagnostic_aggregator = import ./diagnostic_aggregator.nix;
      rospy_tutorials = import ./rospy_tutorials.nix;
      urdf_parser_plugin = import ./robot_model/urdf_parser_plugin/default.nix;
      urdf = import ./robot_model/urdf/default.nix;
      inherit (perception.definitions) sensor_msgs geometry_msgs trajectory_msgs
        angles actionlib actionlib_msgs diagnostic_msgs nav_msgs
        tf tf2 tf2_msgs tf2_ros tf2_py
        rosbag_migration_rule dynamic_reconfigure
        cv_bridge polled_camera camera_info_manager image_transport
        camera_calibration_parsers pluginlib class_loader nodelet
        bond bondcpp bondpy smclib rosconsole_bridge xmlrpcpp;
    } // localPackages;
  } // callPackage ./ros-build-env.nix {} sim.packageSet);
in {
  inherit lunar_perception lunar_comm;
  kinetic_comm = comm;
  kinetic_perception = perception;
}
# in if lib.inNixShell then lunar_perception.shell else lunar_perception.packages
# in lunar_perception.packages
# in lunar_comm.packages.cmake_modules
# in lunar_comm.packages.catkin
# in (callPackage ./ros-build-env.nix {} {}).pyEnv.env
# in if lib.inNixShell then sim.shell else sim.packages
# in if lib.inNixShell then perception.shell else perception.packages
