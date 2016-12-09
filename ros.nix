{ nixpkgs ? (import ./nixpkgs {}) }:
with nixpkgs;
let localPackages = rec {
      console-bridge = import ./console-bridge.nix;
      poco = import ./poco.nix;
      collada-dom = import ./collada-dom.nix;
      urdfdom-headers = import ./urdfdom-headers.nix;
      urdfdom = import ./urdfdom.nix;
    };
  distroParams = {
    inherit (nixpkgs) boost opencv3 qt5;
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
  } // callPackage ./ros-build-env.nix {} perception.packages);
  comm = callPackage ./kinetic_comm.nix (distroParams // {
    extraPackages = {
      turtlesim = import ./turtlesim.nix;
      inherit (perception.definitions) geometry_msgs;
    };
  } // callPackage ./ros-build-env.nix {} comm.packages);
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
  } // callPackage ./ros-build-env.nix {} sim.packages);
in if lib.inNixShell then sim.shell else sim.packages
# in if lib.inNixShell then perception.shell else perception.packages
