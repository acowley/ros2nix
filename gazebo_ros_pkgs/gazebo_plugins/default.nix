{ catkin, cmake, gtest, pkgconfig, pyEnv, mkRosPythonPackage
, fetchurl, message_generation, dynamic_reconfigure
, roscpp, rospy, roslib, nodelet, angles, tf, tf2_ros, urdf, nav_msgs
, std_msgs, geometry_msgs, rosgraph_msgs, trajectory_msgs
, image_transport, rosconsole, cv_bridge
, polled_camera, diagnostic_updater 
, camera_info_manager, std_srvs
, gazebo_ros, gazebo_msgs, gazebo7, ogre
, tinyxml, protobuf, sdformat, ignition, freeimage, libxml2, tbb
}:
mkRosPythonPackage rec {
  name = "gazebo_plugins-${version}";
  version = "2.5.7";
  src = fetchurl {
    url = "https://github.com/ros-simulation/gazebo_ros_pkgs/archive/2.5.7.tar.gz";
    sha256 = "19xaq9kq1fdjf5fr1k8n8id82kxq2irsr2k0sk2wgmmlshlw2h3a";
  };
  unpackPhase = ''
    tar --strip-components=1 -xf ${src} gazebo_ros_pkgs-2.5.7/gazebo_plugins
    export sourceRoot="$PWD"/gazebo_plugins
  '';
  preConfigure = ''
    export SOURCE_DATE_EPOCH=315532800
  '';
  buildInputs = [
    gazebo_msgs
    gazebo_ros
    gazebo7
    ogre
    roslib
    roscpp
    nodelet
    message_generation
    dynamic_reconfigure
    tf
    tf2_ros
    urdf
    std_srvs
    std_msgs
    nav_msgs
    rosgraph_msgs
    geometry_msgs
    trajectory_msgs
    image_transport
    rosconsole
    cv_bridge
    polled_camera
    camera_info_manager
    diagnostic_updater
    tinyxml
    protobuf
    sdformat
    ignition.math2
    freeimage
    libxml2
    tbb
  ];
}
