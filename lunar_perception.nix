{ Cocoa, apr, atk, boost, bzip2, cmake, console-bridge, poco, eigen, extraPackages ? {}, fetchurl, gdk_pixbuf, glib, graphviz, gtest, gtk2, libobjc, libogg, libtheora, libyamlcpp, log4cxx, lz4, mkRosCmakePackage, mkRosPythonPackage, opencv3, pango, pcl, pkgconfig, rosShell, sbcl, stdenv, tinyxml2, ... }@deps:
let
    rosPackageSet = {
      actionlib = { catkin, cmake, gtest, message_generation, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "actionlib";
          version = "1.11.11-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/actionlib-release/archive/release/lunar/actionlib/1.11.11-0.tar.gz";
            sha256 = "0zga69c2bakizacbp2hlmisn363sa54ys53diwws2fql7i36las8";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
          ];
        };
      angles = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "angles";
          version = "1.9.11-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry_angles_utils-release/archive/release/lunar/angles/1.9.11-0.tar.gz";
            sha256 = "1sicy29x9y189mc459gjc3zm39mlplbn1lhwrivc41hz1k3fvpp8";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      bond = { catkin, cmake, gtest, message_generation, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "bond";
          version = "1.8.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/bond/1.8.1-0.tar.gz";
            sha256 = "18na0yv70pk35qxbkrn4kqj8jzs5mcvj1jf0vhvnhp1kd38kw5bg";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            std_msgs
          ];
        };
      bond_core = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "bond_core";
          version = "1.8.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/bond_core/1.8.1-0.tar.gz";
            sha256 = "18j0iaww4gzya73b4yyanzjk7qkfaj8g8bpdy1jl91y2cx9hm6vs";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      bondcpp = { catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "bondcpp";
          version = "1.8.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/bondcpp/1.8.1-0.tar.gz";
            sha256 = "0x4f2hmcp5436iyx6j76wgy26hcknyqzrgrx4f8zsjjrpqhb1m3k";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
          ];
          postPatch = ''
            sed -i -e 's/find_package(UUID REQUIRED)//' -e 's/''${UUID_LIBRARIES}//' ./CMakeLists.txt
          '';
        };
      bondpy = { bond, catkin, cmake, gtest, pkgconfig, pyEnv, rospy, smclib, stdenv }:
      mkRosPythonPackage {
          name = "bondpy";
          version = "1.8.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/bondpy/1.8.1-0.tar.gz";
            sha256 = "1jsg53g4v39rj568pdc3d03qjyi4k7ab2khhqpa27k29br1nli2h";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            bond
            catkin
            rospy
            smclib
          ];
        };
      smclib = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "smclib";
          version = "1.8.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/smclib/1.8.1-0.tar.gz";
            sha256 = "088d8wya5l8mh15hpllr0pnbi391xjwc8xy58cv65xs6pqb7ngap";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      catkin = { cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "catkin";
          version = "0.7.8-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/catkin-release/archive/release/lunar/catkin/0.7.8-0.tar.gz";
            sha256 = "1f832r7hc1nd1ss032m9fhpa61vg0nbkwhy78x230ah47gvysmr0";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
          ];
          patchPhase = ''
            sed -i 's|#!@PYTHON_EXECUTABLE@|#!${pyEnv.python.passthru.interpreter}|' ./cmake/templates/_setup_util.py.in
            sed -i 's/PYTHON_EXECUTABLE/SHELL/' ./cmake/catkin_package_xml.cmake
            sed -i 's|#!/usr/bin/env bash|#!${stdenv.shell}|' ./cmake/templates/setup.bash.in
            sed -i 's|#!/usr/bin/env sh|#!${stdenv.shell}|' ./cmake/templates/setup.sh.in
          '';
        };
      class_loader = { catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, stdenv, console-bridge, poco, boost }:
      mkRosCmakePackage {
          name = "class_loader";
          version = "0.3.8-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/class_loader-release/archive/release/lunar/class_loader/0.3.8-0.tar.gz";
            sha256 = "0w629q0qv75379jidvwzqvdc2k8z6h5479ma0pkk3s0qcf9ci8k3";
          };
          propagatedBuildInputs = [
            cmake
            boost
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            console-bridge
            poco
          ];
        };
      cmake_modules = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "cmake_modules";
          version = "0.4.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/cmake_modules-release/archive/release/lunar/cmake_modules/0.4.1-0.tar.gz";
            sha256 = "093k2zpkmbipg7qbaw73rqya7zqahmiwicyiqv23xwg74wy4386k";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      actionlib_msgs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "actionlib_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/actionlib_msgs/1.12.5-0.tar.gz";
            sha256 = "0msr8wg5kkx1y76l4mabis5b6068wvaxpc07pm1xws3nqv37yhlb";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
            std_msgs
          ];
        };
      common_msgs = { actionlib_msgs, catkin, cmake, diagnostic_msgs, geometry_msgs, gtest, nav_msgs, pkgconfig, pyEnv, sensor_msgs, shape_msgs, stdenv, stereo_msgs, trajectory_msgs, visualization_msgs }:
      mkRosCmakePackage {
          name = "common_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/common_msgs/1.12.5-0.tar.gz";
            sha256 = "0ig3ajnn423avypldlvhljaq71rzb5j9aamq2zyah4cavbv680pp";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            actionlib_msgs
            catkin
            diagnostic_msgs
            geometry_msgs
            nav_msgs
            sensor_msgs
            shape_msgs
            stereo_msgs
            trajectory_msgs
            visualization_msgs
          ];
        };
      diagnostic_msgs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "diagnostic_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/diagnostic_msgs/1.12.5-0.tar.gz";
            sha256 = "04fv44xzfbwnd2ykv81vz35k4z0pij2dvqx2gbzvggpjjzjxpqvm";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
            std_msgs
          ];
        };
      geometry_msgs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "geometry_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/geometry_msgs/1.12.5-0.tar.gz";
            sha256 = "1fygpdhkmdhvnw886ajpcxhd3439cfz1kab0izifwbpjs8j58ckm";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
            std_msgs
          ];
        };
      nav_msgs = { actionlib_msgs, catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "nav_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/nav_msgs/1.12.5-0.tar.gz";
            sha256 = "09pwbb2s5w6cg7qxhph0madqqbwgcnrvjarljp3sdhssigrrdzyl";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            actionlib_msgs
            catkin
            geometry_msgs
            message_generation
            message_runtime
            std_msgs
          ];
        };
      sensor_msgs = { catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
  # mkRosPythonPackage {
  mkRosCmakePackage {
          name = "sensor_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/sensor_msgs/1.12.5-0.tar.gz";
            sha256 = "0m25z1g2rnqrbzadhfgm5l9nrpn3gy0x5c8cdnq51qck7x2zs3n8";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            geometry_msgs
            message_generation
            message_runtime
            std_msgs
          ];
        };
      shape_msgs = { catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "shape_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/shape_msgs/1.12.5-0.tar.gz";
            sha256 = "1585367k9chcfkmil4cznz3xnqqavark2myjdv2mb771pg169wjk";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            geometry_msgs
            message_generation
            message_runtime
            std_msgs
          ];
        };
      stereo_msgs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, sensor_msgs, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "stereo_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/stereo_msgs/1.12.5-0.tar.gz";
            sha256 = "1p84q623crb6mjjm7xi95g1hrd9z62vna8zvls9yp9hqahsqrfdx";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
            sensor_msgs
            std_msgs
          ];
        };
      trajectory_msgs = { catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, rosbag_migration_rule, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "trajectory_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/trajectory_msgs/1.12.5-0.tar.gz";
            sha256 = "097a4kb88qby0rbiv8v1fnya4n9scj4hqb783mwgpaxy37538kf7";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            geometry_msgs
            message_generation
            message_runtime
            rosbag_migration_rule
            std_msgs
          ];
        };
      visualization_msgs = { catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "visualization_msgs";
          version = "1.12.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/common_msgs-release/archive/release/lunar/visualization_msgs/1.12.5-0.tar.gz";
            sha256 = "0h5kvw1jzjysjp0210yimxls4k6ibh13p4amczc69rz2wyrsgcb2";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            geometry_msgs
            message_generation
            message_runtime
            std_msgs
          ];
        };
      dynamic_reconfigure = { catkin, cmake, gtest, message_generation, pkgconfig, pyEnv, roscpp_serialization, rostest, stdenv }:
      mkRosPythonPackage {
          name = "dynamic_reconfigure";
          version = "1.5.49-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/dynamic_reconfigure-release/archive/release/lunar/dynamic_reconfigure/1.5.49-0.tar.gz";
            sha256 = "0ddj4f2q07dn1r3d78h6a4gyjzdfdz4xnyjmg4cnnj5pn3wvkpm8";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            roscpp_serialization
            rostest
          ];
        };
      filters = { catkin, cmake, gtest, pkgconfig, pluginlib, pyEnv, rosconsole, roscpp, roslib, rostest, stdenv }:
      mkRosCmakePackage {
          name = "filters";
          version = "1.8.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/filters-release/archive/release/lunar/filters/1.8.1-0.tar.gz";
            sha256 = "0v01iqxpin77w8yzfg40q4lhyhkpm9vsv88jga4jx762l9mfshbi";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            pluginlib
            rosconsole
            roscpp
            roslib
            rostest
          ];
        };
      gencpp = { catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "gencpp";
          version = "0.5.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/gencpp-release/archive/release/lunar/gencpp/0.5.5-0.tar.gz";
            sha256 = "1xfdsssr3idivf26jr1ldlf9lqf8yyrzhaid7hjrap41lg92xw3a";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genmsg
          ];
          patchPhase = ''
            sed -i 's/''${PYTHON_EXECUTABLE} //' ./cmake/gencpp-extras.cmake.em
          '';
        };
      geneus = { catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "geneus";
          version = "2.2.6-0";
          src = fetchurl {
            url = "https://github.com/tork-a/geneus-release/archive/release/lunar/geneus/2.2.6-0.tar.gz";
            sha256 = "121rdhnmx3gps5phmg9kax22lc2964ivpkxfp4g6n0g2f3jljjqm";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genmsg
          ];
          preConfigure = ''
            sed -i 's/COMMAND ''${CATKIN_ENV} ''${PYTHON_EXECUTABLE}/COMMAND ''${CATKIN_ENV}/' ./cmake/geneus-extras.cmake.em
          '';
        };
      genlisp = { catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "genlisp";
          version = "0.4.16-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/genlisp-release/archive/release/lunar/genlisp/0.4.16-0.tar.gz";
            sha256 = "18kk0vnb1k55ainizr2xzv96x8hx1hhmbmh9vij9y1iywfs3kpzw";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genmsg
          ];
          patchPhase = ''
            sed -i 's/''${PYTHON_EXECUTABLE} //' ./cmake/genlisp-extras.cmake.em
          '';
        };
      genmsg = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "genmsg";
          version = "0.5.9-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/genmsg-release/archive/release/lunar/genmsg/0.5.9-0.tar.gz";
            sha256 = "1gx672j642lhnadrcpnln714k7jh0vrpkwyhqv5v877b9vxdgynm";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
          patchPhase = ''
            sed -i 's/''${PYTHON_EXECUTABLE} ''${GENMSG_CHECK_DEPS_SCRIPT}/''${GENMSG_CHECK_DEPS_SCRIPT}/' ./cmake/pkg-genmsg.cmake.em
          '';
        };
      gennodejs = { catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "gennodejs";
          version = "2.0.1-0";
          src = fetchurl {
            url = "https://github.com/RethinkRobotics-release/gennodejs-release/archive/release/lunar/gennodejs/2.0.1-0.tar.gz";
            sha256 = "19cp2fsy7wymhm77q84ah398zpnvl2mszh9p2am94xwvj9d5z8b6";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genmsg
          ];
        };
      genpy = { catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "genpy";
          version = "0.6.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/genpy-release/archive/release/lunar/genpy/0.6.7-0.tar.gz";
            sha256 = "09kbagavc7j40sz6v0jifa5irb5axaqwdnr0qqfr7jww07f9yxv6";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genmsg
          ];
          patchPhase = ''
            sed -i 's/''${PYTHON_EXECUTABLE} //' ./cmake/genpy-extras.cmake.em
          '';
        };
      eigen_conversions = { catkin, cmake, cmake_modules, eigen, geometry_msgs, gtest, orocos_kdl, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "eigen_conversions";
          version = "1.11.9-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry-release/archive/release/lunar/eigen_conversions/1.11.9-0.tar.gz";
            sha256 = "0rp5j4b100gch3ms8laqcw2dpn0lx6jfzjvvkhv8pnq10vi4ipsl";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            eigen
            geometry_msgs
            orocos_kdl
            std_msgs
          ];
        };
      tf = { angles, catkin, cmake, geometry_msgs, graphviz, gtest, message_filters, message_generation, message_runtime, pkgconfig, pyEnv, rosconsole, roscpp, rostime, roswtf, sensor_msgs, std_msgs, stdenv, tf2_ros }:
      mkRosPythonPackage {
          name = "tf";
          version = "1.11.9-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry-release/archive/release/lunar/tf/1.11.9-0.tar.gz";
            sha256 = "05mp293ndgvc91n1gmv4mi17j72z95q7l11xskj4cqniqpgls3im";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            angles
            catkin
            geometry_msgs
            graphviz
            message_filters
            message_generation
            message_runtime
            rosconsole
            roscpp
            rostime
            roswtf
            sensor_msgs
            std_msgs
            tf2_ros
          ];
        };
      tf2 = { catkin, cmake, console-bridge, geometry_msgs, gtest, pkgconfig, pyEnv, rostime, stdenv, tf2_msgs }:
      mkRosCmakePackage {
          name = "tf2";
          version = "2-0.5.17-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2/0.5.17-0.tar.gz";
            sha256 = "0bzic0kcmpiqw2wdc9avb5ib5gm7c788g8p5ri4gmld58iyqhfg8";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            geometry_msgs
            console-bridge
            rostime
            tf2_msgs
          ];
        };
      tf2_eigen = { catkin, cmake, cmake_modules, eigen, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "tf2_eigen";
          version = "0.5.17-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_eigen/0.5.17-0.tar.gz";
            sha256 = "1njddwy8619w5v2w0m864gjhxzinc1wc4h81gfqm8mij1byhxjqz";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            eigen
          ];
        };
      tf2_geometry_msgs = { catkin, cmake, gtest, pkgconfig, pyEnv, python_orocos_kdl, stdenv }:
      mkRosPythonPackage {
          name = "tf2_geometry_msgs";
          version = "0.5.17-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_geometry_msgs/0.5.17-0.tar.gz";
            sha256 = "030fcsjf1xzxdh5fqjyf958jn81fppf9df6w6f78m6xmkcmr6v6z";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            python_orocos_kdl
          ];
        };
      tf2_msgs = { actionlib_msgs, catkin, cmake, geometry_msgs, gtest, message_generation, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "tf2_msgs";
          version = "0.5.17-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_msgs/0.5.17-0.tar.gz";
            sha256 = "11hy2s3zx846li7yznz9rxcc6q2j6yvc3lmc917ih6bjfwxj8bia";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            actionlib_msgs
            catkin
            geometry_msgs
            message_generation
          ];
        };
      tf2_py = { catkin, cmake, gtest, pkgconfig, pyEnv, rospy, stdenv, tf2 }:
      mkRosPythonPackage {
          name = "tf2_py";
          version = "0.5.17-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_py/0.5.17-0.tar.gz";
            sha256 = "0gw6gjjw56bd9p9fqjvmcncyiqir61l7fv6lspda69n510gjw234";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            rospy
            tf2
          ];
        };
      tf2_ros = { actionlib, actionlib_msgs, catkin, cmake, geometry_msgs, gtest, message_filters, pkgconfig, pyEnv, roscpp, rosgraph, rospy, std_msgs, stdenv, tf2, tf2_msgs, tf2_py, xmlrpcpp }:
      mkRosPythonPackage {
          name = "tf2_ros";
          version = "0.5.17-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_ros/0.5.17-0.tar.gz";
            sha256 = "0lhpmzg7kpnnrn8h1220jl43jqc0pcadqqb2dgvw4jzgz086a5cs";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            actionlib
            actionlib_msgs
            catkin
            geometry_msgs
            message_filters
            roscpp
            rosgraph
            rospy
            std_msgs
            tf2
            tf2_msgs
            tf2_py
            xmlrpcpp
          ];
        };
      camera_calibration_parsers = { boost, catkin, cmake, gtest, libyamlcpp, pkgconfig, pyEnv, rosconsole, roscpp, roscpp_serialization, sensor_msgs, stdenv }:
      mkRosPythonPackage {
          name = "camera_calibration_parsers";
          version = "1.11.13-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/camera_calibration_parsers/1.11.13-0.tar.gz";
            sha256 = "0vglpgjrnkl3nc64rckb9k08laa481m0rwnp7dabxfv220v2wz0k";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            pkgconfig
            rosconsole
            roscpp
            roscpp_serialization
            sensor_msgs
            libyamlcpp
          ];
        };
      camera_info_manager = { boost, camera_calibration_parsers, catkin, cmake, gtest, image_transport, pkgconfig, pyEnv, roscpp, roslib, rostest, sensor_msgs, stdenv }:
      mkRosCmakePackage {
          name = "camera_info_manager";
          version = "1.11.13-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/camera_info_manager/1.11.13-0.tar.gz";
            sha256 = "1yirkbmxblxqnjjc7l0sz800caj7rkywvbnkx7cmhc2nxlza7gkn";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            camera_calibration_parsers
            catkin
            image_transport
            roscpp
            roslib
            rostest
            sensor_msgs
          ];
        };
      image_common = { camera_calibration_parsers, camera_info_manager, catkin, cmake, gtest, image_transport, pkgconfig, polled_camera, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "image_common";
          version = "1.11.13-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/image_common/1.11.13-0.tar.gz";
            sha256 = "1gja2xhsd1ymbg3p5n94qg9bdryxc6bkp1w37cx3xzdhmxw631gv";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            camera_calibration_parsers
            camera_info_manager
            catkin
            image_transport
            polled_camera
          ];
        };
      image_transport = { catkin, cmake, gtest, message_filters, pkgconfig, pluginlib, pyEnv, rosconsole, roscpp, roslib, sensor_msgs, stdenv }:
      mkRosCmakePackage {
          name = "image_transport";
          version = "1.11.13-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/image_transport/1.11.13-0.tar.gz";
            sha256 = "0r102lx9ldyqmgg0k25iflidy5bakjklscx53xlikqvcq4gm13jf";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_filters
            pluginlib
            rosconsole
            roscpp
            roslib
            sensor_msgs
          ];
        };
      polled_camera = { catkin, cmake, gtest, image_transport, message_generation, message_runtime, pkgconfig, pyEnv, roscpp, sensor_msgs, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "polled_camera";
          version = "1.11.13-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/polled_camera/1.11.13-0.tar.gz";
            sha256 = "1603zbdicsgjr5xmjs8rn455bbvazjf3mcbic49c0c25443xk9ax";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            image_transport
            message_generation
            message_runtime
            roscpp
            sensor_msgs
            std_msgs
          ];
        };
      camera_calibration = { catkin, cmake, cv_bridge, gtest, image_geometry, message_filters, pkgconfig, pyEnv, rospy, sensor_msgs, std_srvs, stdenv }:
      mkRosPythonPackage {
          name = "camera_calibration";
          version = "1.12.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/camera_calibration/1.12.21-0.tar.gz";
            sha256 = "1rkigd3kwsd7vvah3ivwhh2cdy8sj6ksgpch7j1dh47d939sc4hz";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cv_bridge
            image_geometry
            message_filters
            rospy
            sensor_msgs
            std_srvs
          ];
        };
      depth_image_proc = { boost, catkin, cmake, cmake_modules, cv_bridge, eigen_conversions, gtest, image_geometry, image_transport, message_filters, nodelet, pkgconfig, pyEnv, sensor_msgs, stdenv, stereo_msgs, tf2, tf2_ros }:
      mkRosCmakePackage {
          name = "depth_image_proc";
          version = "1.12.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/depth_image_proc/1.12.21-0.tar.gz";
            sha256 = "152z7ad5zzrj1brsfq4jf9wndkcgcp9498lyj1qm7pdknay93mpl";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            cmake_modules
            cv_bridge
            eigen_conversions
            image_geometry
            image_transport
            message_filters
            nodelet
            sensor_msgs
            stereo_msgs
            tf2
            tf2_ros
          ];
        };
      image_pipeline = { camera_calibration, catkin, cmake, depth_image_proc, gtest, image_proc, image_publisher, image_rotate, image_view, pkgconfig, pyEnv, stdenv, stereo_image_proc }:
      mkRosCmakePackage {
          name = "image_pipeline";
          version = "1.12.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_pipeline/1.12.21-0.tar.gz";
            sha256 = "1lj4cxs7xmclkmmacjz5z8vmaa5cilrh8kx8v72gmxhfvw72r013";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            camera_calibration
            catkin
            depth_image_proc
            image_proc
            image_publisher
            image_rotate
            image_view
            stereo_image_proc
          ];
        };
      image_proc = { boost, catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_geometry, image_transport, nodelet, nodelet_topic_tools, pkgconfig, pyEnv, roscpp, sensor_msgs, stdenv }:
      mkRosCmakePackage {
          name = "image_proc";
          version = "1.12.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_proc/1.12.21-0.tar.gz";
            sha256 = "17n1ybjfkma8sy99gl7q26cplv3p14syr77hii2alw48wmw5gnxa";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            cv_bridge
            dynamic_reconfigure
            image_geometry
            image_transport
            nodelet
            nodelet_topic_tools
            roscpp
            sensor_msgs
          ];
        };
      image_publisher = { camera_info_manager, catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_transport, nodelet, pkgconfig, pyEnv, roscpp, sensor_msgs, stdenv }:
      mkRosCmakePackage {
          name = "image_publisher";
          version = "1.12.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_publisher/1.12.21-0.tar.gz";
            sha256 = "0f9a01nlqr47bwlyzrwgk30yx6zwx47y9j28r0s0myrlh6sd7c03";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            camera_info_manager
            catkin
            cv_bridge
            dynamic_reconfigure
            image_transport
            nodelet
            roscpp
            sensor_msgs
          ];
        };
      image_rotate = { catkin, cmake, cmake_modules, cv_bridge, dynamic_reconfigure, geometry_msgs, gtest, image_transport, nodelet, pkgconfig, pyEnv, roscpp, stdenv, tf2, tf2_geometry_msgs, tf2_ros }:
      mkRosCmakePackage {
          name = "image_rotate";
          version = "1.12.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_rotate/1.12.21-0.tar.gz";
            sha256 = "1c5lz3phay4330phpgvkfkap9ak1kxb9qijx4apppcla6pip9fz4";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            cv_bridge
            dynamic_reconfigure
            geometry_msgs
            image_transport
            nodelet
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
          ];
        };
      image_view = { camera_calibration_parsers, catkin, cmake, cv_bridge, dynamic_reconfigure, glib, gtest, gtk2, image_transport, message_filters, message_generation, nodelet, pango, pkgconfig, pyEnv, rosconsole, roscpp, sensor_msgs, std_srvs, stdenv, stereo_msgs }:
      mkRosCmakePackage {
          name = "image_view";
          version = "1.12.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_view/1.12.21-0.tar.gz";
            sha256 = "1r6jvv68259fgydm2sy2z2970naywvbpfkshzhdbic527yyb85j3";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            camera_calibration_parsers
            catkin
            cv_bridge
            dynamic_reconfigure
            gtk2
            image_transport
            message_filters
            message_generation
            nodelet
            rosconsole
            roscpp
            sensor_msgs
            std_srvs
            stereo_msgs
            glib
            pango
            gtk2.out
            gtk2.dev
            glib.out
            glib.dev
            pango
          ];
          NIX_CFLAGS_COMPILE = "-I${glib.out}/lib/glib-2.0/include -I${gtk2.dev}/include/gtk-2.0 -I${glib.dev}/include/glib-2.0 -I${pango.dev}/include/pango-1.0 -I${gtk2.out}/lib/gtk-2.0/include";
        };
      stereo_image_proc = { catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_geometry, image_proc, image_transport, message_filters, nodelet, pkgconfig, pyEnv, sensor_msgs, stdenv, stereo_msgs }:
      mkRosCmakePackage {
          name = "stereo_image_proc";
          version = "1.12.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/stereo_image_proc/1.12.21-0.tar.gz";
            sha256 = "1wbrdyvpnkp07f2rsk48fsbkb92vajn34xl6ixad63sslfxrzz0m";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cv_bridge
            dynamic_reconfigure
            image_geometry
            image_proc
            image_transport
            message_filters
            nodelet
            sensor_msgs
            stereo_msgs
          ];
        };
      compressed_depth_image_transport = { catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_transport, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "compressed_depth_image_transport";
          version = "1.9.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_transport_plugins-release/archive/release/lunar/compressed_depth_image_transport/1.9.5-0.tar.gz";
            sha256 = "0rjcnxk8hnayfccq1gd0jfwc8lv6flwlxmdp6860bp53r844n6hq";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cv_bridge
            dynamic_reconfigure
            image_transport
          ];
        };
      compressed_image_transport = { catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_transport, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "compressed_image_transport";
          version = "1.9.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_transport_plugins-release/archive/release/lunar/compressed_image_transport/1.9.5-0.tar.gz";
            sha256 = "0z4l81y2n2hiv1gqkslxxxpkvfwjpxcv0hisjh3d7a0grzmrpgf7";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cv_bridge
            dynamic_reconfigure
            image_transport
          ];
        };
      image_transport_plugins = { catkin, cmake, compressed_depth_image_transport, compressed_image_transport, gtest, pkgconfig, pyEnv, stdenv, theora_image_transport }:
      mkRosCmakePackage {
          name = "image_transport_plugins";
          version = "1.9.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_transport_plugins-release/archive/release/lunar/image_transport_plugins/1.9.5-0.tar.gz";
            sha256 = "0wzfyj460sc69g7jfvrcmxg5kkfmn92daq79y41rw1jyzxr468mg";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            compressed_depth_image_transport
            compressed_image_transport
            theora_image_transport
          ];
        };
      theora_image_transport = { catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_transport, libogg, libtheora, message_generation, message_runtime, pkgconfig, pluginlib, pyEnv, rosbag, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "theora_image_transport";
          version = "1.9.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_transport_plugins-release/archive/release/lunar/theora_image_transport/1.9.5-0.tar.gz";
            sha256 = "060h5689zif6ly26h2lgl2ik6ml6mddz95li1rv6kikyjaksrsjz";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cv_bridge
            dynamic_reconfigure
            image_transport
            libogg
            libtheora
            message_generation
            message_runtime
            pluginlib
            rosbag
            std_msgs
          ];
        };
      laser_assembler = { catkin, cmake, filters, gtest, laser_geometry, message_filters, message_generation, message_runtime, pkgconfig, pluginlib, pyEnv, roscpp, rostest, sensor_msgs, stdenv, tf }:
      mkRosCmakePackage {
          name = "laser_assembler";
          version = "1.7.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/laser_assembler-release/archive/release/lunar/laser_assembler/1.7.4-0.tar.gz";
            sha256 = "0c09pyplbdjqhim4bnid5z6vh4ihfrsvbpjamfxda9m9qqj3kalc";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            filters
            laser_geometry
            message_filters
            message_generation
            message_runtime
            pluginlib
            roscpp
            rostest
            sensor_msgs
            tf
          ];
        };
      laser_filters = { angles, catkin, cmake, filters, gtest, laser_geometry, message_filters, pkgconfig, pluginlib, pyEnv, roscpp, rostest, sensor_msgs, stdenv, tf }:
      mkRosCmakePackage {
          name = "laser_filters";
          version = "1.8.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/laser_filters-release/archive/release/lunar/laser_filters/1.8.5-0.tar.gz";
            sha256 = "1rx66fm2kr4azmmb5f8s7nkhz2cr4r7q6zmqkjiwb37hkl11vifk";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            angles
            catkin
            filters
            laser_geometry
            message_filters
            pluginlib
            roscpp
            rostest
            sensor_msgs
            tf
          ];
        };
      laser_geometry = { angles, boost, catkin, cmake, cmake_modules, eigen, gtest, pkgconfig, pyEnv, roscpp, sensor_msgs, stdenv, tf }:
      mkRosPythonPackage {
          name = "laser_geometry";
          version = "1.6.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/laser_geometry-release/archive/release/lunar/laser_geometry/1.6.4-0.tar.gz";
            sha256 = "0b003jvm4zkn8p9bibg5ya9d46q69wlygzv7k5yxha2x1sljzc7s";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            angles
            boost
            catkin
            cmake_modules
            eigen
            roscpp
            sensor_msgs
            tf
          ];
        };
      laser_pipeline = { catkin, cmake, gtest, laser_assembler, laser_filters, laser_geometry, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "laser_pipeline";
          version = "1.6.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/laser_pipeline-release/archive/release/lunar/laser_pipeline/1.6.2-0.tar.gz";
            sha256 = "08n2qqkiiw2wrj53xwcvngp8gidyz95m6m3rz58fsyhpscgxgrgq";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            laser_assembler
            laser_filters
            laser_geometry
          ];
        };
      message_generation = { catkin, cmake, gencpp, geneus, genlisp, genmsg, gennodejs, genpy, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "message_generation";
          version = "0.4.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/message_generation-release/archive/release/lunar/message_generation/0.4.0-0.tar.gz";
            sha256 = "1y6w0i9ld0wj1kksrnbjg8n57r69jj9zj78lymsq55abzhqmbpqh";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            gencpp
            geneus
            genlisp
            genmsg
            gennodejs
            genpy
          ];
        };
      message_runtime = { catkin, cmake, cpp_common, genpy, gtest, pkgconfig, pyEnv, roscpp_serialization, roscpp_traits, rostime, stdenv }:
      mkRosCmakePackage {
          name = "message_runtime";
          version = "0.4.12-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/message_runtime-release/archive/release/lunar/message_runtime/0.4.12-0.tar.gz";
            sha256 = "0qqpl8f6pk6dnfs5gahwfh7vvaq2np4j04fgr4pa1xdlgvrgyvnx";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cpp_common
            genpy
            roscpp_serialization
            roscpp_traits
            rostime
          ];
        };
      perception = { catkin, cmake, gtest, image_common, image_pipeline, image_transport_plugins, laser_pipeline, perception_pcl, pkgconfig, pyEnv, ros_base, stdenv, vision_opencv }:
      mkRosCmakePackage {
          name = "perception";
          version = "1.3.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/metapackages-release/archive/release/lunar/perception/1.3.1-0.tar.gz";
            sha256 = "0nx26x7h51s1j16llhbhns7s5hcnxykyp8fb4p91il5fjvl6hka8";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            image_common
            image_pipeline
            image_transport_plugins
            laser_pipeline
            perception_pcl
            ros_base
            vision_opencv
          ];
        };
      ros_base = { actionlib, bond_core, catkin, class_loader, cmake, dynamic_reconfigure, gtest, nodelet_core, pkgconfig, pluginlib, pyEnv, ros_core, stdenv }:
      mkRosCmakePackage {
          name = "ros_base";
          version = "1.3.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/metapackages-release/archive/release/lunar/ros_base/1.3.1-0.tar.gz";
            sha256 = "0pkkxgf70h2ydp807wlg0s75akd5rrhxwyrdc18fhc8q93j0g2ig";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            actionlib
            bond_core
            catkin
            class_loader
            dynamic_reconfigure
            nodelet_core
            pluginlib
            ros_core
          ];
        };
      ros_core = { catkin, cmake, cmake_modules, common_msgs, gencpp, geneus, genlisp, genmsg, gennodejs, genpy, gtest, message_generation, message_runtime, pkgconfig, pyEnv, ros, ros_comm, rosbag_migration_rule, rosconsole_bridge, roscpp_core, rosgraph_msgs, roslisp, rospack, std_msgs, std_srvs, stdenv }:
      mkRosCmakePackage {
          name = "ros_core";
          version = "1.3.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/metapackages-release/archive/release/lunar/ros_core/1.3.1-0.tar.gz";
            sha256 = "0vdasx56hqfg15kbrr46h4bkmhrzxp0v6lh5zgaid0h4pjs1szas";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            common_msgs
            gencpp
            geneus
            genlisp
            genmsg
            gennodejs
            genpy
            message_generation
            message_runtime
            ros
            ros_comm
            rosbag_migration_rule
            rosconsole_bridge
            roscpp_core
            rosgraph_msgs
            roslisp
            rospack
            std_msgs
            std_srvs
          ];
        };
      nodelet = { catkin, cmake, cmake_modules, gtest, message_generation, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "nodelet";
          version = "1.9.14-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/lunar/nodelet/1.9.14-0.tar.gz";
            sha256 = "08fihgwa8r61g89vjw38a4rh27h783i7wv4cn3xaz9gcygjg0crf";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            message_generation
          ];
          postPatch = ''
            sed -i -e 's/find_package(UUID REQUIRED)//' -e 's/ ''${UUID_INCLUDE_DIRS}//' -e 's/ ''${UUID_LIBRARIES}//g' ./CMakeLists.txt
          '';
        };
      nodelet_core = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "nodelet_core";
          version = "1.9.14-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/lunar/nodelet_core/1.9.14-0.tar.gz";
            sha256 = "0yb7891lmz5ph9j5ih35ws5vzgnpdiqv8qlz3vb6sb0l8ir7mxz3";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      nodelet_topic_tools = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "nodelet_topic_tools";
          version = "1.9.14-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/lunar/nodelet_topic_tools/1.9.14-0.tar.gz";
            sha256 = "15rbgkifyjn72fy7avr4fpmkxrz7l8a9v36grpawq6yr742ka0jy";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      orocos_kdl = { catkin, cmake, eigen, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "orocos_kdl";
          version = "1.3.1-0";
          src = fetchurl {
            url = "https://github.com/smits/orocos-kdl-release/archive/release/lunar/orocos_kdl/1.3.1-0.tar.gz";
            sha256 = "0cjim80k50xb57zx8fc70pg5ab43b294rd8kjcafr15l4i6zxhgi";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            eigen
            pkgconfig
          ];
          preConfigure = ''
            sed -i 's|FIND_PATH(EIGEN3_INCLUDE_DIR Eigen/Core |FIND_PATH(EIGEN3_INCLUDE_DIR Eigen/Core ${eigen}/include/eigen3 |' ./config/FindEigen3.cmake
          '';
        };
      python_orocos_kdl = { catkin, cmake, gtest, orocos_kdl, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "python_orocos_kdl";
          version = "1.3.1-0";
          src = fetchurl {
            url = "https://github.com/smits/orocos-kdl-release/archive/release/lunar/python_orocos_kdl/1.3.1-0.tar.gz";
            sha256 = "08l4n50a5brk4n4k85v1h7p11qyilsig8a8fnyzk9jblg4m39zph";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            orocos_kdl
          ];
        };
      pcl_conversions = { catkin, cmake, cmake_modules, gtest, pcl, pcl_msgs, pkgconfig, pyEnv, roscpp, sensor_msgs, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "pcl_conversions";
          version = "0.2.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/pcl_conversions-release/archive/release/lunar/pcl_conversions/0.2.1-0.tar.gz";
            sha256 = "13bb02i2vd7w0a5mk4r4h1smyxa7grxf2n44qji9vap6wvgbx5v4";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            pcl
            pcl
            pcl_msgs
            roscpp
            sensor_msgs
            std_msgs
          ];
        };
      pcl_msgs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, sensor_msgs, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "pcl_msgs";
          version = "0.2.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/pcl_msgs-release/archive/release/lunar/pcl_msgs/0.2.0-0.tar.gz";
            sha256 = "0bl1ik9ampmr35zgrk03khxnn2asqv23bz0i2q2hsq1g0xh49866";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
            sensor_msgs
            std_msgs
          ];
        };
      pcl_ros = { Cocoa, catkin, cmake, cmake_modules, dynamic_reconfigure, eigen, genmsg, gtest, libobjc, nodelet, nodelet_topic_tools, pcl, pcl_conversions, pkgconfig, pyEnv, rosconsole, roslib, stdenv, tf, tf2_eigen }:
      mkRosCmakePackage {
          name = "pcl_ros";
          version = "1.5.3-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/perception_pcl-release/archive/release/lunar/pcl_ros/1.5.3-0.tar.gz";
            sha256 = "0cmkhhrg74ijaw3zm90r4vqy1sn95pzj0h87z52ysb2v6d01233w";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            genmsg
            rosconsole
            roslib
            dynamic_reconfigure
            eigen
            nodelet
            nodelet_topic_tools
            pcl_conversions
            pcl
            tf
            tf2_eigen
            dynamic_reconfigure
            eigen
            nodelet
            nodelet_topic_tools
            pcl_conversions
            pcl
            tf
            tf2_eigen
          ] ++ stdenv.lib.optionals stdenv.isDarwin [
            libobjc
            Cocoa
          ];
          preConfigure = ''
            sed -i 's/find_package(Eigen3 REQUIRED)//' ./CMakeLists.txt
          '';
        };
      perception_pcl = { catkin, cmake, gtest, pcl_conversions, pcl_msgs, pcl_ros, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "perception_pcl";
          version = "1.5.3-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/perception_pcl-release/archive/release/lunar/perception_pcl/1.5.3-0.tar.gz";
            sha256 = "1lk53swfc62rhgqk54ywqbx3yh6jgvwnamb2sfsx01bhmvcap4hy";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            pcl_conversions
            pcl_msgs
            pcl_ros
          ];
        };
      pluginlib = { catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, stdenv, class_loader, rosconsole, roslib }:
  # mkRosPythonPackage {
  mkRosCmakePackage {
          name = "pluginlib";
          version = "1.11.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/pluginlib-release/archive/release/lunar/pluginlib/1.11.2-0.tar.gz";
            sha256 = "0nj9kc3f158lghxjr7jmla46fn831z5m8zck3ghaj8kxg57yngay";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cmake_modules
            class_loader
            rosconsole
            roslib
          ];
        };
      mk = { catkin, cmake, gtest, pkgconfig, pyEnv, rosbuild, stdenv }:
      mkRosCmakePackage {
          name = "mk";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/mk/1.14.2-0.tar.gz";
            sha256 = "0nfk3pblb4498c8q95hcyfi9mdixld3vys15fnxjp7fjrw7vlw18";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            rosbuild
          ];
        };
      ros = { catkin, cmake, gtest, mk, pkgconfig, pyEnv, rosbash, rosboost_cfg, rosbuild, rosclean, roscreate, roslang, roslib, rosmake, rosunit, stdenv }:
      mkRosCmakePackage {
          name = "ros";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/ros/1.14.2-0.tar.gz";
            sha256 = "00a97c75zfham2zwc001cyqzi4g4br8ry3xsjyq7cnzigfwhkwkq";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            mk
            rosbash
            rosboost_cfg
            rosbuild
            rosclean
            roscreate
            roslang
            roslib
            rosmake
            rosunit
          ];
        };
      rosbash = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "rosbash";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosbash/1.14.2-0.tar.gz";
            sha256 = "01fdsrffr9rlry26vb1g5109hli1xz8i5wmlrw6fnp487a7zpbjh";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
          patchPhase = ''
            sed -i 's|_perm="+111"|_perm="/111"|' ./scripts/rosrun
          '';
        };
      rosboost_cfg = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "rosboost_cfg";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosboost_cfg/1.14.2-0.tar.gz";
            sha256 = "175m1dgwy62fdgqh1mrszfaykc78kf85g4i6avdsl5hj2bmxhrxf";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      rosbuild = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "rosbuild";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosbuild/1.14.2-0.tar.gz";
            sha256 = "1f40184nq1vsg31p12w87lrhpdga2pq4fwm3fhryai2jpjn63g6c";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
            pkgconfig
          ];
        };
      rosclean = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "rosclean";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosclean/1.14.2-0.tar.gz";
            sha256 = "1ay4pr0c1fb02pvk86b1wlvcnw1fk4907wg01i17mymbv8d27ss0";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      roscreate = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "roscreate";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/roscreate/1.14.2-0.tar.gz";
            sha256 = "178xdqyazxacvpnbjxdp5m35bgj3an2yh05698a762m139hy0xlz";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      roslang = { catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "roslang";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/roslang/1.14.2-0.tar.gz";
            sha256 = "0cplv4pcl66v2cqsmh20xf6g9z76d2w1bz1cqxp3sjmiqpy18k69";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genmsg
          ];
        };
      roslib = { boost, catkin, cmake, gtest, pkgconfig, pyEnv, rospack, stdenv }:
      mkRosPythonPackage {
          name = "roslib";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/roslib/1.14.2-0.tar.gz";
            sha256 = "1x7m5qlx952q7y24wvpxvcfqy812c3n49kmllxy9lrf631akbxy5";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            rospack
          ];
        };
      rosmake = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "rosmake";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosmake/1.14.2-0.tar.gz";
            sha256 = "11d19acrk852zd7wa041qzir2jl6ad6irdx0bj6v3bb45c720757";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      rosunit = { catkin, cmake, gtest, pkgconfig, pyEnv, roslib, stdenv }:
      mkRosPythonPackage {
          name = "rosunit";
          version = "1.14.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosunit/1.14.2-0.tar.gz";
            sha256 = "0cqrvnffqs3p99l9s9wdkm4mr2z1l4jx9kgkp3qx8xqbfkhj93i2";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            roslib
          ];
        };
      message_filters = { boost, catkin, cmake, gtest, pkgconfig, pyEnv, rosconsole, roscpp, rostest, rosunit, stdenv, xmlrpcpp }:
      mkRosPythonPackage {
          name = "message_filters";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/message_filters/1.13.5-0.tar.gz";
            sha256 = "1v9hz75kxg31y79z6ffxm7vgz941j2n4hm5i5i57vwfxpx67wfjg";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            rosconsole
            roscpp
            rostest
            rosunit
            xmlrpcpp
          ];
        };
      ros_comm = { catkin, cmake, gtest, message_filters, pkgconfig, pyEnv, ros, rosbag, rosconsole, roscpp, rosgraph, rosgraph_msgs, roslaunch, roslisp, rosmaster, rosmsg, rosnode, rosout, rosparam, rospy, rosservice, rostest, rostopic, roswtf, std_srvs, stdenv, topic_tools, xmlrpcpp }:
      mkRosCmakePackage {
          name = "ros_comm";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/ros_comm/1.13.5-0.tar.gz";
            sha256 = "043sy5hcbsx220a2f9cf0ihjhw1lrnx5dagqsb5z5lwpb9klvlfm";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_filters
            ros
            rosbag
            rosconsole
            roscpp
            rosgraph
            rosgraph_msgs
            roslaunch
            roslisp
            rosmaster
            rosmsg
            rosnode
            rosout
            rosparam
            rospy
            rosservice
            rostest
            rostopic
            roswtf
            std_srvs
            topic_tools
            xmlrpcpp
          ];
        };
      rosbag = { boost, catkin, cmake, cpp_common, genmsg, genpy, gtest, pkgconfig, pyEnv, rosbag_storage, rosconsole, roscpp, roscpp_serialization, roslib, rospy, std_srvs, stdenv, topic_tools, xmlrpcpp }:
      mkRosPythonPackage {
          name = "rosbag";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosbag/1.13.5-0.tar.gz";
            sha256 = "1ndqji4cvrg8ri23si6kqc5bq3klxasxaa2bv55v2rbf6znp9wyq";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            cpp_common
            genmsg
            genpy
            rosbag_storage
            rosconsole
            roscpp
            roscpp_serialization
            roslib
            rospy
            std_srvs
            topic_tools
            xmlrpcpp
          ];
        };
      rosbag_storage = { boost, bzip2, catkin, cmake, console-bridge, cpp_common, gtest, pkgconfig, pyEnv, roscpp_serialization, roscpp_traits, roslz4, rostime, stdenv }:
      mkRosCmakePackage {
          name = "rosbag_storage";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosbag_storage/1.13.5-0.tar.gz";
            sha256 = "1rbshi7nvpxw58r33iwnbfylq7h64pb0lz5v6y12nqpigzrkvmh4";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            bzip2
            catkin
            cpp_common
            console-bridge
            roscpp_serialization
            roscpp_traits
            roslz4
            rostime
          ];
        };
      rosconsole = { apr, boost, catkin, cmake, cpp_common, gtest, log4cxx, pkgconfig, pyEnv, rosbuild, rostime, rosunit, stdenv }:
      mkRosCmakePackage {
          name = "rosconsole";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosconsole/1.13.5-0.tar.gz";
            sha256 = "07mirws4nbl5h7b3a3c5l8kx5rc3s6124rw590c8rldz0bqdxrmi";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            apr
            boost
            catkin
            cpp_common
            log4cxx
            rosbuild
            rostime
            rosunit
          ];
        };
      roscpp = { catkin, cmake, cpp_common, gtest, message_generation, message_runtime, pkgconfig, pyEnv, rosconsole, roscpp_serialization, roscpp_traits, rosgraph_msgs, roslang, rostime, std_msgs, stdenv, xmlrpcpp }:
      mkRosCmakePackage {
          name = "roscpp";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/roscpp/1.13.5-0.tar.gz";
            sha256 = "0ks1gaybsbwrjjx6l0vx16wh95y34ck6w3w0qyj6z3pg3qd6m3dj";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cpp_common
            message_generation
            message_runtime
            pkgconfig
            rosconsole
            roscpp_serialization
            roscpp_traits
            rosgraph_msgs
            roslang
            rostime
            std_msgs
            xmlrpcpp
          ];
        };
      rosgraph = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "rosgraph";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosgraph/1.13.5-0.tar.gz";
            sha256 = "0w5yn0i1cf1xdqc53bi8ij3qwxjxxg80npxpfzvhxj9xfqx6whdn";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      roslaunch = { catkin, cmake, gtest, pkgconfig, pyEnv, rosclean, rosgraph_msgs, roslib, rosmaster, rosout, rosparam, rosunit, stdenv }:
      mkRosPythonPackage {
          name = "roslaunch";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/roslaunch/1.13.5-0.tar.gz";
            sha256 = "0rvf7wnq7jcz0hqmqv42xxsr6fr97vh0n534v0h5qgkawlpvyrww";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            rosclean
            rosgraph_msgs
            roslib
            rosmaster
            rosout
            rosparam
            rosunit
          ];
        };
      roslz4 = { catkin, cmake, gtest, lz4, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "roslz4";
          version = "4-1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/roslz4/1.13.5-0.tar.gz";
            sha256 = "0369kqa0ilzhp7mhb135npc2q4yk9lzi6xra6vri6aaykblf0ri4";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            lz4
          ];
        };
      rosmaster = { catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, stdenv }:
      mkRosPythonPackage {
          name = "rosmaster";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosmaster/1.13.5-0.tar.gz";
            sha256 = "0iap8h7qznr6y4b7iy7dsvwn0xala1cvfcfv3cv7xag3fxvz8rrg";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            rosgraph
          ];
        };
      rosmsg = { catkin, cmake, genmsg, genpy, gtest, pkgconfig, pyEnv, rosbag, roslib, stdenv }:
      mkRosPythonPackage {
          name = "rosmsg";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosmsg/1.13.5-0.tar.gz";
            sha256 = "17bf4w8xrbxz4jbpia4f55aprhnvjkn8j6wrc5vi879dhylhsz6j";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genmsg
            genpy
            rosbag
            roslib
          ];
        };
      rosnode = { catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, rostest, rostopic, stdenv }:
      mkRosPythonPackage {
          name = "rosnode";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosnode/1.13.5-0.tar.gz";
            sha256 = "0jlrkik22nwy88p7i49xq8ldypbggd626fksrdnjyq8w31ll75ls";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            rosgraph
            rostest
            rostopic
          ];
        };
      rosout = { catkin, cmake, gtest, pkgconfig, pyEnv, roscpp, rosgraph_msgs, stdenv }:
      mkRosCmakePackage {
          name = "rosout";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosout/1.13.5-0.tar.gz";
            sha256 = "10n6z2gsl0ypyi1n5l2szq1s49abwvz9bq1akhqj8j1sj3v996gy";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            roscpp
            rosgraph_msgs
          ];
        };
      rosparam = { catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, stdenv }:
      mkRosPythonPackage {
          name = "rosparam";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosparam/1.13.5-0.tar.gz";
            sha256 = "1dxqnhs156anqsvhfnnfpxsb8fr5nghf1bs7z6gmh649bxwa63fx";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            rosgraph
          ];
        };
      rospy = { catkin, cmake, genpy, gtest, pkgconfig, pyEnv, roscpp, rosgraph, rosgraph_msgs, roslib, std_msgs, stdenv }:
      mkRosPythonPackage {
          name = "rospy";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rospy/1.13.5-0.tar.gz";
            sha256 = "0q5ri153dbd85hm3ggx6z6hdsjj1ivs8gmd193yh5jfznbyrwhk7";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genpy
            roscpp
            rosgraph
            rosgraph_msgs
            roslib
            std_msgs
          ];
        };
      rosservice = { catkin, cmake, genpy, gtest, pkgconfig, pyEnv, rosgraph, roslib, rosmsg, rospy, stdenv }:
      mkRosPythonPackage {
          name = "rosservice";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosservice/1.13.5-0.tar.gz";
            sha256 = "0l0rrpxq940hdc6l6hs3l0nii90hchmfjv6w8siiwrynxqvny9m4";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genpy
            rosgraph
            roslib
            rosmsg
            rospy
          ];
        };
      rostest = { boost, catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, roslaunch, rosmaster, rospy, rosunit, stdenv }:
      mkRosPythonPackage {
          name = "rostest";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rostest/1.13.5-0.tar.gz";
            sha256 = "1x08ry1rwz69k3g5k9dksz3dw2vi4f3k5mm6mgj86snfzlgz1ipf";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            rosgraph
            roslaunch
            rosmaster
            rospy
            rosunit
          ];
        };
      rostopic = { catkin, cmake, genpy, gtest, pkgconfig, pyEnv, rosbag, rospy, rostest, stdenv }:
      mkRosPythonPackage {
          name = "rostopic";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rostopic/1.13.5-0.tar.gz";
            sha256 = "0h8apzh9v0i4sp44di8r36gii6mpxr5c106slmx3lx6kgi47ncb0";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genpy
            rosbag
            rospy
            rostest
          ];
        };
      roswtf = { catkin, cmake, gtest, pkgconfig, pyEnv, rosbuild, rosgraph, roslaunch, roslib, rosnode, rosservice, rostest, stdenv }:
      mkRosPythonPackage {
          name = "roswtf";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/roswtf/1.13.5-0.tar.gz";
            sha256 = "1ykqaxhc18nzkj4r5badglb3694mvchipm2w7mrrs1gjkrqvb6nj";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            rosbuild
            rosgraph
            roslaunch
            roslib
            rosnode
            rosservice
            rostest
          ];
        };
      topic_tools = { catkin, cmake, cpp_common, gtest, message_generation, message_runtime, pkgconfig, pyEnv, rosconsole, roscpp, rostest, rostime, rosunit, std_msgs, stdenv, xmlrpcpp }:
      mkRosPythonPackage {
          name = "topic_tools";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/topic_tools/1.13.5-0.tar.gz";
            sha256 = "0nr4f6hdvsd82c0vijbx6kz1avhzgsj200mkgi2p0csr3g1wj4y3";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cpp_common
            message_generation
            message_runtime
            rosconsole
            roscpp
            rostest
            rostime
            rosunit
            std_msgs
            xmlrpcpp
          ];
        };
      xmlrpcpp = { catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "xmlrpcpp";
          version = "1.13.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/xmlrpcpp/1.13.5-0.tar.gz";
            sha256 = "0qawf5x2shbn5bpp3zwfj1pgnhyyv7l8nl6nm9mlljmzwsjbh8wn";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cpp_common
          ];
        };
      rosgraph_msgs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "rosgraph_msgs";
          version = "1.11.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm_msgs-release/archive/release/lunar/rosgraph_msgs/1.11.2-0.tar.gz";
            sha256 = "0mxa9c7ja0aj58gd7kbp84k5vird2b6jlv36fwksf0z7q593h38s";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
            std_msgs
          ];
        };
      std_srvs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "std_srvs";
          version = "1.11.2-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm_msgs-release/archive/release/lunar/std_srvs/1.11.2-0.tar.gz";
            sha256 = "01qzn821hnb6896vc3mgnfcv8jjrp31ynvzg3dk467bhppvkvaap";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
          ];
        };
      rosbag_migration_rule = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "rosbag_migration_rule";
          version = "1.0.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/rosbag_migration_rule-release/archive/release/lunar/rosbag_migration_rule/1.0.0-0.tar.gz";
            sha256 = "01644vwszrdi28n7xmd24l8v7bh65s2h3glfxkssh66kk6bk9m78";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      rosconsole_bridge = { catkin, cmake, console-bridge, gtest, pkgconfig, pyEnv, rosconsole, stdenv }:
      mkRosCmakePackage {
          name = "rosconsole_bridge";
          version = "0.5.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/rosconsole_bridge-release/archive/release/lunar/rosconsole_bridge/0.5.0-0.tar.gz";
            sha256 = "00z7k8isgsnq5rv5hbvfyhb8cap4hjgbnadifq6jraqbdfgghvd3";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            console-bridge
            rosconsole
          ];
        };
      cpp_common = { boost, catkin, cmake, console-bridge, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "cpp_common";
          version = "0.6.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/cpp_common/0.6.7-0.tar.gz";
            sha256 = "1ybh3civd1vy20j6i2a4gyc6v572yp1xphhdcghzlsppr0f19y1j";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            console-bridge
          ];
        };
      roscpp_core = { catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, roscpp_serialization, roscpp_traits, rostime, stdenv }:
      mkRosCmakePackage {
          name = "roscpp_core";
          version = "0.6.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/roscpp_core/0.6.7-0.tar.gz";
            sha256 = "1jxkr5jhgvqwk11fhla03rgdfmd39n8shqrb24wdim2xr68cx2hb";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cpp_common
            roscpp_serialization
            roscpp_traits
            rostime
          ];
        };
      roscpp_serialization = { catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, roscpp_traits, rostime, stdenv }:
      mkRosCmakePackage {
          name = "roscpp_serialization";
          version = "0.6.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/roscpp_serialization/0.6.7-0.tar.gz";
            sha256 = "085yzyhmd9yqpi44pjhxp5p2c30x22zilr3iipwgp3qpb4z2mzhz";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cpp_common
            roscpp_traits
            rostime
          ];
        };
      roscpp_traits = { catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, rostime, stdenv }:
      mkRosCmakePackage {
          name = "roscpp_traits";
          version = "0.6.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/roscpp_traits/0.6.7-0.tar.gz";
            sha256 = "1p36jl0fh75zqbl0wmsrdy101ym5z0lpcmra53ilr7p3vjn6jsbs";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cpp_common
            rostime
          ];
        };
      rostime = { boost, catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "rostime";
          version = "0.6.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/rostime/0.6.7-0.tar.gz";
            sha256 = "03rm1ba8xmh8bsgacm2r5grpdcp7jz2w4lg1winpq5i4ci08sjxv";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            cpp_common
          ];
        };
      roslisp = { catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph_msgs, roslang, rospack, sbcl, std_srvs, stdenv }:
      mkRosCmakePackage {
          name = "roslisp";
          version = "1.9.21-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roslisp-release/archive/release/lunar/roslisp/1.9.21-0.tar.gz";
            sha256 = "1fhbspfpp29z39bqm1vm0nj5687jd0v7jx0ghzfpsq23s5k233my";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            rosgraph_msgs
            roslang
            rospack
            sbcl
            std_srvs
          ];
        };
      rospack = { boost, catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, stdenv, tinyxml2 }:
      mkRosCmakePackage {
          name = "rospack";
          version = "2.4.3-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/rospack-release/archive/release/lunar/rospack/2.4.3-0.tar.gz";
            sha256 = "0ikhpw1gp02y92i0sdfnvxqk7qx84pn0vw8qh2jzyh1fjirakp61";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            cmake_modules
            gtest
            pkgconfig
            tinyxml2
          ];
        };
      std_msgs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "std_msgs";
          version = "0.5.11-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/std_msgs-release/archive/release/lunar/std_msgs/0.5.11-0.tar.gz";
            sha256 = "016h60nqm49skjki8p4gxrz0p9bmz8l6i14l1qy5x0r1v6czbdqd";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            message_generation
            message_runtime
          ];
        };
      cv_bridge = { boost, catkin, cmake, gtest, opencv3, pkgconfig, pyEnv, rosconsole, sensor_msgs, stdenv }:
      mkRosPythonPackage {
          name = "cv_bridge";
          version = "1.12.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/lunar/cv_bridge/1.12.7-0.tar.gz";
            sha256 = "15201pl562mj5y75vrgpq8s9ghy2fbbbh6cjkqs796gzi8dh0vhk";
    };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            pyEnv
            gtest
            boost
            catkin
            # opencv3
            rosconsole
            sensor_msgs
          ];
        };
      image_geometry = { catkin, cmake, gtest, opencv3, pkgconfig, pyEnv, sensor_msgs, stdenv }:
      mkRosPythonPackage {
          name = "image_geometry";
          version = "1.12.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/lunar/image_geometry/1.12.7-0.tar.gz";
            sha256 = "0j34pp3f7v5zs50ldj475j29bnnwciphlv03afg7cc4asj4pzplv";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            opencv3
            sensor_msgs
          ];
        };
      vision_opencv = { catkin, cmake, cv_bridge, gtest, image_geometry, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "vision_opencv";
          version = "1.12.7-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/lunar/vision_opencv/1.12.7-0.tar.gz";
            sha256 = "12k9q27w5agfvi4nb1y6fl0fw17xybhaka0f5sh98h347i67schc";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            cv_bridge
            image_geometry
          ];
        };
    };
    packageSet = stdenv.lib.mapAttrs (_:
    v:
      stdenv.lib.callPackageWith (deps // packageSet) v {}) (rosPackageSet // extraPackages);
    in {
      inherit packageSet;
      packages = stdenv.lib.attrValues packageSet;
      definitions = rosPackageSet;
      shell = stdenv.mkDerivation {
        name = "rosPackages";
        buildInputs = [
          cmake
          pkgconfig
          glib
        ] ++ stdenv.lib.attrValues packageSet;
        src = [];
        shellHook = rosShell;
      };
    }
