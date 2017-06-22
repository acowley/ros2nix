{ Cocoa, apr, atk, boost, bzip2, cmake, console-bridge, eigen, extraPackages ? {}, fetchurl, gdk_pixbuf, glib, graphviz, gtest, gtk2, libobjc, libogg, libtheora, libyamlcpp, log4cxx, lz4, mkRosCmakePackage, mkRosPythonPackage, opencv3, pango, pcl, pkgconfig, poco, rosShell, sbcl, stdenv, tinyxml, tinyxml2, tinyxml-2, uuid, qt5, ... }@deps:
let
    rosPackageSet = {
      actionlib = { actionlib_msgs, boost, catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, roscpp, rospy, rostest, std_msgs, stdenv }:
      mkRosPythonPackage {
          name = "actionlib";
          version = "1.11.9-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/actionlib-release/archive/release/lunar/actionlib/1.11.9-0.tar.gz";
            sha256 = "0c3w2x2r7v0141886qdab6nmpdl6ac909nmlz0a7a2p58fi8207v";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            actionlib_msgs
            boost
            catkin
            message_generation
            message_runtime
            roscpp
            rospy
            rostest
            std_msgs
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
      bond = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
      mkRosCmakePackage {
          name = "bond";
          version = "1.7.19-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/bond/1.7.19-0.tar.gz";
            sha256 = "00ibhp989rpz55di37lxd64dwkia1awwy49yivlhgdw8wzzfm1zv";
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
      bond_core = { bond, bondcpp, bondpy, catkin, cmake, gtest, pkgconfig, pyEnv, smclib, stdenv }:
      mkRosCmakePackage {
          name = "bond_core";
          version = "1.7.19-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/bond_core/1.7.19-0.tar.gz";
            sha256 = "12r7zmyqzchpr8qy9fig0l0r99a1b2q41ms9zzvxdr0zhg75n0s0";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            bond
            bondcpp
            bondpy
            catkin
            smclib
          ];
        };
      bondcpp = { bond, boost, catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, roscpp, smclib, stdenv, uuid }:
      mkRosCmakePackage {
          name = "bondcpp";
          version = "1.7.19-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/bondcpp/1.7.19-0.tar.gz";
            sha256 = "1mjp2q31fhx86bjndyy7ixz2jx953n0iprcv7nlxaq252swlzxnv";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            bond
            boost
            catkin
            cmake_modules
            roscpp
            smclib
            uuid
          ];
          postPatch = ''
            sed -i -e 's/find_package(UUID REQUIRED)//' -e 's/''${UUID_LIBRARIES}//' ./CMakeLists.txt
          '';
        };
      bondpy = { bond, catkin, cmake, gtest, pkgconfig, pyEnv, rospy, smclib, stdenv, uuid }:
      mkRosPythonPackage {
          name = "bondpy";
          version = "1.7.19-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/bondpy/1.7.19-0.tar.gz";
            sha256 = "1jc1k2qmh9p35a2h5admg79n9gnpj6i8jxpdw35sckcfr2hgwryi";
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
            uuid
          ];
        };
      smclib = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "smclib";
          version = "1.7.19-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/bond_core-release/archive/release/lunar/smclib/1.7.19-0.tar.gz";
            sha256 = "0vc574c6q31wjzc05jzwkf17riw81nzr09ab2455rav44hpjw4hw";
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
          version = "0.7.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/catkin-release/archive/release/lunar/catkin/0.7.6-0.tar.gz";
            sha256 = "1xabz88sbv78537nmk1k9aqpxggamb6vizyzmn1wfg69gr1zqbvp";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
          ];
          patchPhase = ''
            sed -i 's|#!@PYTHON_EXECUTABLE@|#!${pyEnv.python.passthru.interpreter}|' ./cmake/templates/_setup_util.py.in
            sed -i 's|#!/usr/bin/env python|#!${stdenv.shell} ${pyEnv}/bin/python|' ./cmake/parse_package_xml.py
            sed -i 's/PYTHON_EXECUTABLE/SHELL/' ./cmake/catkin_package_xml.cmake
            sed -i 's|#!/usr/bin/env bash|#!${stdenv.shell}|' ./cmake/templates/setup.bash.in
            sed -i 's|#!/usr/bin/env sh|#!${stdenv.shell}|' ./cmake/templates/setup.sh.in
          '';
        };
      class_loader = { boost, catkin, cmake, cmake_modules, console-bridge, gtest, pkgconfig, poco, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "class_loader";
          version = "0.3.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/class_loader-release/archive/release/lunar/class_loader/0.3.6-0.tar.gz";
            sha256 = "0c5pbq8fq71s1qvssxk9r673hgls5rqczmbkzildyf3s4pzmnfq3";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
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
      mkRosPythonPackage {
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
      dynamic_reconfigure = { boost, catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, roscpp, roscpp_serialization, roslib, rospy, rosservice, rostest, std_msgs, stdenv }:
      mkRosPythonPackage {
          name = "dynamic_reconfigure";
          version = "1.5.48-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/dynamic_reconfigure-release/archive/release/lunar/dynamic_reconfigure/1.5.48-0.tar.gz";
            sha256 = "0abbahzs7pq57sm2178wn45ad5sq8lnp7683g15p5cm4ky8s2dhj";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            message_generation
            message_runtime
            roscpp
            roscpp_serialization
            roslib
            rospy
            rosservice
            rostest
            std_msgs
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
          version = "0.5.8-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/genmsg-release/archive/release/lunar/genmsg/0.5.8-0.tar.gz";
            sha256 = "1mk0l4hkjlp16ihiap4yarvcgvpar9g9drd7ll31q9xw38pqkx1j";
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
          version = "0.6.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/genpy-release/archive/release/lunar/genpy/0.6.5-0.tar.gz";
            sha256 = "1wh202x7dnjfg14r7l08zqsjxm6jhx4f4m0a6i4r6jw8nx6g544b";
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
          version = "1.11.8-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry-release/archive/release/lunar/eigen_conversions/1.11.8-0.tar.gz";
            sha256 = "16qsdk7dym9i59iz9xv3jnhl4apfnwik09bmhsc7k829x7p6rlwa";
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
      tf = { angles, catkin, cmake, geometry_msgs, graphviz, gtest, message_filters, message_generation, message_runtime, pkgconfig, pyEnv, rosconsole, roscpp, rostest, roswtf, sensor_msgs, std_msgs, stdenv, tf2, tf2_ros }:
      mkRosPythonPackage {
          name = "tf";
          version = "1.11.8-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry-release/archive/release/lunar/tf/1.11.8-0.tar.gz";
            sha256 = "08fsbn14mz0p28yg1gf9r0mnxw3a2h31gph3vir5c86xjgg1p5k7";
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
            rostest
            roswtf
            sensor_msgs
            std_msgs
            tf2
            tf2_ros
          ];
        };
      tf2 = { catkin, cmake, console-bridge, geometry_msgs, gtest, pkgconfig, pyEnv, rostime, stdenv, tf2_msgs }:
      mkRosCmakePackage {
          name = "tf2";
          version = "2-0.5.15-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2/0.5.15-0.tar.gz";
            sha256 = "1yzrlaafk6dr2wfs8y4x6h08pdz8hkm4ki54axb58zc8s5xpxkdk";
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
      tf2_eigen = { catkin, cmake, cmake_modules, eigen, geometry_msgs, gtest, pkgconfig, pyEnv, stdenv, tf2 }:
      mkRosCmakePackage {
          name = "tf2_eigen";
          version = "0.5.15-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_eigen/0.5.15-0.tar.gz";
            sha256 = "0ds7rs05y1w9x7lk1zal1vxc7130z0gwdpkiljns51icfpbcd1pz";
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
            tf2
          ];
        };
      tf2_geometry_msgs = { catkin, cmake, geometry_msgs, gtest, orocos_kdl, pkgconfig, pyEnv, python_orocos_kdl, stdenv, tf2, tf2_ros }:
      mkRosPythonPackage {
          name = "tf2_geometry_msgs";
          version = "0.5.15-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_geometry_msgs/0.5.15-0.tar.gz";
            sha256 = "1mkdzkkz0lg0b3hs24bj4mfl6z0qgwkrzf94givpapx7wrg1a99k";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            geometry_msgs
            orocos_kdl
            python_orocos_kdl
            tf2
            tf2_ros
          ];
        };
      tf2_msgs = { actionlib_msgs, catkin, cmake, geometry_msgs, gtest, message_generation, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "tf2_msgs";
          version = "0.5.15-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_msgs/0.5.15-0.tar.gz";
            sha256 = "1xjhfahikccg8w4x26cpakx7wyb3wl8cmyrys5sd8yaa9skxg29m";
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
          version = "0.5.15-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_py/0.5.15-0.tar.gz";
            sha256 = "1gyailfjafrfnp8rid6s2qx2001w8n19i67sgi7jljvbdcvfpfvm";
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
          version = "0.5.15-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/geometry2-release/archive/release/lunar/tf2_ros/0.5.15-0.tar.gz";
            sha256 = "1wxwkanmpd2ggbivh754awznn4jvfy1i5m16q77cl92lvzh9xyin";
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
          version = "1.11.12-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/camera_calibration_parsers/1.11.12-0.tar.gz";
            sha256 = "1591ifzda40gc29pj4h22hzfinbzq33b4yb210yhw1lv97p64iix";
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
          version = "1.11.12-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/camera_info_manager/1.11.12-0.tar.gz";
            sha256 = "08rhcfq80vih3h3bxgzrn9wda5pfi9bj8ra25wyk38hp1y68l7l3";
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
          version = "1.11.12-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/image_common/1.11.12-0.tar.gz";
            sha256 = "0y1w39f3dkc9yamjmkjffqv60cmv5z3dvzzf9ypvx5dsb8gxrnzb";
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
          version = "1.11.12-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/image_transport/1.11.12-0.tar.gz";
            sha256 = "1c62n8vlkh5hvbjbklpizm0b44qipqqh1l3lib49vbdkqai82qmn";
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
          version = "1.11.12-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_common-release/archive/release/lunar/polled_camera/1.11.12-0.tar.gz";
            sha256 = "0k97wlkm38qpakvs8c8zb2p7379vscdlva3zi1mbj36z5n36gmjf";
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
          version = "1.12.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/camera_calibration/1.12.20-0.tar.gz";
            sha256 = "126v1xq571zkm6z1hzhjwpqnmmwn0nxgiy5bxppb5c732b17kjqq";
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
          version = "1.12.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/depth_image_proc/1.12.20-0.tar.gz";
            sha256 = "0h27gv6ma69wghqnix2qvn86fwwx3zpx2azgm7bjvbixdna8wapa";
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
          version = "1.12.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_pipeline/1.12.20-0.tar.gz";
            sha256 = "0g88cp931r3rcmzbl365sqzv89sbpffgwby65r412p19v51ipcj7";
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
          version = "1.12.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_proc/1.12.20-0.tar.gz";
            sha256 = "13nvxsj2xajvbsb2hl0i1afqb62aig8a863yqnyjyqi0wy5bsbb7";
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
          version = "1.12.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_publisher/1.12.20-0.tar.gz";
            sha256 = "0z12fccwcpqrd1k35qnlsix6i9vw9b9fcqr2jb215w1v8jwwq9s5";
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
          version = "1.12.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_rotate/1.12.20-0.tar.gz";
            sha256 = "14x1rnjz7x3k1r1gdqib1x9pfj53mdhb14087wiyi6dp2yj74dzx";
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
          version = "1.12.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/image_view/1.12.20-0.tar.gz";
            sha256 = "1q03z0qknrdafznca6id8zi17slys1nb5ncgk8dk0c4iv43yvdv3";
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
          version = "1.12.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/lunar/stereo_image_proc/1.12.20-0.tar.gz";
            sha256 = "1jzii6anjlz2srciv3is0hrh2xhafkw2l2njp66vs31lnssykp8z";
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
            sha256 = "0n622fgm40jbs64s91d1299zc53fj69clqhq10c1i3v2p4bcj24k";
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
            sha256 = "02dc93ylkyfw1cdxn4fjxsjdmdfvxd9bml9ypp9z2cqhpmw9vp78";
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
            sha256 = "1b7wrxsan369x7l3ahslpb01arsd5r25sw3264brc0df3rsb587k";
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
          version = "1.8.4-1";
          src = fetchurl {
            url = "https://github.com/ros-gbp/laser_filters-release/archive/release/lunar/laser_filters/1.8.4-1.tar.gz";
            sha256 = "1sknds214gc97j1cl4y4nb2l1jm283gpcfcg0pwr673fdkr84zhj";
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
      nodelet = { bondcpp, boost, catkin, cmake, cmake_modules, gtest, message_generation, message_runtime, pkgconfig, pluginlib, pyEnv, rosconsole, roscpp, rospy, std_msgs, stdenv, uuid }:
      mkRosCmakePackage {
          name = "nodelet";
          version = "1.9.10-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/lunar/nodelet/1.9.10-0.tar.gz";
            sha256 = "12nl8nz0szqhnbp7sb1z02d1j9iz3wz6iznzv15xmf2ymsj4lx8k";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            bondcpp
            boost
            catkin
            cmake_modules
            message_generation
            message_runtime
            pluginlib
            rosconsole
            roscpp
            rospy
            std_msgs
            uuid
          ];
          postPatch = ''
            sed -i -e 's/find_package(UUID REQUIRED)//' -e 's/ ''${UUID_INCLUDE_DIRS}//' -e 's/ ''${UUID_LIBRARIES}//g' ./CMakeLists.txt
          '';
        };
      nodelet_core = { catkin, cmake, gtest, nodelet, nodelet_topic_tools, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "nodelet_core";
          version = "1.9.10-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/lunar/nodelet_core/1.9.10-0.tar.gz";
            sha256 = "120ywa3j7py44aaya8ysivxaldvlwzj700w96lxcvf1q8j8x0gri";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            nodelet
            nodelet_topic_tools
          ];
        };
      nodelet_topic_tools = { boost, catkin, cmake, dynamic_reconfigure, gtest, message_filters, nodelet, pkgconfig, pluginlib, pyEnv, roscpp, stdenv }:
      mkRosCmakePackage {
          name = "nodelet_topic_tools";
          version = "1.9.10-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/lunar/nodelet_topic_tools/1.9.10-0.tar.gz";
            sha256 = "0pmbnfd96bpg9rspfpmhfyqqb9l9nnzqb76sglsxcflna2ph2qjl";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            dynamic_reconfigure
            message_filters
            nodelet
            pluginlib
            roscpp
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
      pluginlib = { boost, catkin, class_loader, cmake, cmake_modules, gtest, pkgconfig, pyEnv, rosconsole, roslib, stdenv, tinyxml }:
      mkRosPythonPackage {
          name = "pluginlib";
          version = "1.10.5-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/pluginlib-release/archive/release/lunar/pluginlib/1.10.5-0.tar.gz";
            sha256 = "1aixz12fjyan22chlj71syaskc5pk50c8a0l2ni7ffz94347lxaj";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            class_loader
            cmake_modules
            rosconsole
            roslib
            tinyxml
          ];
        };
      mk = { catkin, cmake, gtest, pkgconfig, pyEnv, rosbuild, stdenv }:
      mkRosCmakePackage {
          name = "mk";
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/mk/1.14.0-0.tar.gz";
            sha256 = "1w5gzcylgb1bddsi6ya1ad7dr8frmhmba2x1w36d5pbs7yrbp1hl";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/ros/1.14.0-0.tar.gz";
            sha256 = "0gn9k8d7cvf4nzppbxdb4c3ml7rj6kr9p0fbgh76zdqjw6gkbni6";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosbash/1.14.0-0.tar.gz";
            sha256 = "1hmjsxhvk4ksip6shb6b3jvxb9x6vwm7l32r11j01l0wkpcgqd6b";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosboost_cfg/1.14.0-0.tar.gz";
            sha256 = "0yz7bx7r9615gjm02f8nhff0izsy13qsyxk3i1swp5siqrqgjzmn";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosbuild/1.14.0-0.tar.gz";
            sha256 = "1qw3h5gwmh8m78zjx5jibm1vvzfz3b6rcb2jwadjac7p5ymzsb2a";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosclean/1.14.0-0.tar.gz";
            sha256 = "18h8v6m6ss3i77lfkwljcf1bxb5gxjxlridw86vmd7f5ld7q0arx";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/roscreate/1.14.0-0.tar.gz";
            sha256 = "13z6zgjh3bgzsh6lqhr69m1vqv3980x54rsisr4nxjp4ryy2niy9";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/roslang/1.14.0-0.tar.gz";
            sha256 = "1w22mpa9iwp2lanw5z1by1q20dwhpypgaggmwsif7h6rdsd66hck";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/roslib/1.14.0-0.tar.gz";
            sha256 = "1ngwczagxjqqv6cnpafmwhll3g6kijgam2fyjg51sxllnsllwp47";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosmake/1.14.0-0.tar.gz";
            sha256 = "0x73p3n9jymg7gc47i70q7x83b2qyh4h0x998hqmk1chzn9aam8n";
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
          version = "1.14.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/lunar/rosunit/1.14.0-0.tar.gz";
            sha256 = "0s1xdgxlzzr9hl7bw5lpyj7nq6qrds9jp9fk9hxxvkgqiw30m47g";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/message_filters/1.13.0-0.tar.gz";
            sha256 = "1hawqk4dp3ll9hb6249w2kqbki95a6h5g80qgzjr9kjbzwng0s8i";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/ros_comm/1.13.0-0.tar.gz";
            sha256 = "15x76ic6fbmskz13075qziyw2w4d6iwmf1sfi4sbn858idgambn3";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosbag/1.13.0-0.tar.gz";
            sha256 = "0cfpagwyp9z8fjpwvq8l21gg2284nbvl8zngp957x0b02q7hf1yx";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosbag_storage/1.13.0-0.tar.gz";
            sha256 = "0glz0y97i6k9xiyym2b0fgnih5fkfsmvdpwpag4lfxd40y9saba7";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosconsole/1.13.0-0.tar.gz";
            sha256 = "1v6xc2rvq0b33006f9zmpl005syhgn25crvdhnwrbljcbmjz2xbv";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/roscpp/1.13.0-0.tar.gz";
            sha256 = "0kxhn1y19v83cz7pzv2lfgkdvsn0x776k6971rqzqw0hngvi4mca";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosgraph/1.13.0-0.tar.gz";
            sha256 = "0q5ycflb0b8icwzwywidggfqhcz8ljf5760kgxd1l2vsvd6hq1yx";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/roslaunch/1.13.0-0.tar.gz";
            sha256 = "1bvcz0xb24s5y8bshfj5mrfhqhaz5k33w3q936v0p87g5bdkvxnf";
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
          version = "4-1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/roslz4/1.13.0-0.tar.gz";
            sha256 = "0xd300kwk14400fqpqcbx09rb4hbv5v979g09x1gn2l0gsc9395a";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosmaster/1.13.0-0.tar.gz";
            sha256 = "0b2zh5m0c2yxp0k6lnil1j26zvmrkryqpvrm239rnkw2jww7z3iw";
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
      rosmsg = { catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, rosbag, roslib, stdenv }:
      mkRosPythonPackage {
          name = "rosmsg";
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosmsg/1.13.0-0.tar.gz";
            sha256 = "0lwnih5y30fsmh4rlff30n056xckncasn3r2a5r04xnzh39dkd5r";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
            genmsg
            rosbag
            roslib
          ];
        };
      rosnode = { catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, rostest, rostopic, stdenv }:
      mkRosPythonPackage {
          name = "rosnode";
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosnode/1.13.0-0.tar.gz";
            sha256 = "08svgdv8mqjz39flniz53rhiymgfa5gw2qm4x0jf2ax7xyjpydzm";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosout/1.13.0-0.tar.gz";
            sha256 = "11fanww1kxi2fnfw0dd7agwwr2qhgb0dfx99z84agllzry3j5rks";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosparam/1.13.0-0.tar.gz";
            sha256 = "1hih4zi376j1j83bih2dra02ywyhlvnagizfx7savgv0dj7zxzjj";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rospy/1.13.0-0.tar.gz";
            sha256 = "0lwg5lxvcm5qvag5rgp85jh2q889xmxbxxcrw5wj69il02r9dgkz";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rosservice/1.13.0-0.tar.gz";
            sha256 = "0mglyla5cfdccmji31c7a079q0vqc9x1djiw5ah5vbzgqhy7mnp5";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rostest/1.13.0-0.tar.gz";
            sha256 = "1b9br7jyd4x305xgq3r2cvpk1qr36d56mfhjp39hqzwkr6v0illv";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/rostopic/1.13.0-0.tar.gz";
            sha256 = "0ji1vdaf1p8gfacrb70k8iyv2wwd26p1irlnkaqc68q7vp6626f8";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/roswtf/1.13.0-0.tar.gz";
            sha256 = "0ai7mphbxyqr7bfi86gp6bnq55ddfik9nh4cp8rpnwx8hd2wh98c";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/topic_tools/1.13.0-0.tar.gz";
            sha256 = "1pxx4gry753j2ha03zjpw3gnbicyzg5b1qvyyinpxd1bjcq1g338";
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
          version = "1.13.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/lunar/xmlrpcpp/1.13.0-0.tar.gz";
            sha256 = "193q5zwz5y045i128mfh0dn8c983l8nyn7b4dfzjw2ccb43h0yms";
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
          version = "0.4.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/rosconsole_bridge-release/archive/release/lunar/rosconsole_bridge/0.4.4-0.tar.gz";
            sha256 = "0sy9985nhhf8z7rfa2ai3w602fsx2q934vpcc6z1h30x4r9w784s";
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
          version = "0.6.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/cpp_common/0.6.4-0.tar.gz";
            sha256 = "1py422rcin687a7kdfwf9r55h6qziyskdxb32rjqp8pa0n2mg55f";
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
          version = "0.6.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/roscpp_core/0.6.4-0.tar.gz";
            sha256 = "0jgggmv2bcnaj39fd7bkklm6336x4bnw1sn4kkcad0jfb5xxm5cw";
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
          version = "0.6.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/roscpp_serialization/0.6.4-0.tar.gz";
            sha256 = "0zsvwdyzd8vxdswlc6w6d606ik9zffqkz6ylfmlyv35zh8vwng44";
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
          version = "0.6.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/roscpp_traits/0.6.4-0.tar.gz";
            sha256 = "0lghfl0y1n3zcplq3ljz8j9z1525gw5i59jcqgdi3rjnppvhh9wq";
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
          version = "0.6.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/lunar/rostime/0.6.4-0.tar.gz";
            sha256 = "0kd602ci5f8zjlgj913qk5g961vdix52da72cqxrb2hhv4hay51v";
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
          version = "1.9.20-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roslisp-release/archive/release/lunar/roslisp/1.9.20-0.tar.gz";
            sha256 = "08nn4mll61wkmmw68w0ki6p9vj9k4hrwpjlzq0nayq58lsim0bhw";
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
      rospack = { boost, catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, stdenv, tinyxml-2 }:
      mkRosCmakePackage {
          name = "rospack";
          version = "2.4.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/rospack-release/archive/release/lunar/rospack/2.4.1-0.tar.gz";
            sha256 = "0p4nnj246mqhsmzzvs98nxvnrc9mb9i076r7cx7rqkfm53a5nypr";
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
            tinyxml-2
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
          version = "1.12.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/lunar/cv_bridge/1.12.4-0.tar.gz";
            sha256 = "1slnqq739l2q99x6lwyks7m97c5b7b56z66pkrj15fbvkpv0wr74";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            boost
            catkin
            opencv3
            rosconsole
            sensor_msgs
          ];
        };
      image_geometry = { catkin, cmake, gtest, opencv3, pkgconfig, pyEnv, sensor_msgs, stdenv }:
      mkRosPythonPackage {
          name = "image_geometry";
          version = "1.12.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/lunar/image_geometry/1.12.4-0.tar.gz";
            sha256 = "0rjssx8p1qswqgnvbypjxwdcfcyvjfr7hdwg3m1kngzaa25rxhsr";
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
          version = "1.12.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/lunar/vision_opencv/1.12.4-0.tar.gz";
            sha256 = "05dgf3kkpd9035s8lvp2zimffwhak8miydh3c1rdw26jlxk0m6g3";
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
