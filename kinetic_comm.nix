{ Cocoa, apr, atk, boost, bzip2, cmake, console-bridge, extraPackages ? {}, fetchurl, gdk_pixbuf, glib, gtest, libobjc, log4cxx, lz4, mkRosCmakePackage, mkRosPythonPackage, opencv3, pango, pkgconfig, rosShell, sbcl, stdenv, tinyxml, ... }@deps:
let
    rosPackageSet = {
      catkin = { cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "catkin";
          version = "0.7.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/catkin-release/archive/release/kinetic/catkin/0.7.4-0.tar.gz";
            sha256 = "13zy4mg416pm3jpkngdc8ykpk9dzxgg93nlcgnrxmv1rv2fjl0bw";
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
      cmake_modules = { catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "cmake_modules";
          version = "0.4.0-1";
          src = fetchurl {
            url = "https://github.com/ros-gbp/cmake_modules-release/archive/release/kinetic/cmake_modules/0.4.0-1.tar.gz";
            sha256 = "1zphqfzklj8fvxlqrn8k978n48dy6dxnfq90gfjwsiffqp560pcv";
          };
          propagatedBuildInputs = [
            cmake
            pkgconfig
            gtest
            pyEnv
            catkin
          ];
        };
      gencpp = { catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosPythonPackage {
          name = "gencpp";
          version = "0.5.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/gencpp-release/archive/release/kinetic/gencpp/0.5.4-0.tar.gz";
            sha256 = "1mznr9hjq79idj40ka94iiaf9b7qk67sr56rghx6rv7r1c9cg59c";
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
          version = "2.2.5-1";
          src = fetchurl {
            url = "https://github.com/tork-a/geneus-release/archive/release/kinetic/geneus/2.2.5-1.tar.gz";
            sha256 = "1jxqcywwidd9qcwf0lhgwvqv7lz9m2a53s3gczysxzm2vms2wchr";
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
            url = "https://github.com/ros-gbp/genlisp-release/archive/release/kinetic/genlisp/0.4.16-0.tar.gz";
            sha256 = "0wgqvsn3bfi2dlx5qnk3p6cbm9214brgj03z0qqs867kr7aj3p0g";
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
            url = "https://github.com/ros-gbp/genmsg-release/archive/release/kinetic/genmsg/0.5.8-0.tar.gz";
            sha256 = "0nc37bivcl9fdvj2ivaj3nyyxzig5bsrxld3z90ws9z3iw8d5ikp";
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
          version = "1.0.3-0";
          src = fetchurl {
            url = "https://github.com/RethinkRobotics-release/gennodejs-release/archive/release/kinetic/gennodejs/1.0.3-0.tar.gz";
            sha256 = "00v6dhl12jgci02rk5x99wlkwqvgsjyryl9695rcdaaaim369rg1";
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
          version = "0.6.3-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/genpy-release/archive/release/kinetic/genpy/0.6.3-0.tar.gz";
            sha256 = "1lvc9xswnnp2gqbasyxhd75myv9hrw52cl7nyigrnx7h16largfp";
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
      message_generation = { catkin, cmake, gencpp, geneus, genlisp, genmsg, gennodejs, genpy, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "message_generation";
          version = "0.4.0-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/message_generation-release/archive/release/kinetic/message_generation/0.4.0-0.tar.gz";
            sha256 = "0qz446rzf441scw6fqspdm2lj21sk9kgfg51lhxf866x406wlxmc";
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
            url = "https://github.com/ros-gbp/message_runtime-release/archive/release/kinetic/message_runtime/0.4.12-0.tar.gz";
            sha256 = "0chdy02ryk70n7iwijvhxz4gg1gmxzs3yrilcwm8pdliw5fsjacy";
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
      mk = { catkin, cmake, gtest, pkgconfig, pyEnv, rosbuild, stdenv }:
      mkRosCmakePackage {
          name = "mk";
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/mk/1.13.4-0.tar.gz";
            sha256 = "1b4k7ma2fy6kxi1nmzd7bkrrysm5snbiacim1c78j1f5yxfjdh05";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/ros/1.13.4-0.tar.gz";
            sha256 = "151bxssvnqfxdd5qmjaz10xqd6mk1y1whh8h0k21fyziq4g6gs53";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/rosbash/1.13.4-0.tar.gz";
            sha256 = "15yxds0s623lp5gvlfz5f60ahddnjyw8ab4yh3fd1b9gvvm0bgkg";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/rosboost_cfg/1.13.4-0.tar.gz";
            sha256 = "0kbm0viy78sjgp8bygqjk01r6v28q6iy7c90q8vkk4wm795jpqvd";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/rosbuild/1.13.4-0.tar.gz";
            sha256 = "0lcci50wjm7i8kp0rra48wx019406q5y7hxksaprf4pw49wr176h";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/rosclean/1.13.4-0.tar.gz";
            sha256 = "0015xv4xflqsx7gkcp1md0k11fx6wf849b9agw42r9fm8s8s3iwk";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/roscreate/1.13.4-0.tar.gz";
            sha256 = "08176w0kw9d53mjgg2zy95dyhl07clrj2njibgm0c5z4c5iwvflk";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/roslang/1.13.4-0.tar.gz";
            sha256 = "0x9bdg78h89pbxpjf7h96pqlxz0152qr01q7lxyg2adgf0kq71cj";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/roslib/1.13.4-0.tar.gz";
            sha256 = "07ify1zdh3xlf35vz013x5h2gggc0j6dr9b8x98z14p5sz3y2s54";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/rosmake/1.13.4-0.tar.gz";
            sha256 = "1lc4syi4vzrq63d28183n2sjqbqvw0af7lbgd2b0hd3hibknml3q";
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
          version = "1.13.4-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros-release/archive/release/kinetic/rosunit/1.13.4-0.tar.gz";
            sha256 = "1mk7snyzf8f9vxqdfmhx3ac8858rw9zsnz1ibs4qpl2vmi2cajjb";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/message_filters/1.12.6-0.tar.gz";
            sha256 = "1akclfyw4z7bmrby63jgi54ww0ld2zixd6rzgqgvajr015sl43gg";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/ros_comm/1.12.6-0.tar.gz";
            sha256 = "0c9bnww0lp8ak6wf538skjaja2hlv9hjb573m4zgfg2vjmsv4hxi";
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
      rosbag = { boost, catkin, cmake, cpp_common, genmsg, genpy, gtest, pkgconfig, pyEnv, rosbag_storage, rosconsole, roscpp, roscpp_serialization, roslib, rospy, stdenv, topic_tools, xmlrpcpp }:
      mkRosPythonPackage {
          name = "rosbag";
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosbag/1.12.6-0.tar.gz";
            sha256 = "0xbqd4gsmy6ycjlmljbs1aas2rk23yp6qz3n0z9cl9vbcxcf40hy";
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
            topic_tools
            xmlrpcpp
          ];
        };
      rosbag_storage = { boost, bzip2, catkin, cmake, console-bridge, cpp_common, gtest, pkgconfig, pyEnv, roscpp_serialization, roscpp_traits, roslz4, rostime, stdenv }:
      mkRosCmakePackage {
          name = "rosbag_storage";
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosbag_storage/1.12.6-0.tar.gz";
            sha256 = "1gvnjpdw5xfd6vwx0sg3l8zb9x89i4c0cxgp2iv5ss48zj93pnap";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosconsole/1.12.6-0.tar.gz";
            sha256 = "1b8z7z5w0w6sn4bcipz5k72414y4fpkzwrwhvdhsx0g42crcya77";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/roscpp/1.12.6-0.tar.gz";
            sha256 = "0sd7g8lycggc73jjvg2r646qdznq2ja0kwpqqiapwy4lckyji0c2";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosgraph/1.12.6-0.tar.gz";
            sha256 = "0wlxp6pijzkzz1vxb094qh5v3k297cs60klfwh26xzbj4zyfxmiz";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/roslaunch/1.12.6-0.tar.gz";
            sha256 = "09ys6f1j8l36w7rjpbb8rdg2c9adyvdvvkv4zmwm8dz0xr5p0m9r";
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
          version = "4-1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/roslz4/1.12.6-0.tar.gz";
            sha256 = "0a2sk5xx66kxss4xwllh8cfybplp881clwx36cg65pcnasfv98iw";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosmaster/1.12.6-0.tar.gz";
            sha256 = "15rafvdk7j69h2k58czkydk0f1vdcpsfla1yxf5g44qkmidvvr4m";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosmsg/1.12.6-0.tar.gz";
            sha256 = "05jg8959k9s40hhrmg70nzci03mjl31z1l66y28qi9v1lbxamwy4";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosnode/1.12.6-0.tar.gz";
            sha256 = "02msn9f1n6c7sbfl0h4jr8lbn8s63v3k83h4ps8ds51717rzs9ib";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosout/1.12.6-0.tar.gz";
            sha256 = "035d7jlv3xhqapb80kgqkvvs0k8dgkmi1r52nsapv5x2ali447kq";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosparam/1.12.6-0.tar.gz";
            sha256 = "1c4m1ww7kb4nhb885j66hvwyd7bhjjzsxig0migy2313z6889f1v";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rospy/1.12.6-0.tar.gz";
            sha256 = "0d5ak4dx4g07jpjxm4l0lr1hf49i85i90vfazkl6mmv4y9zks2qf";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosservice/1.12.6-0.tar.gz";
            sha256 = "0722885g45kjc94yvzjjjd8q7xxk19fc61jwi9p1ygxap8b27i13";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rostest/1.12.6-0.tar.gz";
            sha256 = "15naq0xgdpcf5vw1vis1wvs498z6d8wwl3h7dk87i64hjv09hwrm";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rostopic/1.12.6-0.tar.gz";
            sha256 = "0j299v6px0w18a5aygqxv27cm0jciq2fhxlzv8zp3qvzxxmvmiag";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/roswtf/1.12.6-0.tar.gz";
            sha256 = "0i82j5kd65383szbszgz2nhyn8g9h28yli20ihd6211z6q7npwfr";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/topic_tools/1.12.6-0.tar.gz";
            sha256 = "0rmlp57yfwyw7a8dbsj82zla6mdk15chmnpyq0yflbxiskbfyv4s";
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
          version = "1.12.6-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/xmlrpcpp/1.12.6-0.tar.gz";
            sha256 = "1mlcqng8anyngc7xbsnhr2448aag01sdnabbgj8h5af3s7ajji3s";
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
            url = "https://github.com/ros-gbp/ros_comm_msgs-release/archive/release/kinetic/rosgraph_msgs/1.11.2-0.tar.gz";
            sha256 = "17xa95na0znd3a1vryvj32g86xdvqibhmw87jqkrhnf4a0vwgwf2";
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
            url = "https://github.com/ros-gbp/ros_comm_msgs-release/archive/release/kinetic/std_srvs/1.11.2-0.tar.gz";
            sha256 = "111i4ln61chjwmz743q6z71nlbsabv2h0zgmq7sqcvayw3i5af92";
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
      cpp_common = { boost, catkin, cmake, console-bridge, gtest, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "cpp_common";
          version = "0.6.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/kinetic/cpp_common/0.6.1-0.tar.gz";
            sha256 = "0gdch3gbzncal0k203iiajvmrnsp7mc5a54275rq7zb5amb1vc7a";
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
      roscpp_serialization = { catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, roscpp_traits, rostime, stdenv }:
      mkRosCmakePackage {
          name = "roscpp_serialization";
          version = "0.6.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/kinetic/roscpp_serialization/0.6.1-0.tar.gz";
            sha256 = "04qrcb35d1lrizclyvp2lcgcbq0ly0riap12lqqjyf945pb8sr7d";
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
          version = "0.6.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/kinetic/roscpp_traits/0.6.1-0.tar.gz";
            sha256 = "1qq7nwxq4ijw82wzrnf9801y01718hg6iibhwiabr4k68qdpsxln";
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
          version = "0.6.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/kinetic/rostime/0.6.1-0.tar.gz";
            sha256 = "05m92ccybjpczwar22k1q42c0sls6kh8dfs6n2ydhh3wk5hbw3q9";
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
            url = "https://github.com/ros-gbp/roslisp-release/archive/release/kinetic/roslisp/1.9.20-0.tar.gz";
            sha256 = "0wa6rkivxhdjnqd8lnr1a87r79kdcix7fxfpmggvi7vvhdgjd3ir";
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
      rospack = { boost, catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, stdenv, tinyxml }:
      mkRosCmakePackage {
          name = "rospack";
          version = "2.3.1-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/rospack-release/archive/release/kinetic/rospack/2.3.1-0.tar.gz";
            sha256 = "10pyraf3vfmdh4b1nkzrx3bnm6lmij3li6lsqapmcbpqyw595k8l";
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
            tinyxml
          ];
        };
      std_msgs = { catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, stdenv }:
      mkRosCmakePackage {
          name = "std_msgs";
          version = "0.5.10-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/std_msgs-release/archive/release/kinetic/std_msgs/0.5.10-0.tar.gz";
            sha256 = "1icw69hhx3vy15yl4f07jr6pv644xnbz9hjs2v45fya3mp72yykh";
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
    };
    packages = stdenv.lib.mapAttrs (_:
    v:
      stdenv.lib.callPackageWith (deps // packages) v {}) (rosPackageSet // extraPackages);
    in {
      inherit packages;
      definitions = rosPackageSet;
      shell = stdenv.mkDerivation {
        name = "rosPackages";
        buildInputs = [
          cmake
          pkgconfig
          glib
        ] ++ stdenv.lib.attrValues packages;
        src = [];
        shellHook = rosShell;
      };
    }