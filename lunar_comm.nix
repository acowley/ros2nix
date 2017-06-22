{ Cocoa, apr, atk, boost, bzip2, cmake, console-bridge, extraPackages ? {}, fetchurl, gdk_pixbuf, glib, gtest, libobjc, log4cxx, lz4, mkRosCmakePackage, mkRosPythonPackage, opencv3, pango, pkgconfig, rosShell, sbcl, stdenv, tinyxml2, tinyxml-2, qt5, ... }@deps:
let
    rosPackageSet = {
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
