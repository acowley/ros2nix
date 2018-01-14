{ Cocoa, apr, atk, boost, bzip2, cmake, console-bridge, poco, extraPackages ? {}, fetchurl, gdk_pixbuf, glib, gtest, libobjc, log4cxx, lz4, mkRosCmakePackage, mkRosPythonPackage, tree, graphviz, opencv3, pango, pkgconfig, rosShell, sbcl, stdenv, tinyxml2, tinyxml-2, ... }@deps:
let
    rosPackageSet = {
      catkin = { cmake, gtest, pkgconfig, pyEnv, stdenv }:
  # mkRosPythonPackage {
  mkRosCmakePackage {
          name = "catkin";
          version = "0.7.8-0";
          src = fetchurl {
            url = "https://github.com/ros-gbp/catkin-release/archive/release/lunar/catkin/0.7.8-0.tar.gz";
            sha256 = "1f832r7hc1nd1ss032m9fhpa61vg0nbkwhy78x230ah47gvysmr0";
    };
    buildInputs = [ tree ];
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
  # mkRosPythonPackage {
      mkRosCmakePackage {
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
  # mkRosPythonPackage {
  mkRosCmakePackage {
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
      rospack = { boost, catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, stdenv, tinyxml-2 }:
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
