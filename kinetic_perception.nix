{ Cocoa, apr, atk, boost, bzip2, cmake, console-bridge, eigen, ensureNewerSourcesHook, fetchurl, ffmpeg, gdk_pixbuf, glib, graphviz, gtest, gtk2, jasper, libjpeg, libobjc, libogg, libpng12, libtheora, libtiff, libv4l, libyamlcpp, log4cxx, lz4, makeWrapper, pango, pcl, pkgconfig, poco, protobuf, python27, python27Packages, sbcl, stdenv, tinyxml, unzip, uuid, vtkWithQt4, zlib, opencv3 }:
let
  callPackage = stdenv.lib.callPackageWith rosPackageSet;
  SHLIB = if stdenv.isDarwin
    then "dylib"
    else "so";
  cmakeFlags = [
    "-DPYTHON_LIBRARY=${pyEnv}/lib/libpython2.7.${SHLIB}"
    "-DPYTHON_INCLUDE_DIR=${pyEnv}/include/python2.7"
    "-DPYTHON_EXECUTABLE=${pyEnv}/bin/python"
    "-DEIGEN_ROOT_DIR=${eigen}"
    "-DEIGEN3_INCLUDE_DIR=${eigen}/include/eigen3"
    "-DEigen3_INCLUDE_DIRS=${eigen}/include/eigen3"
  ];
  postInstall = ''
    pushd ..
    if [ -f 'package.xml' ]; then
      cp package.xml ''$out
    fi
    if [ -d 'resources' ]; then
      cp -r resources ''$out
    fi
    if [ -d 'env-hooks' ]; then
      cp -r env-hooks ''$out
    fi
    popd
  '';
  rosShellHook = pkg: ''
    if [ -d "${pkg}/env-hooks" ]; then
      for i in ''$(find "${pkg}/env-hooks" -name "*.sh"); do
        source "''$i"
      done
    fi
  '';
  pyCallPackage = stdenv.lib.callPackageWith {
    inherit (pyPackages) python
    setuptools wrapPython;
    inherit (stdenv) lib;
    inherit ensureNewerSourcesHook
    stdenv fetchurl makeWrapper
    unzip;
    callPackage = pyCallPackage;
  };
  pyBuild = pyCallPackage ./python-install.nix {};
  pyPackages = python27Packages.override (_:
  {
    python = pyEnv.python;
  }) // callPackage ./ros-python-packages.nix {
    inherit fetchurl;
    inherit (pyPackages) buildPythonPackage;
    extradeps = {
      inherit (pyPackages) setuptools;
    };
  };
  pyEnv = python27.buildEnv.override {
    extraLibs = with pyPackages; [
      numpy
      setuptools
      sphinx
      # six
      dateutil
      docutils
      argparse
      pyyaml
      nose
      rospkg
      rosdep
      rosinstall-generator
      wstool
      rosinstall
      catkin_tools
      catkin_pkg
      bloom
      empy
      matplotlib
      pillow
      pydot
      paramiko
      coverage
      netifaces
      mock
      psutil
      pyqt4
      # pyside
      defusedxml
      (pygraphviz.override { doCheck = false; })
      # (callPackage ./sip.nix { inherit fetchurl python buildPythonPackage; })
    ];
  };
  rosPackageSet = {
    inherit stdenv pyEnv glib pango
    cmake libobjc Cocoa apr boost
    bzip2 console-bridge eigen
    ffmpeg graphviz gtest gtk2
    jasper libjpeg libogg libpng12
    libtheora libtiff
    libyamlcpp log4cxx lz4 pcl
    pkgconfig poco protobuf sbcl
    tinyxml uuid vtkWithQt4 zlib opencv3;
    inherit (pyPackages) buildPythonPackage;
    actionlib = callPackage ({ actionlib_msgs, boost, catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, roscpp, rospy, rostest, std_msgs, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "actionlib";
      version = "1.11.6-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/actionlib-release/archive/release/kinetic/actionlib/1.11.6-0.tar.gz";
        sha256 = "1vjlx7ydmfl1bpl2ki7lqnfs4d1q3zjfv1f0z44qc2fy4i2f5732";
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
      inherit cmakeFlags postInstall;
    })) {};
    angles = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "angles";
      version = "1.9.10-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry_angles_utils-release/archive/release/kinetic/angles/1.9.10-0.tar.gz";
        sha256 = "00ikqbg2a57vj9ldm9sfnbxbiyb65gxvg2wax6ys96bcgjsy0vwx";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
      ];
      inherit cmakeFlags postInstall;
    }) {};
    bond = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "bond";
      version = "1.7.17-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/bond_core-release/archive/release/kinetic/bond/1.7.17-0.tar.gz";
        sha256 = "0ghhzwv6qbd82k5ykiw4x78j9lqmxd3g5m8yfkj1g4mbhhgqg14y";
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
      inherit cmakeFlags postInstall;
    }) {};
    bond_core = callPackage ({ bond, bondcpp, bondpy, catkin, cmake, gtest, pkgconfig, pyEnv, smclib, stdenv }:
    stdenv.mkDerivation {
      name = "bond_core";
      version = "1.7.17-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/bond_core-release/archive/release/kinetic/bond_core/1.7.17-0.tar.gz";
        sha256 = "1xsm456knv7padcdc4fbpzqnzqv4gc6kqygcci2qpnzsg1in74mp";
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
      inherit cmakeFlags postInstall;
    }) {};
    bondcpp = callPackage ({ bond, boost, catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, roscpp, smclib, stdenv, uuid }:
    stdenv.mkDerivation {
      name = "bondcpp";
      version = "1.7.17-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/bond_core-release/archive/release/kinetic/bondcpp/1.7.17-0.tar.gz";
        sha256 = "1khxb59hbn5zmzz6ihiqx4ri8g2z8sl7vlng48nzw53cg41x2q4c";
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
      inherit cmakeFlags postInstall;
    }) {};
    bondpy = callPackage ({ bond, catkin, cmake, gtest, pkgconfig, pyEnv, rospy, smclib, stdenv, uuid }:
    stdenv.mkDerivation (pyBuild {
      name = "bondpy";
      version = "1.7.17-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/bond_core-release/archive/release/kinetic/bondpy/1.7.17-0.tar.gz";
        sha256 = "0a042s6vhayq249rjrjgrlhgg0q4iw9iv0w833s2l574298d9d4j";
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
      inherit cmakeFlags postInstall;
    })) {};
    smclib = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "smclib";
      version = "1.7.17-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/bond_core-release/archive/release/kinetic/smclib/1.7.17-0.tar.gz";
        sha256 = "05gj1vkjdg1rmw2armmpmjxqdrvzjl2rfp0bv4rs7f3sa3n32rx8";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
      ];
      inherit cmakeFlags postInstall;
    })) {};
    catkin = callPackage ({ cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
      patchPhase = ''
        sed -i 's|#!@PYTHON_EXECUTABLE@|#!${pyEnv.python.passthru.interpreter}|' ./cmake/templates/_setup_util.py.in
        sed -i 's/PYTHON_EXECUTABLE/SHELL/' ./cmake/catkin_package_xml.cmake
        sed -i 's|#!/usr/bin/env bash|#!${stdenv.shell}|' ./cmake/templates/setup.bash.in
        sed -i 's|#!/usr/bin/env sh|#!${stdenv.shell}|' ./cmake/templates/setup.sh.in
      '';
    })) {};
    class_loader = callPackage ({ boost, catkin, cmake, cmake_modules, console-bridge, gtest, pkgconfig, poco, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "class_loader";
      version = "0.3.5-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/class_loader-release/archive/release/kinetic/class_loader/0.3.5-0.tar.gz";
        sha256 = "0ak7k92kg16w264a6385ylz0ma96mbp3yjhm3vwy0xz2b194nhc6";
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
      inherit cmakeFlags postInstall;
    }) {};
    cmake_modules = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    actionlib_msgs = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "actionlib_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/actionlib_msgs/1.12.4-0.tar.gz";
        sha256 = "1hfzidv33zkk7qph6adi07z1j6gqzfjv7zsg2xlpx5pbsv2bkwz4";
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
      inherit cmakeFlags postInstall;
    }) {};
    common_msgs = callPackage ({ actionlib_msgs, catkin, cmake, diagnostic_msgs, geometry_msgs, gtest, nav_msgs, pkgconfig, pyEnv, sensor_msgs, shape_msgs, stdenv, stereo_msgs, trajectory_msgs, visualization_msgs }:
    stdenv.mkDerivation {
      name = "common_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/common_msgs/1.12.4-0.tar.gz";
        sha256 = "0jwqsxr52dcbfy9hf00fvc5fvqx2f3dxs93sjalj4njjr7cqq0ay";
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
      inherit cmakeFlags postInstall;
    }) {};
    diagnostic_msgs = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "diagnostic_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/diagnostic_msgs/1.12.4-0.tar.gz";
        sha256 = "0gxwf33mzyk39mmpdln6jxh9s64pwbalx82z5ym6hkn1hgd8sjs0";
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
      inherit cmakeFlags postInstall;
    }) {};
    geometry_msgs = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "geometry_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/geometry_msgs/1.12.4-0.tar.gz";
        sha256 = "04200rym9d6bdi3lx0sgwzi56c8y47zba5mm8pwclpjp10b8d5lb";
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
      inherit cmakeFlags postInstall;
    }) {};
    nav_msgs = callPackage ({ actionlib_msgs, catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "nav_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/nav_msgs/1.12.4-0.tar.gz";
        sha256 = "05j133jxjnwfqqnpgkq4cj77yvqhmmwnns5czrf1ng776khs2hk3";
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
      inherit cmakeFlags postInstall;
    }) {};
    sensor_msgs = callPackage ({ catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "sensor_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/sensor_msgs/1.12.4-0.tar.gz";
        sha256 = "1bdrjnybkj3qic34jlgi0m3xm674104f3dcf5v8qzq6a4ix3zb3m";
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
      inherit cmakeFlags postInstall;
    })) {};
    shape_msgs = callPackage ({ catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "shape_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/shape_msgs/1.12.4-0.tar.gz";
        sha256 = "0p61wjqi7mzf975m4cb0vzvh0dgiprjrps34psn2gm697x4v62mx";
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
      inherit cmakeFlags postInstall;
    }) {};
    stereo_msgs = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, sensor_msgs, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "stereo_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/stereo_msgs/1.12.4-0.tar.gz";
        sha256 = "1j9maf6nam3q7d90nikrcdajkpfcpbqd9z4bssk9ycac9apf61cz";
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
      inherit cmakeFlags postInstall;
    }) {};
    trajectory_msgs = callPackage ({ catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, rosbag_migration_rule, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "trajectory_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/trajectory_msgs/1.12.4-0.tar.gz";
        sha256 = "01gkqq57m886sk0ibjgcxbr2n6gd5b2qqzjdr0179vrhfarqv3wx";
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
      inherit cmakeFlags postInstall;
    }) {};
    visualization_msgs = callPackage ({ catkin, cmake, geometry_msgs, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "visualization_msgs";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/common_msgs-release/archive/release/kinetic/visualization_msgs/1.12.4-0.tar.gz";
        sha256 = "1a9ir5qc9xl74x0afd4j8lric8z1d0p53bryvf4qsx3mrlagvhc7";
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
      inherit cmakeFlags postInstall;
    }) {};
    dynamic_reconfigure = callPackage ({ boost, catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, roscpp, roscpp_serialization, roslib, rospy, rosservice, rostest, std_msgs, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "dynamic_reconfigure";
      version = "1.5.43-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/dynamic_reconfigure-release/archive/release/kinetic/dynamic_reconfigure/1.5.43-0.tar.gz";
        sha256 = "0jvagr8gxj2vd8rxz1sipgyb0xhiwqcxr7313dsbj8xznd6s0bav";
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
      inherit cmakeFlags postInstall;
    })) {};
    filters = callPackage ({ catkin, cmake, gtest, pkgconfig, pluginlib, pyEnv, rosconsole, roscpp, roslib, rostest, stdenv }:
    stdenv.mkDerivation {
      name = "filters";
      version = "1.7.4-1";
      src = fetchurl {
        url = "https://github.com/ros-gbp/filters-release/archive/release/kinetic/filters/1.7.4-1.tar.gz";
        sha256 = "0qmnzhcalf56pjwx0ihd6mdhvxa6a7xz3wzqm1l61ni55hc2ww0i";
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
      inherit cmakeFlags postInstall;
    }) {};
    gencpp = callPackage ({ catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
      patchPhase = ''
        sed -i 's/''${PYTHON_EXECUTABLE} //' ./cmake/gencpp-extras.cmake.em
      '';
    })) {};
    geneus = callPackage ({ catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
      preConfigure = ''
        sed -i 's/COMMAND ''${CATKIN_ENV} ''${PYTHON_EXECUTABLE}/COMMAND ''${CATKIN_ENV}/' ./cmake/geneus-extras.cmake.em
      '';
    })) {};
    genlisp = callPackage ({ catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
      patchPhase = ''
        sed -i 's/''${PYTHON_EXECUTABLE} //' ./cmake/genlisp-extras.cmake.em
      '';
    })) {};
    genmsg = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
      patchPhase = ''
        sed -i 's/''${PYTHON_EXECUTABLE} ''${GENMSG_CHECK_DEPS_SCRIPT}/''${GENMSG_CHECK_DEPS_SCRIPT}/' ./cmake/pkg-genmsg.cmake.em
      '';
    })) {};
    gennodejs = callPackage ({ catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
    })) {};
    genpy = callPackage ({ catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "genpy";
      version = "0.6.2-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/genpy-release/archive/release/kinetic/genpy/0.6.2-0.tar.gz";
        sha256 = "0kvp2lycgbcb73g4r2dmn6vhpb38csg4h5ds49r5mz9rfmb6n7i8";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
        genmsg
      ];
      inherit cmakeFlags postInstall;
      patchPhase = ''
        sed -i 's/''${PYTHON_EXECUTABLE} //' ./cmake/genpy-extras.cmake.em
      '';
    })) {};
    eigen_conversions = callPackage ({ catkin, cmake, cmake_modules, eigen, geometry_msgs, gtest, orocos_kdl, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "eigen_conversions";
      version = "1.11.8-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry-release/archive/release/kinetic/eigen_conversions/1.11.8-0.tar.gz";
        sha256 = "1apw9s053q05qbvsf2ll1yqrpn10r545ywj7yjbgs9nlrk6wd7jv";
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
      inherit cmakeFlags postInstall;
    }) {};
    tf = callPackage ({ angles, catkin, cmake, geometry_msgs, graphviz, gtest, message_filters, message_generation, message_runtime, pkgconfig, pyEnv, rosconsole, roscpp, rostest, roswtf, sensor_msgs, std_msgs, stdenv, tf2, tf2_ros }:
    stdenv.mkDerivation (pyBuild {
      name = "tf";
      version = "1.11.8-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry-release/archive/release/kinetic/tf/1.11.8-0.tar.gz";
        sha256 = "1qly96yl8xxsnk5dj82sbap0bjsi4gwykk8r84qn3ya30yvdfvmk";
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
      inherit cmakeFlags postInstall;
    })) {};
    tf2 = callPackage ({ catkin, cmake, console-bridge, geometry_msgs, gtest, pkgconfig, pyEnv, rostime, stdenv, tf2_msgs }:
    stdenv.mkDerivation {
      name = "tf2";
      version = "2-0.5.13-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry2-release/archive/release/kinetic/tf2/0.5.13-0.tar.gz";
        sha256 = "176ys827aa2ywgrvh90i1hqw3bn5cl08di14r5izxbx36zz8mgm0";
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
      inherit cmakeFlags postInstall;
    }) {};
    tf2_eigen = callPackage ({ catkin, cmake, cmake_modules, eigen, geometry_msgs, gtest, pkgconfig, pyEnv, stdenv, tf2 }:
    stdenv.mkDerivation {
      name = "tf2_eigen";
      version = "0.5.13-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry2-release/archive/release/kinetic/tf2_eigen/0.5.13-0.tar.gz";
        sha256 = "14f44l531r8id7k5zjjs3yvvz76w0krmr3f71q4v1h04hnqj1xyf";
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
      inherit cmakeFlags postInstall;
    }) {};
    tf2_geometry_msgs = callPackage ({ catkin, cmake, geometry_msgs, gtest, orocos_kdl, pkgconfig, pyEnv, python_orocos_kdl, stdenv, tf2, tf2_ros }:
    stdenv.mkDerivation (pyBuild {
      name = "tf2_geometry_msgs";
      version = "0.5.13-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry2-release/archive/release/kinetic/tf2_geometry_msgs/0.5.13-0.tar.gz";
        sha256 = "0rssxq4agan2sfgrw7bllkpfl60xwnps4y2j61w8yr639svzqi73";
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
      inherit cmakeFlags postInstall;
    })) {};
    tf2_msgs = callPackage ({ actionlib_msgs, catkin, cmake, geometry_msgs, gtest, message_generation, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "tf2_msgs";
      version = "0.5.13-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry2-release/archive/release/kinetic/tf2_msgs/0.5.13-0.tar.gz";
        sha256 = "13p9v495x593nhn21kfci7fzbaz9qxx5nrck4lwqg28xhdfja8mp";
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
      inherit cmakeFlags postInstall;
    }) {};
    tf2_py = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, rospy, stdenv, tf2 }:
    stdenv.mkDerivation (pyBuild {
      name = "tf2_py";
      version = "0.5.13-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry2-release/archive/release/kinetic/tf2_py/0.5.13-0.tar.gz";
        sha256 = "1d60hf9m5zgpgimm62d0l4zs0gshxificsfh66r4hkm9v19nf3k0";
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
      inherit cmakeFlags postInstall;
    })) {};
    tf2_ros = callPackage ({ actionlib, actionlib_msgs, catkin, cmake, geometry_msgs, gtest, message_filters, pkgconfig, pyEnv, roscpp, rosgraph, rospy, std_msgs, stdenv, tf2, tf2_msgs, tf2_py }:
    stdenv.mkDerivation (pyBuild {
      name = "tf2_ros";
      version = "0.5.13-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/geometry2-release/archive/release/kinetic/tf2_ros/0.5.13-0.tar.gz";
        sha256 = "1bmvb1ycdd8qxjyijhnii4p70i0viwqhhp603r8jrdmbs1ygidaq";
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
      ];
      inherit cmakeFlags postInstall;
    })) {};
    camera_calibration_parsers = callPackage ({ boost, catkin, cmake, gtest, libyamlcpp, pkgconfig, pyEnv, rosconsole, roscpp, roscpp_serialization, sensor_msgs, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "camera_calibration_parsers";
      version = "1.11.11-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_common-release/archive/release/kinetic/camera_calibration_parsers/1.11.11-0.tar.gz";
        sha256 = "0gflfyjq0blapawcnb0i5kd7jpg1lzsf2pxqsijr6bxjjn5mijr0";
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
      inherit cmakeFlags postInstall;
    })) {};
    camera_info_manager = callPackage ({ boost, camera_calibration_parsers, catkin, cmake, gtest, image_transport, pkgconfig, pyEnv, roscpp, roslib, rostest, sensor_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "camera_info_manager";
      version = "1.11.11-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_common-release/archive/release/kinetic/camera_info_manager/1.11.11-0.tar.gz";
        sha256 = "0s17m2wbd6mjj56xrw9zpylk6cjr3arbm5l5gjlncapx7v4zplzf";
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
      inherit cmakeFlags postInstall;
    }) {};
    image_common = callPackage ({ camera_calibration_parsers, camera_info_manager, catkin, cmake, gtest, image_transport, pkgconfig, polled_camera, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "image_common";
      version = "1.11.11-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_common-release/archive/release/kinetic/image_common/1.11.11-0.tar.gz";
        sha256 = "0iqw5ac79r18gmrh69c2901h3jn9kb33fipnf34plp74pqb91610";
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
      inherit cmakeFlags postInstall;
    }) {};
    image_transport = callPackage ({ catkin, cmake, gtest, message_filters, pkgconfig, pluginlib, pyEnv, rosconsole, roscpp, roslib, sensor_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "image_transport";
      version = "1.11.11-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_common-release/archive/release/kinetic/image_transport/1.11.11-0.tar.gz";
        sha256 = "1v2glhjn5xxyhdwg64glbhrfppglifywdzdy6gfc2x5rxvi8hibz";
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
      inherit cmakeFlags postInstall;
    }) {};
    polled_camera = callPackage ({ catkin, cmake, gtest, image_transport, message_generation, pkgconfig, pyEnv, rosconsole, roscpp, sensor_msgs, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "polled_camera";
      version = "1.11.11-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_common-release/archive/release/kinetic/polled_camera/1.11.11-0.tar.gz";
        sha256 = "1va0w6my7ggzj31xjg4bfna5wzj405ajzkqnvnnkg2bnsn9s1azb";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
        image_transport
        message_generation
        rosconsole
        roscpp
        sensor_msgs
        std_msgs
      ];
      inherit cmakeFlags postInstall;
    }) {};
    camera_calibration = callPackage ({ catkin, cmake, cv_bridge, gtest, image_geometry, message_filters, pkgconfig, pyEnv, rospy, sensor_msgs, std_srvs, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "camera_calibration";
      version = "1.12.19-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/kinetic/camera_calibration/1.12.19-0.tar.gz";
        sha256 = "13ym159zl7ibzl1iychvmnkymqli6dw442nxcwwkfi0gw9vq359g";
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
      inherit cmakeFlags postInstall;
    })) {};
    depth_image_proc = callPackage ({ boost, catkin, cmake, cmake_modules, cv_bridge, eigen_conversions, gtest, image_geometry, image_transport, message_filters, nodelet, pkgconfig, pyEnv, sensor_msgs, stdenv, stereo_msgs, tf2, tf2_ros }:
    stdenv.mkDerivation {
      name = "depth_image_proc";
      version = "1.12.19-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/kinetic/depth_image_proc/1.12.19-0.tar.gz";
        sha256 = "0gzd6dcphgdw3mwhyk8g772nhhffz0d04izrak9mjbj2y0pgh276";
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
      inherit cmakeFlags postInstall;
    }) {};
    image_pipeline = callPackage ({ camera_calibration, catkin, cmake, depth_image_proc, gtest, image_proc, image_rotate, image_view, pkgconfig, pyEnv, stdenv, stereo_image_proc }:
    stdenv.mkDerivation {
      name = "image_pipeline";
      version = "1.12.19-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/kinetic/image_pipeline/1.12.19-0.tar.gz";
        sha256 = "0f0ghdcjkai2bqdfzgng0sbgynp49d68b6dxv9mss46zxax6d38x";
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
        image_rotate
        image_view
        stereo_image_proc
      ];
      inherit cmakeFlags postInstall;
    }) {};
    image_proc = callPackage ({ boost, catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_geometry, image_transport, nodelet, pkgconfig, pyEnv, roscpp, sensor_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "image_proc";
      version = "1.12.19-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/kinetic/image_proc/1.12.19-0.tar.gz";
        sha256 = "1s0aivvkv16mpq0520mdir4cm0wb01sp4j5fy5r9biqsdffg1m5y";
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
        roscpp
        sensor_msgs
      ];
      inherit cmakeFlags postInstall;
    }) {};
    image_rotate = callPackage ({ catkin, cmake, cmake_modules, cv_bridge, dynamic_reconfigure, eigen_conversions, geometry_msgs, gtest, image_transport, nodelet, pkgconfig, pyEnv, roscpp, stdenv, tf2, tf2_geometry_msgs, tf2_ros }:
    stdenv.mkDerivation {
      name = "image_rotate";
      version = "1.12.19-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/kinetic/image_rotate/1.12.19-0.tar.gz";
        sha256 = "02qwm36ghfp56vmbxqjn3ardq4ai0n4plxc9lamssmgkzd817gq3";
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
        eigen_conversions
        geometry_msgs
        image_transport
        nodelet
        roscpp
        tf2
        tf2_geometry_msgs
        tf2_ros
      ];
      inherit cmakeFlags postInstall;
    }) {};
    image_view = callPackage ({ camera_calibration_parsers, catkin, cmake, cv_bridge, dynamic_reconfigure, glib, gtest, gtk2, image_transport, message_filters, message_generation, nodelet, pango, pkgconfig, pyEnv, rosconsole, roscpp, sensor_msgs, std_srvs, stdenv, stereo_msgs }:
    stdenv.mkDerivation {
      name = "image_view";
      version = "1.12.19-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/kinetic/image_view/1.12.19-0.tar.gz";
        sha256 = "0qmg5vm71hzlz0rj6hhhrxqvv7b581na47aqy3d62xb8xc0wi1kb";
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
      inherit cmakeFlags postInstall;
      NIX_CFLAGS_COMPILE = "-I${glib.out}/lib/glib-2.0/include -I${gtk2.dev}/include/gtk-2.0 -I${glib.dev}/include/glib-2.0 -I${pango.dev}/include/pango-1.0 -I${gtk2.out}/lib/gtk-2.0/include";
    }) {};
    stereo_image_proc = callPackage ({ catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_geometry, image_proc, image_transport, message_filters, nodelet, pkgconfig, pyEnv, sensor_msgs, stdenv, stereo_msgs }:
    stdenv.mkDerivation {
      name = "stereo_image_proc";
      version = "1.12.19-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_pipeline-release/archive/release/kinetic/stereo_image_proc/1.12.19-0.tar.gz";
        sha256 = "1xjcbaj989nny4na4n93y7hnfznjiasvxwd505iyv1y103ml9d05";
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
      inherit cmakeFlags postInstall;
    }) {};
    compressed_depth_image_transport = callPackage ({ catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_transport, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "compressed_depth_image_transport";
      version = "1.9.3-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_transport_plugins-release/archive/release/kinetic/compressed_depth_image_transport/1.9.3-0.tar.gz";
        sha256 = "0gkbw12dlymbrsn7q32xy1yqimmhz0vn56jl5978lsillvzih9f1";
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
      inherit cmakeFlags postInstall;
    }) {};
    compressed_image_transport = callPackage ({ catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_transport, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "compressed_image_transport";
      version = "1.9.3-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_transport_plugins-release/archive/release/kinetic/compressed_image_transport/1.9.3-0.tar.gz";
        sha256 = "1r69h7mx1kbq9hd3nzxjbd0556frs5gflrgd09421iyq8rzm5rwi";
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
      inherit cmakeFlags postInstall;
    }) {};
    image_transport_plugins = callPackage ({ catkin, cmake, compressed_depth_image_transport, compressed_image_transport, gtest, pkgconfig, pyEnv, stdenv, theora_image_transport }:
    stdenv.mkDerivation {
      name = "image_transport_plugins";
      version = "1.9.3-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_transport_plugins-release/archive/release/kinetic/image_transport_plugins/1.9.3-0.tar.gz";
        sha256 = "164kvwmmkb95is0qxqqfrrlwvji787kmvwfi1dwz04nlfgcf2a79";
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
      inherit cmakeFlags postInstall;
    }) {};
    theora_image_transport = callPackage ({ catkin, cmake, cv_bridge, dynamic_reconfigure, gtest, image_transport, libogg, libtheora, message_generation, message_runtime, pkgconfig, pluginlib, pyEnv, rosbag, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "theora_image_transport";
      version = "1.9.3-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/image_transport_plugins-release/archive/release/kinetic/theora_image_transport/1.9.3-0.tar.gz";
        sha256 = "1f6fsdd0lcck1kn1mpmh2d2h7h115vhjiq2mvzqfg3c12yjv7s73";
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
      inherit cmakeFlags postInstall;
    }) {};
    laser_assembler = callPackage ({ catkin, cmake, filters, gtest, laser_geometry, message_filters, message_generation, message_runtime, pkgconfig, pluginlib, pyEnv, roscpp, rostest, sensor_msgs, stdenv, tf }:
    stdenv.mkDerivation {
      name = "laser_assembler";
      version = "1.7.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/laser_assembler-release/archive/release/kinetic/laser_assembler/1.7.4-0.tar.gz";
        sha256 = "1hykzwsrgdgklnfnczf7zz0xz0lkpii4r0s2vly1gbw2rzvbpsmm";
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
      inherit cmakeFlags postInstall;
    }) {};
    laser_filters = callPackage ({ angles, catkin, cmake, filters, gtest, laser_geometry, message_filters, pkgconfig, pluginlib, pyEnv, roscpp, rostest, sensor_msgs, stdenv, tf }:
    stdenv.mkDerivation {
      name = "laser_filters";
      version = "1.8.3-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/laser_filters-release/archive/release/kinetic/laser_filters/1.8.3-0.tar.gz";
        sha256 = "09m56v5cwk1mv0ip7npzl0zb8bzk7jns357wagj6rkw6b49p9lna";
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
      inherit cmakeFlags postInstall;
    }) {};
    laser_geometry = callPackage ({ angles, boost, catkin, cmake, cmake_modules, eigen, gtest, pkgconfig, pyEnv, roscpp, sensor_msgs, stdenv, tf }:
    stdenv.mkDerivation (pyBuild {
      name = "laser_geometry";
      version = "1.6.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/laser_geometry-release/archive/release/kinetic/laser_geometry/1.6.4-0.tar.gz";
        sha256 = "0d3pd2v88bchppdr2qfj07gc443pky7l17gk4prldk44iwlmv0kg";
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
      inherit cmakeFlags postInstall;
    })) {};
    laser_pipeline = callPackage ({ catkin, cmake, gtest, laser_assembler, laser_filters, laser_geometry, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "laser_pipeline";
      version = "1.6.2-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/laser_pipeline-release/archive/release/kinetic/laser_pipeline/1.6.2-0.tar.gz";
        sha256 = "0rpyq8anl7nryk4mhlpr8shq2d3851i9mni0prnhckz4jz02pkfy";
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
      inherit cmakeFlags postInstall;
    }) {};
    message_generation = callPackage ({ catkin, cmake, gencpp, geneus, genlisp, genmsg, gennodejs, genpy, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    message_runtime = callPackage ({ catkin, cmake, cpp_common, genpy, gtest, pkgconfig, pyEnv, roscpp_serialization, roscpp_traits, rostime, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    perception = callPackage ({ catkin, cmake, gtest, image_common, image_pipeline, image_transport_plugins, laser_pipeline, perception_pcl, pkgconfig, pyEnv, ros_base, stdenv, vision_opencv }:
    stdenv.mkDerivation {
      name = "perception";
      version = "1.3.0-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/metapackages-release/archive/release/kinetic/perception/1.3.0-0.tar.gz";
        sha256 = "12j6530vdibbdnak3bnc66hwrfsf8lrj8vrccpvhk06rni7v9r9x";
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
      inherit cmakeFlags postInstall;
    }) {};
    ros_base = callPackage ({ actionlib, bond_core, catkin, class_loader, cmake, dynamic_reconfigure, gtest, nodelet_core, pkgconfig, pluginlib, pyEnv, ros_core, stdenv }:
    stdenv.mkDerivation {
      name = "ros_base";
      version = "1.3.0-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/metapackages-release/archive/release/kinetic/ros_base/1.3.0-0.tar.gz";
        sha256 = "042h6yrpi4cyawrwni8nzivgdb1l0ss7qijyi67rzg7w1v5dlcmd";
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
      inherit cmakeFlags postInstall;
    }) {};
    ros_core = callPackage ({ catkin, cmake, cmake_modules, common_msgs, gencpp, geneus, genlisp, genmsg, gennodejs, genpy, gtest, message_generation, message_runtime, pkgconfig, pyEnv, ros, ros_comm, rosbag_migration_rule, rosconsole_bridge, roscpp_core, rosgraph_msgs, roslisp, rospack, std_msgs, std_srvs, stdenv }:
    stdenv.mkDerivation {
      name = "ros_core";
      version = "1.3.0-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/metapackages-release/archive/release/kinetic/ros_core/1.3.0-0.tar.gz";
        sha256 = "01immxxk8nijx00s0z7q7idrpqbld06f1gw8mpzgwf1kl8c9w6q6";
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
      inherit cmakeFlags postInstall;
    }) {};
    nodelet = callPackage ({ bondcpp, boost, catkin, cmake, cmake_modules, gtest, message_generation, message_runtime, pkgconfig, pluginlib, pyEnv, rosconsole, roscpp, rospy, std_msgs, stdenv, tinyxml, uuid }:
    stdenv.mkDerivation {
      name = "nodelet";
      version = "1.9.6-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/kinetic/nodelet/1.9.6-0.tar.gz";
        sha256 = "003i533rz5ji8j97yqdjq2xd082138gsxailyiz03h0igr8gxq08";
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
        tinyxml
        uuid
      ];
      inherit cmakeFlags postInstall;
    }) {};
    nodelet_core = callPackage ({ catkin, cmake, gtest, nodelet, nodelet_topic_tools, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "nodelet_core";
      version = "1.9.6-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/kinetic/nodelet_core/1.9.6-0.tar.gz";
        sha256 = "12x7lzbg0dhrs2ylbxzm5z9xfkx30sgdkhk2lj79yw6qkrxdhm62";
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
      inherit cmakeFlags postInstall;
    }) {};
    nodelet_topic_tools = callPackage ({ boost, catkin, cmake, dynamic_reconfigure, gtest, message_filters, nodelet, pkgconfig, pluginlib, pyEnv, roscpp, stdenv }:
    stdenv.mkDerivation {
      name = "nodelet_topic_tools";
      version = "1.9.6-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/nodelet_core-release/archive/release/kinetic/nodelet_topic_tools/1.9.6-0.tar.gz";
        sha256 = "0y2d6ia3s9i4dx2cdrggdj1a03r2afizdi7gr30bb721xb4mxai0";
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
      inherit cmakeFlags postInstall;
    }) {};
    orocos_kdl = callPackage ({ catkin, cmake, eigen, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "orocos_kdl";
      version = "1.3.0-0";
      src = fetchurl {
        url = "https://github.com/smits/orocos-kdl-release/archive/release/kinetic/orocos_kdl/1.3.0-0.tar.gz";
        sha256 = "04dakl1vw5ablpis0wkqslii7hji5hdv9f4x35wdgwz325arwl8z";
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
      inherit cmakeFlags postInstall;
    }) {};
    python_orocos_kdl = callPackage ({ catkin, cmake, gtest, orocos_kdl, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "python_orocos_kdl";
      version = "1.3.0-0";
      src = fetchurl {
        url = "https://github.com/smits/orocos-kdl-release/archive/release/kinetic/python_orocos_kdl/1.3.0-0.tar.gz";
        sha256 = "1ndv0zqf3alliv4jx81p5fwafd67jjb75cdyrzjpgdil1ivg1xpj";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
        orocos_kdl
      ];
      inherit cmakeFlags postInstall;
    }) {};
    pcl_conversions = callPackage ({ catkin, cmake, cmake_modules, gtest, pcl, pcl_msgs, pkgconfig, pyEnv, roscpp, sensor_msgs, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "pcl_conversions";
      version = "0.2.1-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/pcl_conversions-release/archive/release/kinetic/pcl_conversions/0.2.1-0.tar.gz";
        sha256 = "0niyjinbnnipklm9ah0zmwhyggmyrbbxz41r4p5cj71qgm09mrbd";
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
      inherit cmakeFlags postInstall;
    }) {};
    pcl_msgs = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, sensor_msgs, std_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "pcl_msgs";
      version = "0.2.0-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/pcl_msgs-release/archive/release/kinetic/pcl_msgs/0.2.0-0.tar.gz";
        sha256 = "0040q6lyr61nf1r60z40akn4cgnhlg0gzzvr2q315dydxygriswx";
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
      inherit cmakeFlags postInstall;
    }) {};
    pcl_ros = callPackage ({ Cocoa, catkin, cmake, cmake_modules, genmsg, gtest, libobjc, pkgconfig, pyEnv, rosconsole, roslib, stdenv, eigen, pcl, dynamic_reconfigure, nodelet, nodelet_topic_tools, pcl_conversions, tf, tf2_eigen }:
    stdenv.mkDerivation {
      name = "pcl_ros";
      version = "1.4.1-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/perception_pcl-release/archive/release/kinetic/pcl_ros/1.4.1-0.tar.gz";
        sha256 = "04z9mbaxv71wd8pkkiv0j8h77k3vk3lzjqz1mz1b5iy5r9ybirah";
      };
      propagatedBuildInputs = [
        cmake
        dynamic_reconfigure
        pkgconfig
        eigen
        gtest
        pyEnv
        catkin
        cmake_modules
        genmsg
        nodelet
        nodelet_topic_tools
        pcl_conversions
        rosconsole
        roslib
        pcl
        tf
        tf2_eigen
      ] ++ stdenv.lib.optionals stdenv.isDarwin [
        libobjc
        Cocoa
      ];
      inherit cmakeFlags postInstall;
      preConfigure = ''
        sed -i 's/find_package(Eigen3 REQUIRED)//' ./CMakeLists.txt
      '';
    }) {};
    perception_pcl = callPackage ({ catkin, cmake, gtest, pcl_conversions, pcl_msgs, pcl_ros, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "perception_pcl";
      version = "1.4.1-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/perception_pcl-release/archive/release/kinetic/perception_pcl/1.4.1-0.tar.gz";
        sha256 = "0msqwbgcx6f40b5ccpz4x2azvyp8dizz5dd923vdh97vvqxsxvka";
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
      inherit cmakeFlags postInstall;
    }) {};
    pluginlib = callPackage ({ boost, catkin, class_loader, cmake, cmake_modules, gtest, pkgconfig, pyEnv, rosconsole, roslib, stdenv, tinyxml }:
    stdenv.mkDerivation (pyBuild {
      name = "pluginlib";
      version = "1.10.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/pluginlib-release/archive/release/kinetic/pluginlib/1.10.4-0.tar.gz";
        sha256 = "0ps5mmgbiwrc9wq4hlvqk8y3r4j8sj66dznj88bvl8pdr6ylppi3";
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
      inherit cmakeFlags postInstall;
    })) {};
    mk = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, rosbuild, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    ros = callPackage ({ catkin, cmake, gtest, mk, pkgconfig, pyEnv, rosbash, rosboost_cfg, rosbuild, rosclean, roscreate, roslang, roslib, rosmake, rosunit, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    rosbash = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    rosboost_cfg = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
    })) {};
    rosbuild = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    rosclean = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
    })) {};
    roscreate = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
    })) {};
    roslang = callPackage ({ catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    roslib = callPackage ({ boost, catkin, cmake, gtest, pkgconfig, pyEnv, rospack, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
    })) {};
    rosmake = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
    })) {};
    rosunit = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, roslib, stdenv }:
    stdenv.mkDerivation (pyBuild {
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
      inherit cmakeFlags postInstall;
    })) {};
    message_filters = callPackage ({ boost, catkin, cmake, gtest, pkgconfig, pyEnv, rosconsole, roscpp, rostest, rosunit, stdenv, xmlrpcpp }:
    stdenv.mkDerivation (pyBuild {
      name = "message_filters";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/message_filters/1.12.4-0.tar.gz";
        sha256 = "00b4c13zbjbyxdxpc86fn3j1ajgdb5wh1y9qxcypm7md1yw9p9bn";
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
      inherit cmakeFlags postInstall;
    })) {};
    ros_comm = callPackage ({ catkin, cmake, gtest, message_filters, pkgconfig, pyEnv, ros, rosbag, rosconsole, roscpp, rosgraph, rosgraph_msgs, roslaunch, roslisp, rosmaster, rosmsg, rosnode, rosout, rosparam, rospy, rosservice, rostest, rostopic, roswtf, std_srvs, stdenv, topic_tools, xmlrpcpp }:
    stdenv.mkDerivation {
      name = "ros_comm";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/ros_comm/1.12.4-0.tar.gz";
        sha256 = "08p4p314akgy43wp8xgfs444rsd99qpbndmwdgxzfzbq5wkg9y7m";
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
      inherit cmakeFlags postInstall;
    }) {};
    rosbag = callPackage ({ boost, catkin, cmake, cpp_common, genmsg, genpy, gtest, pkgconfig, pyEnv, rosbag_storage, rosconsole, roscpp, roscpp_serialization, roslib, rospy, stdenv, topic_tools, xmlrpcpp }:
    stdenv.mkDerivation (pyBuild {
      name = "rosbag";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosbag/1.12.4-0.tar.gz";
        sha256 = "0vi6i2gzpimn18ljmn7a6vhqh07h0kvvb4505h5k5fd8r3czzv7n";
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
      inherit cmakeFlags postInstall;
    })) {};
    rosbag_storage = callPackage ({ boost, bzip2, catkin, cmake, console-bridge, cpp_common, gtest, pkgconfig, pyEnv, roscpp_serialization, roscpp_traits, roslz4, rostime, stdenv }:
    stdenv.mkDerivation {
      name = "rosbag_storage";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosbag_storage/1.12.4-0.tar.gz";
        sha256 = "1pp5c722w9kw0w70hdrwfvr0gn7jnlqd5mda0fwa52k4kjrlsw59";
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
      inherit cmakeFlags postInstall;
    }) {};
    rosconsole = callPackage ({ apr, boost, catkin, cmake, cpp_common, gtest, log4cxx, pkgconfig, pyEnv, rosbuild, rostime, rosunit, stdenv }:
    stdenv.mkDerivation {
      name = "rosconsole";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosconsole/1.12.4-0.tar.gz";
        sha256 = "0c492ysm3flkfs58jyxf8797g1dd43x7ymkbnf4r9z4qch1a80r2";
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
      inherit cmakeFlags postInstall;
    }) {};
    roscpp = callPackage ({ catkin, cmake, cpp_common, gtest, message_generation, message_runtime, pkgconfig, pyEnv, rosconsole, roscpp_serialization, roscpp_traits, rosgraph_msgs, roslang, rostime, std_msgs, stdenv, xmlrpcpp }:
    stdenv.mkDerivation {
      name = "roscpp";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/roscpp/1.12.4-0.tar.gz";
        sha256 = "1bywr2qxxfglbvym1spd7r8zyh8mffz305dnnxhiiii50msikwsh";
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
      inherit cmakeFlags postInstall;
    }) {};
    rosgraph = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rosgraph";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosgraph/1.12.4-0.tar.gz";
        sha256 = "19b1iw75w5pjrch21davx6mrqs0qv1lc03crckm8irbdilp8cgb7";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
      ];
      inherit cmakeFlags postInstall;
    })) {};
    roslaunch = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, rosclean, rosgraph_msgs, roslib, rosmaster, rosout, rosparam, rosunit, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "roslaunch";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/roslaunch/1.12.4-0.tar.gz";
        sha256 = "0641k8ljk2rvj0y5891lmwkxzqq09r2bm47a2baz9m3hzd47wfrj";
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
      inherit cmakeFlags postInstall;
    })) {};
    roslz4 = callPackage ({ catkin, cmake, gtest, lz4, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "roslz4";
      version = "4-1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/roslz4/1.12.4-0.tar.gz";
        sha256 = "1z7rsbq1jmzj1cjsxng4m4bxira2agkcl36zn0zdf74sslz9gkc8";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
        lz4
      ];
      inherit cmakeFlags postInstall;
    })) {};
    rosmaster = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rosmaster";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosmaster/1.12.4-0.tar.gz";
        sha256 = "1rk9jxx8f6n88cah2q2gpbny0bcrrax1iwd2iyj22qxsa782jsg3";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
        rosgraph
      ];
      inherit cmakeFlags postInstall;
    })) {};
    rosmsg = callPackage ({ catkin, cmake, genmsg, gtest, pkgconfig, pyEnv, rosbag, roslib, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rosmsg";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosmsg/1.12.4-0.tar.gz";
        sha256 = "0vaks2z0r3frhlzas3gm3ysd0a8ydlaxvf350hwgscvjshh1i4cy";
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
      inherit cmakeFlags postInstall;
    })) {};
    rosnode = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, rostest, rostopic, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rosnode";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosnode/1.12.4-0.tar.gz";
        sha256 = "1lfpcn63ay877d8drr6cq8bbzqv9sr4q2pyhswkmmqsf1mi9qzaj";
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
      inherit cmakeFlags postInstall;
    })) {};
    rosout = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, roscpp, rosgraph_msgs, stdenv }:
    stdenv.mkDerivation {
      name = "rosout";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosout/1.12.4-0.tar.gz";
        sha256 = "0gl1zjamfj1b02g6r27k8a13d7pisl5qy9bljkkr88p2iflc30wc";
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
      inherit cmakeFlags postInstall;
    }) {};
    rosparam = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rosparam";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosparam/1.12.4-0.tar.gz";
        sha256 = "0r6f13x3w5wc2ym3k0h58iq8wplhrdfw71zrsgvad7m398vy9xcr";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
        rosgraph
      ];
      inherit cmakeFlags postInstall;
    })) {};
    rospy = callPackage ({ catkin, cmake, genpy, gtest, pkgconfig, pyEnv, roscpp, rosgraph, rosgraph_msgs, roslib, std_msgs, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rospy";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rospy/1.12.4-0.tar.gz";
        sha256 = "0jlpiq3k48g2cv50k28bw6gqll45dd0qjqdgdaajfz05fnl1kvkg";
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
      inherit cmakeFlags postInstall;
    })) {};
    rosservice = callPackage ({ catkin, cmake, genpy, gtest, pkgconfig, pyEnv, rosgraph, roslib, rosmsg, rospy, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rosservice";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rosservice/1.12.4-0.tar.gz";
        sha256 = "089p56an6y13f8fdgyaqgzqx7sily15vwrfmn7vcfb0jq6vv0znc";
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
      inherit cmakeFlags postInstall;
    })) {};
    rostest = callPackage ({ boost, catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph, roslaunch, rosmaster, rospy, rosunit, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rostest";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rostest/1.12.4-0.tar.gz";
        sha256 = "1xwqknpvn066p57l4442zs2csdnajif3xgk00hylcdrmaycpl7cl";
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
      inherit cmakeFlags postInstall;
    })) {};
    rostopic = callPackage ({ catkin, cmake, genpy, gtest, pkgconfig, pyEnv, rosbag, rospy, rostest, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "rostopic";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/rostopic/1.12.4-0.tar.gz";
        sha256 = "18s5wx5b6iarbrm54rlvjdayrmv3bjs1c1s08h7q3i31npzf2ilv";
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
      inherit cmakeFlags postInstall;
    })) {};
    roswtf = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, rosbuild, rosgraph, roslaunch, roslib, rosnode, rosservice, rostest, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "roswtf";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/roswtf/1.12.4-0.tar.gz";
        sha256 = "17cgrwjly1v5fx1qrzz1j9i6fz13f0g42w9wqd8ds11akdyzjv5h";
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
      inherit cmakeFlags postInstall;
    })) {};
    topic_tools = callPackage ({ catkin, cmake, cpp_common, gtest, message_generation, message_runtime, pkgconfig, pyEnv, rosconsole, roscpp, rostest, rostime, rosunit, std_msgs, stdenv, xmlrpcpp }:
    stdenv.mkDerivation (pyBuild {
      name = "topic_tools";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/topic_tools/1.12.4-0.tar.gz";
        sha256 = "05gw7rdacmayn9hdm26r4qvcnjan6qkx93pfvscqdn9hk2sa0grx";
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
      inherit cmakeFlags postInstall;
    })) {};
    xmlrpcpp = callPackage ({ catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "xmlrpcpp";
      version = "1.12.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/ros_comm-release/archive/release/kinetic/xmlrpcpp/1.12.4-0.tar.gz";
        sha256 = "1xscigmj4n2jyj8cfxkcgbjmy0jhd5kkdvmvjwx0ikqmwh8bcdh1";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
        cpp_common
      ];
      inherit cmakeFlags postInstall;
    }) {};
    rosgraph_msgs = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, std_msgs, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    std_srvs = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    rosbag_migration_rule = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "rosbag_migration_rule";
      version = "1.0.0-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/rosbag_migration_rule-release/archive/release/kinetic/rosbag_migration_rule/1.0.0-0.tar.gz";
        sha256 = "1nkxa00d4l8wwaa9dvawbzzzvs0ld97j6vi662mghk7v0g063vcf";
      };
      propagatedBuildInputs = [
        cmake
        pkgconfig
        gtest
        pyEnv
        catkin
      ];
      inherit cmakeFlags postInstall;
    }) {};
    rosconsole_bridge = callPackage ({ catkin, cmake, console-bridge, gtest, pkgconfig, pyEnv, rosconsole, stdenv }:
    stdenv.mkDerivation {
      name = "rosconsole_bridge";
      version = "0.4.4-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/rosconsole_bridge-release/archive/release/kinetic/rosconsole_bridge/0.4.4-0.tar.gz";
        sha256 = "1774qkv0zr0vz58b6ibrimhlj9qvk88wg3wqzamc9whk37ygy1xm";
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
      inherit cmakeFlags postInstall;
    }) {};
    cpp_common = callPackage ({ boost, catkin, cmake, console-bridge, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    roscpp_core = callPackage ({ catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, roscpp_serialization, roscpp_traits, rostime, stdenv }:
    stdenv.mkDerivation {
      name = "roscpp_core";
      version = "0.6.1-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/roscpp_core-release/archive/release/kinetic/roscpp_core/0.6.1-0.tar.gz";
        sha256 = "0fsdl1462vggy6z4v2r3xazs07yxppga0hx4y8nwlq0g7v4w51pz";
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
      inherit cmakeFlags postInstall;
    }) {};
    roscpp_serialization = callPackage ({ catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, roscpp_traits, rostime, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    roscpp_traits = callPackage ({ catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, rostime, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    rostime = callPackage ({ boost, catkin, cmake, cpp_common, gtest, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    roslisp = callPackage ({ catkin, cmake, gtest, pkgconfig, pyEnv, rosgraph_msgs, roslang, rospack, sbcl, std_srvs, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    rospack = callPackage ({ boost, catkin, cmake, cmake_modules, gtest, pkgconfig, pyEnv, stdenv, tinyxml }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    std_msgs = callPackage ({ catkin, cmake, gtest, message_generation, message_runtime, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
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
      inherit cmakeFlags postInstall;
    }) {};
    cv_bridge = callPackage ({ boost, catkin, cmake, gtest, opencv3, pkgconfig, pyEnv, rosconsole, sensor_msgs, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "cv_bridge";
      version = "1.12.2-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/kinetic/cv_bridge/1.12.2-0.tar.gz";
        sha256 = "0z1512g2dzhrnxf8hqmca9rgbadkngwkxvgpk5iqm2yzx0baww1g";
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
      inherit cmakeFlags postInstall;
    })) {};
    image_geometry = callPackage ({ catkin, cmake, gtest, opencv3, pkgconfig, pyEnv, sensor_msgs, stdenv }:
    stdenv.mkDerivation (pyBuild {
      name = "image_geometry";
      version = "1.12.2-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/kinetic/image_geometry/1.12.2-0.tar.gz";
        sha256 = "0p4kskvx2w292v67ann9hqahxnwjxbnrcvdy52hbh9z7757knnij";
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
      inherit cmakeFlags postInstall;
    })) {};
    vision_opencv = callPackage ({ catkin, cmake, cv_bridge, gtest, image_geometry, pkgconfig, pyEnv, stdenv }:
    stdenv.mkDerivation {
      name = "vision_opencv";
      version = "1.12.2-0";
      src = fetchurl {
        url = "https://github.com/ros-gbp/vision_opencv-release/archive/release/kinetic/vision_opencv/1.12.2-0.tar.gz";
        sha256 = "1sbgfc5wv0syn817q4xn1l1biw7pl0698g3bd172imh5sdm0v2gs";
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
      inherit cmakeFlags postInstall;
    }) {};
  };
in if (import <nixpkgs> {}).lib.inNixShell
   then stdenv.mkDerivation {
          name = "rosPackages";
          buildInputs = [
            cmake
            pkgconfig
            glib
          ] ++ stdenv.lib.filter builtins.isAttrs (stdenv.lib.attrValues rosPackageSet);
          src = [];
          shellHook = ''
            export ROS_PACKAGE_PATH=${stdenv.lib.concatStringsSep ":" (stdenv.lib.filter builtins.isAttrs (stdenv.lib.attrValues rosPackageSet))}
            ${stdenv.lib.concatMapStringsSep "\n" rosShellHook (stdenv.lib.filter builtins.isAttrs (stdenv.lib.attrValues rosPackageSet))}
          '';
        }
   else rosPackageSet
