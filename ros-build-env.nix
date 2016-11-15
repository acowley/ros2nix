{ stdenv, callPackage, fetchurl, python27, python27Packages
, ensureNewerSourcesHook, makeWrapper, unzip
}:
let
  SHLIB = if stdenv.isDarwin then "dylib" else "so";
  cmakeFlags = [
    "-DPYTHON_LIBRARY=${pyEnv}/lib/libpython2.7.${SHLIB}"
    "-DPYTHON_INCLUDE_DIR=${pyEnv}/include/python2.7"
    "-DPYTHON_EXECUTABLE=${pyEnv}/bin/python"
  ];
  postInstall = ''
    pushd ..
    if [ -f 'package.xml' ]; then
      cp package.xml ''$out
    fi
    if [ -d 'resources' ]; then
      cp -r resources ''$out
    fi
    if [ -d 'images' ]; then
      cp -r images ''$out
    fi
    if [ -d 'env-hooks' ]; then
      cp -r env-hooks ''$out
    fi
    popd
  '';
  postFixup = ''
    find "''$prefix" -type f -perm -0100 | while read f; do
      if [ "''$(head -1 "''$f" | head -c+2)" != '#!' ]; then
        # missing shebang => not a script
        continue
      fi
      sed -i 's|#!\(/nix/store/.*/python\)|#!/usr/bin/env \1|' "''$f"
    done
  '';
  rosShellHook = pkg: ''
    if [ -d "${pkg}/env-hooks" ]; then
      for i in ''$(find "${pkg}/env-hooks" -name "*.sh"); do
        source "''$i"
      done
    fi
  '';
  extendAttrString = attrs: k: v: if attrs ? ${k} then attrs.${k} + v else v;
  extendAttrList = attrs: k: v: if attrs ? ${k} then attrs.${k} ++ v else v;
  rosBuildHooks = attrs: attrs // { 
    cmakeFlags = extendAttrList attrs "cmakeFlags" cmakeFlags;
    postInstall = extendAttrString attrs "postInstall" postInstall;
    postFixup = extendAttrString attrs "postFixup" postFixup;
  };
# inherit cmakeFlags postInstall postFixup; };
  pyEnv = python27.buildEnv.override {
    extraLibs = with pyPackages; [
      numpy
      setuptools
      sphinx
      #six
      dateutil
      docutils
      argparse
      pyyaml
      nose
      rosdep
      rospkg
      rosdistro
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
      vcstools
      #pyside
      defusedxml
      (pygraphviz.override { doCheck = false; })
      #(callPackage ./sip.nix { inherit fetchurl python buildPythonPackage; })
    ];
  };
  pyPackages = python27Packages.override (_: {
    python = pyEnv.python;
  }) // callPackage ./ros-python-packages.nix {
    inherit fetchurl;
    inherit (pyPackages) buildPythonPackage;
    extradeps = { inherit (pyPackages) setuptools; };
  };
  pyCallPackage = stdenv.lib.callPackageWith {
    inherit (pyPackages) python setuptools wrapPython;
    inherit (stdenv) lib;
    inherit ensureNewerSourcesHook stdenv fetchurl makeWrapper unzip;
    callPackage = pyCallPackage;
  };
  pyBuild = pyCallPackage ./python-install.nix {};
in rosPackageSet: {
  inherit pyEnv;
  mkRosPythonPackage = attrs:
    stdenv.mkDerivation (pyBuild (rosBuildHooks attrs));
  mkRosCmakePackage = attrs: stdenv.mkDerivation (rosBuildHooks attrs);
  rosShell = ''
    export ROS_MASTER_URI="http://localhost:11311"
    export ROS_PACKAGE_PATH=${with stdenv.lib;
      concatStringsSep ":" (filter builtins.isAttrs (attrValues rosPackageSet))}
    export PYTHONPATH=${with stdenv.lib;
      concatMapStringsSep ":"
        (d: d + "/lib/python2.7/site-packages")
        (attrValues rosPackageSet)}
    ${with stdenv.lib;
      concatMapStringsSep "\n" rosShellHook (attrValues rosPackageSet)}
  '';
}
