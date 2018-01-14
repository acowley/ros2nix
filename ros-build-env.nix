{ stdenv, callPackage, fetchurl, python27, rosPython
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
    # pushd ..
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
    # popd
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

  # ROS packages (using catkin) that are primarily Python will
  # sometimes have non-zero CMake aspects like defining a CMake module for
  # the use of downstream packages. What we can do is disable the catkin
  # python setup logic, and run a separate configure-build-install
  # sequence driven by cmake *after* building and installing the Python
  # package.
  rosPythonCmakeFixup = attrs: attrs // {
    postInstall = extendAttrString attrs "postInstall" ''
      pushd .
      cmakeConfigurePhase
      make
      make install
      popd
    '';
    patchPhase = extendAttrString attrs "patchPhase" ''
      if [ -f ./CMakeLists.txt ]; then
        sed '/catkin_python_setup()/d' -i ./CMakeLists.txt
      fi
    '';
  };

  # Warning: pulling numpy (or something that depends on it, like
  # matplotlib) into this environment can cause problems down the road. In
  # particular, I had conflicts that were I think due to opencv3's
  # optional use of numpy.
  pyEnv = rosPython.withPackages (ps: with ps; [
      # numpy
      # matplotlib
      setuptools
      sphinx
      #six
      dateutil
      # docutils
      argparse
      pyyaml
      nose
      rosdep
      rospkg
      rosdistro
      rosinstall_generator
      wstool
      rosinstall
      catkin_tools
      catkin_pkg
      bloom
      empy
      pillow
      pydot
      paramiko
      coverage
      netifaces
      mock
      psutil
      vcstools
      defusedxml
      pygraphviz
]);
in rosPackageSet: {
  inherit pyEnv;
  mkRosPythonPackage = attrs:
    rosPython.pkgs.buildPythonPackage
      (rosPythonCmakeFixup (rosBuildHooks attrs));
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
