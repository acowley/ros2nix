/* Adapted from the nixpkgs generic Python package builder to piggy-back on an
   existing builder. */
{ python, setuptools
, wrapPython, callPackage, lib, ensureNewerSourcesHook }:
let
  setuppy = <nixpkgs/pkgs/development/python-modules/generic/run_setup.py>;
  bootstrapped-pip = callPackage (<nixpkgs/pkgs/development/python-modules/bootstrapped-pip>) { };
in
attrs:
attrs // {
  buildInputs = [ wrapPython bootstrapped-pip python setuptools
                  (ensureNewerSourcesHook { year = "1980"; })]
                ++ attrs.buildInputs or [];

  preConfigure = ''
    export DETERMINISTIC_BUILD=1
  '' + attrs.preConfigure or "";

  preBuild = ''
    pushd ..
    cp ${setuppy} nix_run_setup.py
    ${python.interpreter} nix_run_setup.py bdist_wheel
    popd
  '' + attrs.preBuild or "";

  installPhase = ''
    runHook preInstall
    mkdir -p "$out/${python.sitePackages}"

    pushd ../dist
    export PYTHONPATH="$out/${python.sitePackages}:$PYTHONPATH"
    ${bootstrapped-pip}/bin/pip install *.whl --no-index --prefix=$out --no-cache
    popd
    make install
    runHook postInstall
  '' + attrs.installPhase or "";

    # ${python.interpreter} ${<nixpkgs> + /pkgs/development/python-modules/generic/catch_conflicts.py}
  postFixup = ''
    find $out -name "*.in" -exec chmod u-x {} +
    wrapPythonPrograms
    find $out -name "*.in" -exec chmod u+x {} +
  '' + attrs.postFixup or "";
  
  shellHook = attrs.shellHook or ''
    if test -e setup.py; then
       tmp_path=$(mktemp -d)
       export PATH="$tmp_path/bin:$PATH"
       export PYTHONPATH="$tmp_path/${python.sitePackages}:$PYTHONPATH"
       mkdir -p $tmp_path/${python.sitePackages}
       ${bootstrapped-pip}/bin/pip install -e . --prefix $tmp_path
    fi
  '';
}
