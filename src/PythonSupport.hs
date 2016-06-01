{-# LANGUAGE OverloadedStrings #-}
-- | Extractions from the nixpkgs generic Python builder
-- @buildPythonPackage@.
module PythonSupport where
import Nix.Types
import NixHelpers

pyPostBuild :: NExpr
pyPostBuild = mkIndented [
               Plain "cp "
               , Antiquoted (mkSym "setuppy")
               , Plain " nix_run_setup.py\n"
               , Antiquoted (mkSym "pyEnv.python.interpreter")
               , Plain " nix_run_setup.py "
               , Antiquoted (mkApp2 (mkSym "lib.optionalString")
                                    (mkOper2 NNEq
                                             (mkSym "setupPyBuildFlags")
                                             (mkList []))
                                    (mkOper2 NConcat
                                             (mkStr' "build_ext ")
                                             (mkApp2 (mkSym "lib.concatStringsSep")
                                                     (mkStr' " ")
                                                     (mkSym "setupPyBuildFlags"))))
               , Plain " bdist_wheel" ]

pyPreConfigure :: NExpr
pyPreConfigure = mkStr Indented "export DETERMINISTIC_BUILD=1"

pyInstall :: NExpr
pyInstall = mkIndented [
             Plain "mkdir -p \"$out/"
             , Antiquoted (mkSym "pyEnv.python.sitePackages")
             , Plain "\nexport PYTHONPATH=\"$out/"
             , Antiquoted (mkSym "pyEnv.python.sitePackages")
             , Plain ":$PYTHONPATH\n"
             , Plain "pushd dist\n"
             , Antiquoted (mkSym "bootstrapped-pip")
             , Plain "/bin/pip install *.whl --no-index --prefix=$out --no-cache "
             , Antiquoted (mkApp (mkSym "toString") (mkSym "installFlags"))
             , Plain "\npopd\n" ]
            
pyPostFixup :: NExpr
pyPostFixup = mkIndented [
               Plain "wrapPythonPrograms\n"
               , Antiquoted (mkSym "pyEnv.python.interpreter")
               , Plain " "
               , Antiquoted (mkPath False "./catch_conflicts.py") ]
            
pyShellHook :: NExpr
pyShellHook = mkIndented [
               Plain "if test -e setup.py; then\n"
               , Plain "  tmp_path=$(mktemp -d)\n"
               , Plain "  export PATH=\"$tmp_path/bin:$PATH\"\n"
               , Plain "  export PYTHONPATH=\"$tmp_path/"
               , Antiquoted (mkSym "pyEnv.python.sitePackages")
               , Plain ":$PYTHONPATH\n"
               , Plain "  mkdir -p $tmp_path/"
               , Antiquoted (mkSym "pyEnv.python.sitePackages")
               , Plain "\n  "
               , Antiquoted (mkSym "bootstrapped-pip")
               , Plain "/bin/pip install -e . --prefix $tmp_path\n"
               , Plain "fi" ]
