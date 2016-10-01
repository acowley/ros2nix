{-# LANGUAGE FlexibleContexts, OverloadedStrings, QuasiQuotes, TemplateHaskell #-}
import Control.Lens hiding (argument)
import Control.Logging (debug, errorL, setLogLevel, withStdoutLogging, LogLevel(..))
import Control.Monad (filterM)
import Data.Bool (bool)
import Data.Fix (Fix(Fix))
import Data.List (isSuffixOf)
import Data.Maybe (catMaybes, mapMaybe, maybeToList)
import qualified Data.Set as S
import Data.Text (Text)
import qualified Data.Text as T
import Data.Text.Encoding
import Data.Yaml.YamlLight
import Data.Yaml.YamlLight.Lens
import HashCache
import Nix.Pretty (prettyNix)
import Nix.Types
import NixHelpers
import Options.Applicative
import RosDep2Nix (rosPyDeps, rosDep2Nix)
import System.Directory (doesFileExist, getDirectoryContents, doesDirectoryExist)
import System.FilePath ((</>))
import System.IO.Temp (withSystemTempDirectory)
import System.Process (callProcess, readCreateProcessWithExitCode, proc)
import Text.XML.HXT.Core

-- Example usage:
-- stack exec -- ros2nix $(nix-store -r $(nix-instantiate ../Nix/Ros/default.nix -A indigo.ros_core-src))/indigo_ros_core.rosinstall -o indigo_core.nix
-- stack exec ros2nix -- $(nix-build --no-out-link ../RosNix/default.nix -A kinetic.perception-src)/kinetic_perception.rosinstall -o kinetic_perception.nix

data RosPackage = RosPackage { _localName :: Text
                             , _uri       :: Text
                             , _version   :: Text }
                  deriving Show
makeClassy ''RosPackage

data BuildType = CMake | Python deriving (Eq, Ord, Show)

data Package = Package { rosPackage' :: RosPackage
                       , sha256      :: Text
                       , deps        :: [Text]
                       , buildType   :: BuildType }
               deriving Show
-- makeLenses ''Package

instance HasRosPackage Package where
  rosPackage = lens rosPackage' (\(Package _ s d b) r -> Package r s d b)

versionCleanup :: Text -> Text
versionCleanup = T.dropWhile (not . (`elem` (['0'..'9'] :: String)))
               . T.takeWhileEnd (`elem` ("-.0123456789" :: String))

-- | Helper to make a Nix name-value binding
nixKeyVal :: Text -> NExpr -> Binding NExpr
nixKeyVal k = NamedVar (mkSelector k)

-- | Generate a @fetchurl@ Nix expression.
fetchSrc :: Package -> NExpr
fetchSrc pkg = mkApp (mkSym "fetchurl") $ mkNonRecSet
               [ nixKeyVal "url" (mkStr' (view uri pkg))
               , nixKeyVal "sha256" (mkStr' (sha256 pkg)) ]

-- | Generate a Nix derivation for a 'Package'.
nixify :: Package -> NExpr
nixify pkg = mkFunction args body
  where args = mkFormalSet $ zip ("stdenv" : deps') (repeat Nothing)
        mkDeriv = case buildType pkg of
                    CMake -> mkApp (mkSym "stdenv.mkDerivation")
                    Python -> mkApp (mkSym "stdenv.mkDerivation")
                            . mkApp (mkSym "pyBuild")
        body = mkDeriv . mkNonRecSet $
               [ nixKeyVal "name" (mkStr' (view localName pkg))
               , nixKeyVal "version" $
                 mkStr' (versionCleanup $ view version pkg)
               , nixKeyVal "src" (fetchSrc pkg)
               , nixKeyVal "propagatedBuildInputs"
                           (extraInputs . mkList $ map mkSym deps')
               , Inherit Nothing (map (pure . StaticKey) ["cmakeFlags", "postInstall", "postFixup"]) ]
               ++ pkgFixes
        deps' = "cmake" : "pkgconfig" : "gtest" : "pyEnv"
              : mapMaybe rosDep2Nix (deps pkg) ++ extraDeps
        pclDarwinDeps = ["libobjc", "Cocoa"]
        pclRosDeps = ["dynamic_reconfigure"
                     ,"eigen"
                     ,"nodelet"
                     ,"nodelet_topic_tools"
                     ,"pcl_conversions"
                     ,"pcl"
                     ,"tf"
                     ,"tf2_eigen"]
        extraDeps = case view localName pkg of
                       "image_view" -> ["glib", "pango"]
                       "pcl_ros" -> pclDarwinDeps ++ pclRosDeps
                       _ -> []
        extraInputs = case view localName pkg of
                        "image_view" ->
                          let aux (Fix (NList ds)) =
                                mkList $ ds ++ map mkSym [ "gtk2.out"
                                                         , "gtk2.dev"
                                                         , "glib.out"
                                                         , "glib.dev"
                                                         , "pango"]
                              aux _ = error "extraInputs applied to something \
                                            \other than a list"
                          in aux
                        "pcl_ros" ->
                          let aux (Fix (NList ds)) =
                                mkOper2
                                  NConcat
                                  (mkList (filter (not . (`elem` pclDarwinDeps)
                                                   . T.pack . show . prettyNix)
                                                  ds
                                           ++ map mkSym pclRosDeps))
                                        (mkApp2 (mkSym "stdenv.lib.optionals")
                                                (mkSym "stdenv.isDarwin")
                                                (mkList $ map mkSym pclDarwinDeps))
                              aux _ = error "extraInputs applied to something \
                                            \other than a list"
                          in aux
                        _ -> id
        pkgFixes =
          case view localName pkg of
            "catkin" ->
              [ nixKeyVal "patchPhase" $
                mkIndented [ Plain "sed -i 's|#!@PYTHON_EXECUTABLE@|#!"
                           , Antiquoted (mkSym "pyEnv.python.passthru.interpreter")
                           , Plain "|' ./cmake/templates/_setup_util.py.in\n"
                           , Plain "sed -i 's/PYTHON_EXECUTABLE/SHELL/' ./cmake/catkin_package_xml.cmake\n"
                           , Plain "sed -i 's|#!/usr/bin/env bash|#!"
                           , Antiquoted (mkSym "stdenv.shell")
                           , Plain "|' ./cmake/templates/setup.bash.in\n"
                           , Plain "sed -i 's|#!/usr/bin/env sh|#!"
                           , Antiquoted (mkSym "stdenv.shell")
                           , Plain "|' ./cmake/templates/setup.sh.in\n"
                           ]]
            "genmsg" ->
              [ nixKeyVal "patchPhase" $ mkStr Indented
                "sed -i 's/${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT}/${GENMSG_CHECK_DEPS_SCRIPT}/' ./cmake/pkg-genmsg.cmake.em\n" ]
            "genpy" ->
              [ nixKeyVal "patchPhase" $ mkStr Indented
                "sed -i 's/${PYTHON_EXECUTABLE} //' ./cmake/genpy-extras.cmake.em" ]
            "genlisp" ->
              [ nixKeyVal "patchPhase" $ mkStr Indented
                "sed -i 's/${PYTHON_EXECUTABLE} //' ./cmake/genlisp-extras.cmake.em" ]
            "gencpp" ->
              [ nixKeyVal "patchPhase" $ mkStr Indented
                "sed -i 's/${PYTHON_EXECUTABLE} //' ./cmake/gencpp-extras.cmake.em" ]
            "pcl_ros" ->
              [ nixKeyVal "preConfigure" $ mkStr Indented
                "sed -i 's/find_package(Eigen3 REQUIRED)//' ./CMakeLists.txt" ]
            "geneus" ->
              [ nixKeyVal "preConfigure" $ mkStr Indented
                "sed -i 's/COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE}/COMMAND ${CATKIN_ENV}/' ./cmake/geneus-extras.cmake.em" ]
            "image_view" ->
              [ nixKeyVal "NIX_CFLAGS_COMPILE"
                $ mkDoubleQ [
                     Plain "-I"
                   , Antiquoted (mkSym "glib.out")
                   , Plain "/lib/glib-2.0/include -I"
                   , Antiquoted (mkSym "gtk2.dev")
                   , Plain "/include/gtk-2.0 -I"
                   , Antiquoted (mkSym "glib.dev")
                   , Plain "/include/glib-2.0 -I"
                   , Antiquoted (mkSym "pango.dev")
                   , Plain "/include/pango-1.0 -I"
                   , Antiquoted (mkSym "gtk2.out")
                   , Plain "/lib/gtk-2.0/include"
                 ]]
            _ -> []

-- | Prefetch the source for a ROS package into the Nix store
-- recording its hash and list of dependencies.
prefetch :: Maybe HashCache -> RosPackage -> IO Package
prefetch hc pkg = do (_,sha,path) <- readCreateProcessWithExitCode cp ""
                     debug $ "Prefetched " <> view localName pkg
                     -- path is printed on stderr as: path is '/nix/store/....'
                     addInfo (T.pack (init sha))
                       <$> extractInfo (init (init (drop 9 path)))
  where url = T.unpack (view uri pkg)
        cp = proc "nix-prefetch-url" $
             url : maybeToList (hc >>= fmap T.unpack . cacheLookup (view uri pkg))
        oops path NoPackageXML = errorL $ "Couldn't find package.xml in "<>path
        oops path NotGzip = errorL $ "Couldn't decompress "<>path<>" (not gzip)"
        addInfo sha (PackageInfo d b) = Package pkg sha d b
        extractInfo path =  either (oops $ T.pack path) id
                        <$> extractPackageInfo path

-- | Monadic combinator that returns the first element of a list for
-- which the provided monadic predicate returns 'True'.
firstM :: Monad m => (a -> m Bool) -> [a] -> m (Maybe a)
firstM f = go
  where go [] = return Nothing
        go (x:xs) = f x >>= bool (go xs) (return (Just x))

-- | Failure modes of reading a @package.xml@ file.
data PackageError = NoPackageXML | NotGzip deriving (Eq,Show)

-- | Information gleaned from package contents.
data PackageInfo = PackageInfo { _packageDeps      :: [Text]
                               , _packageBuildType :: BuildType }

-- | Parse a @package.xml@ file contained within a ROS package's
-- source tarball returning a list of dependencies.
extractPackageInfo :: String -> IO (Either PackageError PackageInfo)
extractPackageInfo nixPath
  | not (".gz" `isSuffixOf` nixPath) = return (Left NotGzip)
extractPackageInfo nixPath =
  withSystemTempDirectory "ros2nix" $ \tmpDir ->
    do callProcess "tar" ["xf", nixPath, "-C", tmpDir]
       dirs <- filter (any (/= '.')) <$> getDirectoryContents tmpDir
               >>= filterM (doesDirectoryExist . (tmpDir </>))
       pkgDir <- firstM (doesFileExist . (</> "package.xml"))
                        (map (tmpDir </>) dirs)
       case pkgDir of
         Nothing -> return (Left NoPackageXML)
         Just d -> Right
                   <$> (PackageInfo
                         <$> getDependencies (tmpDir </> d </> "package.xml")
                         <*> getBuildType (tmpDir </> d))

-- | Remove ROS stack name components from a ROS package path.
nixName :: Text -> Text
nixName = T.takeWhileEnd (/= '/')

-- | Parse a @.rosinstall@ file extracting package information
getPackages :: FilePath -> IO [RosPackage]
getPackages f = aux <$> parseYamlFile f
  where aux y = catMaybes $ y ^.. each . key "tar" . to pkg
        pkg :: YamlLight -> Maybe RosPackage
        pkg y = RosPackage
              <$> y ^? key "local-name" . _Yaml . to (nixName . decodeUtf8)
              <*> y ^? key "uri" . _Yaml . to decodeUtf8
              <*> y ^? key "version" . _Yaml . to decodeUtf8

-- | @package.xml@ parse helper.
parseDependencies :: String -> IOSLA (XIOState s) a String
parseDependencies f = readDocument [] f
                      >>> getChildren
                      >>> hasName "package"
                      >>> getChildren
                      >>> hasName "buildtool_depend"
                          <+> hasName "build_depend"
                          <+> hasName "run_depend"
                      >>> getChildren
                      >>> getText

-- | Helper to parse a @package.xml@ file.
getDependencies :: FilePath -> IO [Text]
getDependencies = fmap (map T.pack . S.toList . S.fromList)
                . runX . parseDependencies

getBuildType :: FilePath -> IO BuildType
getBuildType = fmap (bool CMake Python) . doesFileExist . (</> "setup.py")

-- | Create a Nix attribute set with definitions for each package.
letPackageSet :: [Package] -> NExpr -> NExpr
letPackageSet pkgs =
  mkLet [ callPkg
        , nixKeyVal "SHLIB" $
          mkIf (mkSym "stdenv.isDarwin") (mkStr' "dylib") (mkStr' "so")
        , nixKeyVal "cmakeFlags" cmakeFlags
        , nixKeyVal "postInstall" . mkStr Indented
          $ T.unlines [ "pushd .."
                      , "if [ -f 'package.xml' ]; then"
                      , "  cp package.xml $out"
                      , "fi"
                      , "if [ -d 'resources' ]; then"
                      , "  cp -r resources $out"
                      , "fi"
                      , "if [ -d 'env-hooks' ]; then"
                      , "  cp -r env-hooks $out"
                      , "fi"
                      , "popd" ]
        , nixKeyVal "postFixup" . mkStr Indented
          $ T.unlines [ "find \"$prefix\" -type f -perm -0100 | while read f; do"
                      , "  if [ \"$(head -1 \"$f\" | head -c+2)\" != '#!' ]; then"
                      , "    # missing shebang => not a script"
                      , "    continue"
                      , "  fi\n"
                      , "  sed -i 's|#!\\(/nix/store/.*/python\\)|#!/usr/bin/env \\1|' \"$f\""
                      , "done" ]
        -- , nixKeyVal "preBuild" prepEnv
        , nixKeyVal "rosShellHook" rosShellHook
        , nixKeyVal "pyCallPackage" pyCallPkg
        , nixKeyVal "pyBuild" pyBuild
        , nixKeyVal "pyPackages" rosPyPackages
        , nixKeyVal "pyEnv" rosPyEnv
        , NamedVar (mkSelector "rosPackageSet") pkgSet ]
  where callPkg = NamedVar (mkSelector "callPackage") $
                  mkApp (mkSym "stdenv.lib.callPackageWith")
                        pkgSetName
        pyCallPkg = mkApp (mkSym "stdenv.lib.callPackageWith")
                          (mkNonRecSet
                             [ Inherit (Just (mkSym "pyPackages"))
                                       (map (pure . StaticKey)
                                            ["python", "setuptools", "wrapPython"])
                             , Inherit (Just (mkSym "stdenv")) [[StaticKey "lib"]]
                             , Inherit Nothing
                                       (map (pure . StaticKey)
                                            [ "ensureNewerSourcesHook", "stdenv"
                                            , "fetchurl", "makeWrapper", "unzip"])
                             , nixKeyVal "callPackage" (mkSym "pyCallPackage")])
        -- pyBuild = mkSym "pyPackages.buildPythonPackage"
        pyBuild = mkApp2 (mkSym "pyCallPackage")
                         (mkPath False "./python-install.nix")
                         (mkNonRecSet [])
        rosShellHook = mkFunction
                         (FormalName "pkg")
                         (mkIndented
                            [ Plain "if [ -d \""
                            , Antiquoted (mkSym "pkg")
                            , Plain "/env-hooks\" ]; then\n"
                            , Plain "  for i in $(find \""
                            , Antiquoted (mkSym "pkg")
                            , Plain "/env-hooks\" -name \"*.sh\"); do\n"
                            , Plain "    source \"$i\"\n"
                            , Plain "  done\n"
                            , Plain "fi\n" ])
        cmakeFlags = mkList $ map mkDoubleQ
                     ([ [ Plain "-DPYTHON_LIBRARY="
                        , Antiquoted (mkSym "pyEnv")
                        , Plain "/lib/libpython2.7."
                        , Antiquoted (mkSym "SHLIB") ]
                     , [ Plain "-DPYTHON_INCLUDE_DIR="
                       , Antiquoted (mkSym "pyEnv")
                       , Plain "/include/python2.7" ]
                     , [ Plain "-DPYTHON_EXECUTABLE="
                       , Antiquoted (mkSym "pyEnv")
                       , Plain "/bin/python" ]] ++
                     if "eigen" `elem` edeps
                     then [ [ Plain "-DEIGEN_ROOT_DIR="
                            , Antiquoted (mkSym "eigen") ]
                          , [ Plain "-DEIGEN3_INCLUDE_DIR="
                            , Antiquoted (mkSym "eigen")
                            , Plain "/include/eigen3" ]
                          , [ Plain "-DEigen3_INCLUDE_DIRS="
                            , Antiquoted (mkSym "eigen")
                            , Plain "/include/eigen3" ] ]
                     else [])
        pkgSetName = mkSym "rosPackageSet"
        edeps = externalDeps pkgs
        pkgSet = mkNonRecSet $
                 Inherit Nothing
                         (map (pure . StaticKey)
                              ("stdenv" : "pyEnv" : "glib" : "pango" : "cmake"
                               : "libobjc" : "Cocoa" : edeps))
                 : Inherit (Just (mkSym "pyPackages"))
                           [[StaticKey "buildPythonPackage"]]
                 : map defPkg pkgs
        defPkg pkg = NamedVar (mkSelector (pkg ^. localName)) $
                     mkApp2 (mkSym "callPackage") (nixify pkg) (mkNonRecSet [])

-- | Return the list of dependencies that are not among the packages
-- being defined.
externalDeps :: [Package] -> [Text]
externalDeps pkgs = nub' . mapMaybe rosDep2Nix . S.toList
                  $ S.difference allDeps internalPackages
  where internalPackages = S.fromList $ map (view localName) pkgs
        allDeps = S.fromList $ foldMap deps pkgs
        nub' = S.toList . S.fromList

rosPyEnv :: NExpr
rosPyEnv = mkApp (mkSym "python27.buildEnv.override")
                 (mkNonRecSet [nixKeyVal "extraLibs" pyPkgs])
  where pyPkgs = mkWith (mkSym "pyPackages")
                        (mkList $ map mkSym rosPyDeps)

rosPyPackages :: NExpr
rosPyPackages =
  mkOper2 NUpdate
          (mkApp (mkSym "python27Packages.override")
                 (mkFunction (FormalName "_")
                             (mkNonRecSet [nixKeyVal "python"
                                                     (mkSym "pyEnv.python")])))
          (mkApp2 (mkSym "callPackage")
                  (mkPath False "./ros-python-packages.nix")
                  (mkNonRecSet [
                     Inherit Nothing [[StaticKey "fetchurl"]]
                   , -- Inherit (Just (mkSym "pyPackages")) $
                     --         map (pure . StaticKey)
                     --             [ "buildPythonPackage"
                     --             , "setuptools"
                     --             , "pyyaml"
                     --             , "dateutil"
                     --             , "argparse"
                     --             , "docutils"
                     --             , "nose" ]
                     Inherit (Just (mkSym "pyPackages")) [[StaticKey "buildPythonPackage"]]
                  ,  nixKeyVal "extradeps"
                               (mkNonRecSet
                                  [Inherit (Just (mkSym "pyPackages"))
                                           [[StaticKey "setuptools"]]])
                   ]))

-- rosHelperPackages :: NExpr
-- rosHelperPackages = mkRecSet [
--     call "sip" [ Inherit (Just (mkSym "pyPackages"))
--                                [[StaticKey "buildPythonPackage"]]]
--   , call "console-bridge" []
--   , call "poco" []
--   , call "collada-dom" []
--   , call "urdfdom-headers" []
--   , call "urdfdom" [ Inherit Nothing [ [StaticKey "urdfdom-headers"]
--                                      , [StaticKey "console-bridge"] ]]]
--   where call n = nixKeyVal n
--                . mkApp2 (mkSym "callPackage")
--                         (mkPath False ("./"<>T.unpack n<>".nix"))
--                . mkNonRecSet

-- | Generate a Nix derivation that depends on all given packages.
mkMetaPackage :: [Package] -> NExpr
mkMetaPackage pkgs = mkFunction args body
  where args = mkFormalSet $
               zip ("stdenv" : "python27" : "python27Packages" : "fetchurl"
                    : "glib" : "pango" : "gdk_pixbuf" : "atk" : "makeWrapper"
                    : "unzip" : "ensureNewerSourcesHook" : "libobjc" : "Cocoa"
                    : "cmake" : "opencv3" : externalDeps pkgs)
                   (repeat Nothing)
        body = letPackageSet pkgs . mkApp (mkSym "stdenv.mkDerivation") $
               mkNonRecSet
                 [ nixKeyVal "name" (mkStr' "rosPackages")
                 , nixKeyVal "buildInputs" deps'
                 , nixKeyVal "src" (mkList [])
                 , nixKeyVal "shellHook" $ mkIndented
                   [ Plain "export ROS_PACKAGE_PATH="
                   , Antiquoted (mkApp2
                                   (mkSym "stdenv.lib.concatStringsSep")
                                   (mkStr' ":")
                                   (mkApp2
                                      (mkSym "stdenv.lib.filter")
                                      (mkSym "builtins.isAttrs")
                                      (mkApp (mkSym "stdenv.lib.attrValues")
                                             (mkSym "rosPackageSet"))))
                   , Plain "\n"
                   , Antiquoted (mkApp3 (mkSym "stdenv.lib.concatMapStringsSep")
                                        (mkStr' "\n")
                                        (mkSym "rosShellHook")
                                        (mkApp2
                                           (mkSym "stdenv.lib.filter")
                                           (mkSym "builtins.isAttrs")
                                           (mkApp (mkSym "stdenv.lib.attrValues")
                                                  (mkSym "rosPackageSet")))) ]]
        deps' = mkOper2 NConcat
                        (mkList $ map mkSym ["cmake", "pkgconfig", "glib"])
                        (mkApp2
                           (mkSym "stdenv.lib.filter")
                           (mkSym "builtins.isAttrs")
                           (mkApp (mkSym "stdenv.lib.attrValues")
                                  (mkSym "rosPackageSet")))

data Opts = Opts { _rosinstall :: FilePath
                 , _outFile :: Maybe FilePath }

optParser :: Parser Opts
optParser = Opts
            <$> strArgument (metavar "ROSINSTALL"
                             <> help ".rosinstall file defining a ROS distro")
            <*> optional (strOption $
                 long "output" <> short 'o' <> metavar "OUTFILE"
                 <> help "Write generated Nix expression to OUTFILE")

main :: IO ()
main = withStdoutLogging $
       do -- args <- getArgs
          Opts f out <- execParser opts
          cache <- loadCache "perception_hash_cache.txt"
          maybe (putStrLn "No hash cache available")
                (const $ putStrLn "Using hash cache")
                cache
          fexists <- doesFileExist f
          setLogLevel LevelError
          if fexists
          then do pkgs <- getPackages f >>= mapM (prefetch cache)
                  let pkgs' = filter (\p -> p^.localName /= "opencv3") pkgs
                      nix = show . prettyNix $ mkMetaPackage pkgs'
                  maybe print writeFile out nix
          else putStrLn $ "Couldn't find .rosinstall file: " ++ f

          -- case args of
          --   [f] -> do fexists <- doesFileExist f
          --             setLogLevel LevelError
          --             if fexists
          --             then do pkgs <- getPackages f >>= mapM (prefetch cache)
          --                     print (prettyNix $ mkMetaPackage pkgs)
          --             else putStrLn $ "Couldn't find .rosinstall file: " ++ f
          --   _ -> putStrLn $ "Usage: ros2nix distro.rosinstall"
  where opts = info (helper <*> optParser)
                    (fullDesc
                     <> header "Generate a Nix expression for a ROS distro")
