{-# LANGUAGE FlexibleContexts, OverloadedStrings, QuasiQuotes, TemplateHaskell #-}
{-# OPTIONS_GHC -fdefer-type-errors -Wall #-}
import Control.Lens hiding (argument)
import Control.Logging (debug, errorL, setLogLevel, withStdoutLogging, LogLevel(..))
import Control.Monad (filterM)
import Data.Bool (bool)
import Data.Fix (Fix(Fix))
import Data.List (isSuffixOf)
import Data.Maybe (catMaybes)
import qualified Data.Set as S
import Data.Text (Text)
import qualified Data.Text as T
import Data.Text.Encoding
import Data.Yaml.YamlLight
import Data.Yaml.YamlLight.Lens
import HashCache
import Nix.Pretty (prettyNix)
import Nix.Types
import Options.Applicative
import System.Directory (doesFileExist, getDirectoryContents, doesDirectoryExist)
-- import System.Environment (getArgs)
import System.FilePath ((</>))
import System.IO.Temp (withSystemTempDirectory)
import System.Process (readCreateProcessWithExitCode, proc)
import Text.XML.HXT.Core
-- import Data.Monoid ((<>))
import System.Process (callProcess)
import Data.Maybe (maybeToList)
import RosDep2Nix (rosPyDeps, rosDep2Nix)
import Data.Maybe (mapMaybe)

testFile :: FilePath
testFile = "/Users/acowley/Documents/Projects/Nix/Ros/indigo_perception_ws/src/actionlib/package.xml"

testDistro :: FilePath
testDistro = "/Users/acowley/Documents/Projects/Nix/Ros/indigo_perception_ws/indigo_perception_ws.rosinstall"

-- nix-shell test:
-- cabal run -- /nix/store/v4wjb20r1qysw9n4hdxisbkh0mgws7y3-ros-indigo-perception-src/indigo_perception.rosinstall -o indigo_perception.nix


data RosPackage = RosPackage { _localName :: Text
                             , _uri       :: Text
                             , _version   :: Text }
                  deriving Show
makeClassy ''RosPackage

data Package = Package { rosPackage' :: RosPackage
                       , sha256      :: Text
                       , deps        :: [Text] }
               deriving Show
makeLenses ''Package

instance HasRosPackage Package where
  rosPackage = lens rosPackage' (\(Package _ s d) r -> Package r s d)

versionCleanup :: Text -> Text
versionCleanup = T.dropWhile (not . (`elem` (['0'..'9'] :: [Char])))
               . T.takeWhileEnd (`elem` ("-.0123456789" :: [Char]))

-- | Helper for producing a double-quoted Nix string.
mkStr' :: Text -> NExpr
mkStr' = mkStr DoubleQuoted

-- | Helper to make a Nix name-value binding
nixKeyVal :: Text -> NExpr -> Binding NExpr
nixKeyVal k v = NamedVar (mkSelector k) v

-- | Generate a @fetchurl@ Nix expression.
fetchSrc :: Package -> NExpr
fetchSrc pkg = mkApp (mkSym "fetchurl") $ mkNonRecSet
               [ nixKeyVal "url" (mkStr' (view uri pkg))
               , nixKeyVal "sha256" (mkStr' (sha256 pkg)) ]

-- | Generate a Nix derivation for a 'Package'.
nixify :: Package -> NExpr
nixify pkg = mkFunction args body
  where args = mkFormalSet $ zip ("stdenv" : "makeWrapper" : "wrapPython" : deps')
                                 (repeat Nothing)
        body = mkApp (mkSym "stdenv.mkDerivation") . mkNonRecSet $
               [ nixKeyVal "name" (mkStr' (view localName pkg))
               , nixKeyVal "version" $
                 mkStr' (versionCleanup $ view version pkg)
               , nixKeyVal "src" (fetchSrc pkg)
               -- , nixKeyVal "pythonPath"
               --             (mkList (map mkSym $
               --                      "pyEnv" : mapMaybe rosDep2Nix (deps pkg)))
               , nixKeyVal "nativeBuildInputs" (mkList [mkSym "wrapPython"])
               , nixKeyVal "propagatedBuildInputs"
                           (extraInputs . mkList $ map mkSym deps')
               , nixKeyVal "postInstall" . mkStr Indented $
                 T.unlines [ "if [ -f './package.xml' ]; then"
                           , "  cp package.xml $out"
                           , "fi"
                           , "if [ -d './resources' ]; then"
                           , "  cp -r resources $out"
                           , "fi"
                           , "if [ -d './env-hooks' ]; then"
                           , "  cp -r env-hooks $out"
                           , "fi" ]
               -- , nixKeyVal "postInstall" . mkStr Indented
               --   $ T.unlines [ "if [ -f './setup.py' ]; then "
               --               , "  wrapPythonPrograms"
               --               , "fi" ]
               , Inherit Nothing (map (pure . StaticKey)
                                      [ "cmakeFlags"
                                      -- , "preBuild"
                                      ]) ]
               ++ pkgFixes
        deps' = "cmake" : "pkgconfig" : "gtest" : "pyEnv"
              : mapMaybe rosDep2Nix (deps pkg) ++ extraDeps
        pclDarwinDeps = ["libobjc", "Cocoa"]
        extraDeps = case view localName pkg of
                      "image_view" -> ["glib", "pango"]
                      "pcl_ros" -> pclDarwinDeps
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
                                mkOper2 NConcat
                                        (mkList (filter (not . (`elem` pclDarwinDeps)
                                                         . T.pack . show . prettyNix)
                                                        ds))
                                        (mkApp (mkApp (mkSym "stdenv.lib.optionals") (mkSym "stdenv.isDarwin")) (mkList $ map mkSym pclDarwinDeps))
                              aux _ = error "extraInputs applied to something \
                                            \other than a list"
                          in aux
                        _ -> id
        pkgFixes = case view localName pkg of
                     "catkin" -> [nixKeyVal "patchPhase" (Fix (NStr (NString Indented [ Plain "sed -i 's|#!@PYTHON_EXECUTABLE@|#!", Antiquoted (mkSym "pyEnv.python.passthru.interpreter"), Plain "|' ./cmake/templates/_setup_util.py.in"])))]
                     "image_view" ->
                       [ nixKeyVal "NIX_CFLAGS_COMPILE" (Fix (NStr (NString DoubleQuoted [
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
                       ])))]
                     _ -> []

-- | Prefetch the source for a ROS package into the Nix store
-- recording its hash and list of dependencies.
prefetch :: Maybe HashCache -> RosPackage -> IO Package
prefetch hc pkg = do (_,sha,path) <- readCreateProcessWithExitCode cp ""
                     debug $ "Prefetched " <> view localName pkg
                     -- path is printed on stderr as: path is '/nix/store/....'
                     Package pkg (T.pack (init sha))
                       <$> extractDeps (init (init (drop 9 path)))
  where url = T.unpack (view uri pkg)
        cp = proc "nix-prefetch-url" $
             url : maybeToList (hc >>= fmap T.unpack . cacheLookup (view uri pkg))
        oops path NoPackageXML = errorL $ "Couldn't find package.xml in "<>path
        oops path NotGzip = errorL $ "Couldn't decompress "<>path<>" (not gzip)"
        extractDeps path =  either (oops $ T.pack path) id
                        <$> extractDependencies path

-- | Monadic combinator that returns the first element of a list for
-- which the provided monadic predicate returns 'True'.
firstM :: Monad m => (a -> m Bool) -> [a] -> m (Maybe a)
firstM f = go
  where go [] = return Nothing
        go (x:xs) = f x >>= bool (go xs) (return (Just x))

-- | Failure modes of reading a @package.xml@ file.
data PackageError = NoPackageXML | NotGzip deriving (Eq,Show)

-- | Parse a @package.xml@ file contained within a ROS package's
-- source tarball returning a list of dependencies.
extractDependencies :: String -> IO (Either PackageError [Text])
extractDependencies nixPath
  | not (".gz" `isSuffixOf` nixPath) = return (Left NotGzip)
extractDependencies nixPath =
  withSystemTempDirectory "ros2nix" $ \tmpDir ->
    do callProcess "tar" ["xf", nixPath, "-C", tmpDir]
       dirs <- filter (any (/= '.')) <$> getDirectoryContents tmpDir
               >>= filterM (doesDirectoryExist . (tmpDir </>))
       pkgDir <- firstM (doesFileExist . (</> "package.xml"))
                        (map (tmpDir </>) dirs)
       case pkgDir of
         Nothing -> return (Left NoPackageXML)
         Just d -> Right <$> getDependencies (tmpDir </> d </> "package.xml")

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

-- | Create a Nix attribute set with definitions for each package.
letPackageSet :: [Package] -> NExpr -> NExpr
letPackageSet pkgs =
  mkLet [ callPkg
        , nixKeyVal "SHLIB" $
          mkIf (mkSym "stdenv.isDarwin") (mkStr' "dylib") (mkStr' "so")
        , nixKeyVal "cmakeFlags" cmakeFlags
        -- , nixKeyVal "preBuild" prepEnv
        , nixKeyVal "pyPackages" rosPyPackages
        , nixKeyVal "pyEnv" rosPyEnv
        , NamedVar (mkSelector "rosPackageSet") pkgSet ]
  where callPkg = NamedVar (mkSelector "callPackage") $
                  mkApp (mkSym "stdenv.lib.callPackageWith")
                        pkgSetName
        cmakeFlags = mkList $ map (Fix . NStr . NString DoubleQuoted)
                     [ [ Plain "-DPYTHON_LIBRARY="
                       , Antiquoted (mkSym "pyEnv")
                       , Plain "/lib/libpython2.7."
                       , Antiquoted (mkSym "SHLIB") ]
                     , [ Plain "-DPYTHON_INCLUDE_DIR="
                       , Antiquoted (mkSym "pyEnv")
                       , Plain "/include/python2.7" ]
                     , [ Plain "-DPYTHON_EXECUTABLE="
                       , Antiquoted (mkSym "pyEnv")
                       , Plain "/bin/python" ]
                     , [ Plain "-DEIGEN_ROOT_DIR="
                       , Antiquoted (mkSym "eigen") ]
                     , [ Plain "-DEIGEN3_INCLUDE_DIR="
                       , Antiquoted (mkSym "eigen")
                       , Plain "/include/eigen3" ] ]
        -- prepEnv = Fix . NStr . NString Indented $
        --           [ Plain "HOME=$out\n"
        --           , Plain "export TERM=xterm-256color\n"
        --           , Plain "export PYTHONHOME="
        --           , Antiquoted (mkSym "pyEnv.python")
        --           , Plain "\n" ]
        catkinBuild = mkStr Indented $ T.unlines [
                        "source $stdenv/setup"
                      , "mkdir -p $out"
                      , "HOME=$out"
                      , "export TERM=xterm-256color"
                      , "export PYTHONHOME=${python}"
                      , "catkin config --install --install-space $out"
                      , "catkin build -DCMAKE_BUILD_TYPE=Release -DPYTHON_LIBRARY=${python}/lib/libpython2.7.${SHLIB} -DPYTHON_INCLUDE_DIR=${python}/include/python2.7 -DPYTHON_EXECUTABLE=${python.passthru.interpreter} -DEIGEN_ROOT_DIR=${eigen} -DEIGEN3_INCLUDE_DIR=${eigen}/include/eigen3" ]
        pkgSetName = mkSym "rosPackageSet"
        pkgSet = mkNonRecSet $
                 Inherit Nothing
                         (map (pure . StaticKey)
                              ("stdenv" : "pyEnv" : "glib" : "pango"
                               : "makeWrapper" : "libobjc" : "Cocoa"
                               : externalDeps pkgs))
                 : Inherit (Just (mkSym "pyPackages"))
                           [[StaticKey "wrapPython"]]
                 : (map defPkg pkgs)
        defPkg pkg = NamedVar (mkSelector (pkg ^. localName)) $
                     mkApp (mkApp (mkSym "callPackage") (nixify pkg))
                           (mkNonRecSet [])

-- | Return the list of dependencies that are not among the packages
-- being defined.
externalDeps :: [Package] -> [Text]
externalDeps pkgs = S.toList . S.fromList . mapMaybe rosDep2Nix . S.toList
                  $ S.difference allDeps internalPackages
  where internalPackages = S.fromList $ map (view localName) pkgs
        allDeps = S.fromList $ foldMap deps pkgs

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
          (mkApp (mkApp (mkSym "callPackage")
                        (mkPath False "./ros-python-packages.nix"))
                 (mkNonRecSet [
                    Inherit Nothing [[StaticKey "fetchurl"]]
                  , Inherit (Just (mkSym "pyPackages")) $
                            map (pure . StaticKey)
                                [ "buildPythonPackage"
                                , "setuptools"
                                , "pyyaml"
                                , "dateutil"
                                , "argparse"
                                , "docutils"
                                , "nose" ]]))

rosHelperPackages :: NExpr
rosHelperPackages = mkRecSet [
    call "sip" [ Inherit (Just (mkSym "pyPackages"))
                               [[StaticKey "buildPythonPackage"]]]
  , call "console-bridge" []
  , call "poco" []
  , call "collada-dom" []
  , call "urdfdom-headers" []
  , call "urdfdom" [ Inherit Nothing [ [StaticKey "urdfdom-headers"]
                                     , [StaticKey "console-bridge"] ]]]
  where call n = nixKeyVal n
               . mkApp (mkApp (mkSym "callPackage")
                              (mkPath False ("./"<>T.unpack n<>".nix")))
               . mkNonRecSet

-- | Generate a Nix derivation that depends on all given packages.
mkMetaPackage :: [Package] -> NExpr
mkMetaPackage pkgs = mkFunction args body
  where args = mkFormalSet $
               zip ("stdenv" : "python27" : "python27Packages" : "fetchurl"
                    : "glib" : "pango" : "gdk_pixbuf" : "atk" : "makeWrapper"
                    : "libobjc" : "Cocoa"
                    : externalDeps pkgs)
                   (repeat Nothing)
        body = letPackageSet pkgs . mkApp (mkSym "stdenv.mkDerivation") $
               mkNonRecSet
                 [ nixKeyVal "name" (mkStr' "rosPackages")
                 , nixKeyVal "buildInputs" (deps')
                 , nixKeyVal "src" (mkList [])
                 , nixKeyVal "shellHook" . Fix . NStr $ NString Indented  [
                     Plain "export ROS_PACKAGE_PATH="
                   , Antiquoted (mkApp
                                   (mkApp
                                      (mkSym "stdenv.lib.concatStringsSep")
                                      (mkStr' ":"))
                                   (mkApp
                                      (mkApp
                                         (mkSym "stdenv.lib.filter")
                                         (mkFunction (FormalName "x")
                                                     (mkOper
                                                        NNot
                                                        (mkApp (mkSym "isNull")
                                                               (mkSym "x")))))
                                      (mkApp (mkSym "stdenv.lib.attrValues")
                                             (mkSym "rosPackageSet"))))]]
        deps' = mkOper2 NConcat
                        (mkList $ map mkSym ["cmake", "pkgconfig", "glib"])
                        (mkApp (mkSym "stdenv.lib.attrValues")
                               (mkSym "rosPackageSet"))

data Opts = Opts { _rosinstall :: FilePath
                 , _outFile :: Maybe FilePath }

optParser :: Parser Opts
optParser = Opts
            <$> strArgument (metavar "ROSINSTALL"
                             <> help ".rosinstall file defining a ROS distro")
            <*> (optional $ strOption $
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
                  let nix = show . prettyNix $ mkMetaPackage pkgs
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
