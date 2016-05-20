{-# LANGUAGE FlexibleContexts, OverloadedStrings, QuasiQuotes, TemplateHaskell #-}
{-# OPTIONS_GHC -fdefer-type-errors -Wall #-}
import Control.Lens hiding (argument)
import Control.Logging (debug, errorL, setLogLevel, withStdoutLogging, LogLevel(..))
import Control.Monad (filterM)
import Data.Bool (bool)
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
  where args = mkFormalSet $ zip ("stdenv" : deps pkg) (repeat Nothing)
        body = mkApp (mkSym "stdenv.mkDerivation") $ mkNonRecSet
               [ nixKeyVal "name" (mkStr' (view localName pkg))
               , nixKeyVal "version" (mkStr' (view version pkg))
               , nixKeyVal "src" (fetchSrc pkg)
               , nixKeyVal "buildInputs" (mkList deps') ]
        deps' = map mkSym $ "cmake" : "pkgconfig" : deps pkg

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
        , nixKeyVal "pyPackages" rosPyPackages
        , nixKeyVal "pyEnv" rosPyEnv
        , NamedVar (mkSelector "rosPackageSet") pkgSet ]
  where callPkg = NamedVar (mkSelector "callPackage") $
                  mkApp (mkSym "stdenv.lib.callPackageWith")
                        pkgSetName
        catkinBuild = mkStr Indented $ T.unlines [
                        "source $stdenv/setup"
                      , "mkdir -p $out"
                      , "HOME=$out"
                      , "export TERM=xterm-256color"
                      , "export PYTHONHOME=${python}"
                      , "catkin config --install --install-space $out"
                      , "catkin build -DCMAKE_BUILD_TYPE=Release -DPYTHON_LIBRARY=${python}/lib/libpython2.7.${SHLIB} -DPYTHON_INCLUDE_DIR=${python}/include/python2.7 -DPYTHON_EXECUTABLE=${python.passthru.interpreter} -DEIGEN_ROOT_DIR=${eigen} -DEIGEN3_INCLUDE_DIR=${eigen}/include/eigen3" ]
        pkgSetName = mkSym "rosPackageSet"
        pkgSet = mkNonRecSet (map defPkg pkgs)
        defPkg pkg = NamedVar (mkSelector (pkg ^. localName)) $
                     mkApp (mkSym "callPackage") (nixify pkg)

-- | Return the list of dependencies that are not among the packages
-- being defined.
externalDeps :: [Package] -> [Text]
externalDeps pkgs = mapMaybe rosDep2Nix . S.toList
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
                    : externalDeps pkgs)
                   (repeat Nothing)
        body = letPackageSet pkgs . mkApp (mkSym "stdenv.mkDerivation") $
               mkNonRecSet
               [ nixKeyVal "name" (mkStr' "rosPackages")
               , nixKeyVal "buildInputs" (mkList deps')
               , nixKeyVal "src" (mkList [ mkSym "nixpkgs" ]) ]
        deps' = map mkSym ["cmake", "pkgconfig", "rosPackageSet"]

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
