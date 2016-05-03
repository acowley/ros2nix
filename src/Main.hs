{-# LANGUAGE FlexibleContexts, OverloadedStrings, QuasiQuotes, TemplateHaskell #-}
{-# OPTIONS_GHC -fdefer-type-errors -Wall #-}
import Control.Lens
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
import System.Directory (doesFileExist, getDirectoryContents, doesDirectoryExist)
import System.Environment (getArgs)
import System.FilePath ((</>))
import System.IO.Temp (withSystemTempDirectory)
import System.Process (readCreateProcessWithExitCode, proc)
import Text.XML.HXT.Core
import Data.Monoid ((<>))
import System.Process (callProcess)
import Data.Maybe (maybeToList)

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

-- | Helper to make a Nix let-binding
nixLet :: Text -> NExpr -> Binding NExpr
nixLet k v = NamedVar (mkSelector k) v

-- | Generate a @fetchurl@ Nix expression.
fetchSrc :: Package -> NExpr
fetchSrc pkg = mkApp (mkSym "fetchurl") $ mkNonRecSet
               [ nixLet "url" (mkStr' (view uri pkg))
               , nixLet "sha256" (mkStr' (sha256 pkg)) ]

-- | Generate a Nix derivation for a 'Package'.
nixify :: Package -> NExpr
nixify pkg = mkFunction args body
  where args = mkFormalSet $ zip ("stdenv" : deps pkg) (repeat Nothing)
        body = mkApp (mkSym "stdenv.mkDerivation") $ mkNonRecSet
               [ nixLet "name" (mkStr' (view localName pkg))
               , nixLet "version" (mkStr' (view version pkg))
               , nixLet "src" (fetchSrc pkg)
               , nixLet "buildInputs" (mkList deps') ]
        deps' = map mkSym ("cmake" : "pkgconfig" : "ros-build-tools" : deps pkg)

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
                    >>> hasName "build_depend" <+> hasName "run_depend"
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
        , NamedVar (mkSelector "SHLIB") $
          mkIf (mkSym "stdenv.isDarwin") (mkStr' "dylib") (mkStr' "so")
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
externalDeps pkgs = S.toList $ S.difference allDeps internalPackages
  where internalPackages = S.fromList $ map (view localName) pkgs
        allDeps = S.fromList $ foldMap deps pkgs

-- | Generate a Nix derivation that depends on all given packages.
mkMetaPackage :: [Package] -> NExpr
mkMetaPackage pkgs = mkFunction args body
  where args = mkFormalSet $ zip ("stdenv" : externalDeps pkgs) (repeat Nothing)
        body = letPackageSet pkgs . mkApp (mkSym "stdenv.mkDerivation") $
               mkNonRecSet
               [ nixLet "name" (mkStr' "rosPackages")
               , nixLet "buildInputs" (mkList deps')
               , nixLet "src" (mkList [ mkSym "nixpkgs" ]) ]
        deps' = map mkSym ["cmake", "pkgconfig", "rosPackageSet"]

main :: IO ()
main = withStdoutLogging $
       do args <- getArgs
          cache <- loadCache "perception_hash_cache.txt"
          case args of
            [f] -> do fexists <- doesFileExist f
                      setLogLevel LevelError
                      if fexists
                      then do pkgs <- getPackages f >>= mapM (prefetch cache)
                              print (prettyNix $ mkMetaPackage pkgs)
                      else putStrLn $ "Couldn't find .rosinstall file: " ++ f
            _ -> putStrLn $ "Usage: ros2nix distro.rosinstall"
