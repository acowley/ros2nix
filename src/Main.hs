{-# LANGUAGE FlexibleContexts, OverloadedStrings, TemplateHaskell #-}
{-# OPTIONS_GHC -fdefer-type-errors #-}
import qualified Codec.Archive.Tar as Tar
import Codec.Compression.GZip (decompress)
import Control.Applicative ((<|>), liftA2)
import Control.Lens
import Control.Monad ((>=>), filterM)
import Data.Bool (bool)
import qualified Data.ByteString.Lazy as BL
import Data.Foldable (fold)
import Data.List (isSuffixOf)
import Data.Maybe (catMaybes, fromMaybe)
import qualified Data.Set as S
import Data.Text (Text)
import qualified Data.Text as T
import Data.Text.Encoding
import Data.Yaml.YamlLight
import Data.Yaml.YamlLight.Lens
import Nix.Pretty (prettyNix)
import Nix.Types
import System.Directory (doesFileExist, getDirectoryContents, doesDirectoryExist)
import System.Environment (getArgs)
import System.FilePath ((</>), takeBaseName)
import System.IO.Temp (withSystemTempDirectory)
import System.Process (readCreateProcessWithExitCode, proc)
import Text.XML.HXT.Core

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
        deps' = map mkSym ("cmake" : "pkgconfig" : deps pkg)

-- | Prefetch the source for a ROS package into the Nix store
-- recording its hash and list of dependencies.
prefetch :: RosPackage -> IO Package
prefetch pkg = do (_,sha,path) <- readCreateProcessWithExitCode cp ""
                  putStrLn $ "Prefetched "++ T.unpack (view localName pkg)
                  -- path is printed on stderr as: path is '/nix/store/....'
                  Package pkg (T.pack (init sha))
                    <$> extractDeps (init (init (drop 9 path)))
  where url = T.unpack (view uri pkg)
        cp = proc "nix-prefetch-url" [url]
        oops path NoPackageXML = error $ "Couldn't find package.xml in "++path
        oops path NotGzip = error $ "Couldn't decompress "++path++" (not gzip)"
        extractDeps path = either (oops path) id <$> extractDependencies path

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
  withSystemTempDirectory "ros2nix" $ \dir ->
    do BL.readFile nixPath >>= Tar.unpack dir . Tar.read . decompress 
       dirs <- filter (any (/= '.')) <$> getDirectoryContents dir
               >>= filterM (doesDirectoryExist . (dir </>))
       pkgDir <- firstM (\d -> doesFileExist (dir </> d </> "package.xml")) dirs
       case pkgDir of
         Nothing -> return (Left NoPackageXML)
         Just d -> Right <$> getDependencies (dir </> d </> "package.xml")

-- | Parse a @.rosinstall@ file extracting package information
getPackages :: FilePath -> IO [RosPackage]
getPackages f = aux <$> parseYamlFile f -- >>= mapM prefetch
  where aux y = catMaybes $ y ^.. each . key "tar" . to pkg
        pkg :: YamlLight -> Maybe RosPackage
        pkg y = RosPackage 
              <$> y ^? key "local-name" . _Yaml . to decodeUtf8
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
mkPackageSet :: [Package] -> NExpr
mkPackageSet pkgs = mkLet [ callPkg
                          , NamedVar (mkSelector "rosPackageSet") pkgSet ]
                          pkgSetName
  where callPkg = NamedVar (mkSelector "callPackage") $
                  mkApp (mkSym "stdenv.lib.callPackageWith")
                        pkgSetName
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
mkMetaPackage :: Text -> [Package] -> NExpr
mkMetaPackage name pkgs = mkFunction args body
  where args = mkFormalSet $ zip ("stdenv" : externalDeps pkgs) (repeat Nothing)
        body = mkApp (mkSym "stdenv.mkDerivation") $ mkNonRecSet
               [ nixLet "name" (mkStr' name)
               , nixLet "buildInputs" (mkList deps') ]
        deps' = map mkSym ["cmake", "pkgconfig"] ++ [mkPackageSet pkgs]

guessPackageSetName :: Text -> Text
guessPackageSetName name
  | "ros-" `T.isPrefixOf` name = guessPackageSetName (T.drop 4 name)
  | "-src" `T.isSuffixOf` name = guessPackageSetName (T.dropEnd 4 name)
  | otherwise = name

main :: IO ()
main = do args <- getArgs
          case args of
            [f] -> do fexists <- doesFileExist f
                      let setName = guessPackageSetName . T.pack $ takeBaseName f
                      if fexists
                      then do pkgs <- getPackages f >>= mapM prefetch
                              let pkgSet = mkPackageSet pkgs
                                  meta = mkMetaPackage setName pkgs
                              print (prettyNix meta)
                      else putStrLn $ "Couldn't find .rosinstall file: " ++ f
            _ -> putStrLn $ "Usage: ros2nix distro.rosinstall"
