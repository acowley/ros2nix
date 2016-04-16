{-# LANGUAGE FlexibleContexts, OverloadedStrings, TemplateHaskell #-}
import Control.Applicative ((<|>), liftA2)
import qualified Codec.Archive.Tar as Tar
import Codec.Compression.GZip (decompress)
import Control.Lens
import Control.Monad (filterM)
import Data.Bool (bool)
import qualified Data.ByteString.Lazy as BL
import Data.Maybe (catMaybes)
import qualified Data.Set as S
import Data.Text (Text)
import qualified Data.Text as T
import Data.Text.Encoding
import Data.Yaml.YamlLight
import Data.Yaml.YamlLight.Lens
import Nix.Pretty (prettyNix)
import Nix.Types
import System.Directory (doesFileExist, getDirectoryContents, doesDirectoryExist)
import System.FilePath ((</>))
import System.IO.Temp (withSystemTempDirectory)
import System.Process (readCreateProcessWithExitCode, proc)
import Text.XML.HXT.Core
import Data.Maybe (fromMaybe)
import Data.List (isSuffixOf)

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

mkStr' :: Text -> NExpr
mkStr' = mkStr DoubleQuoted

fetchSrc :: Package -> NExpr
fetchSrc pkg = mkApp (mkSym "fetchurl") $ mkNonRecSet
               [ NamedVar (mkSelector "url") (mkStr' (view uri pkg))
               , NamedVar (mkSelector "sha256") (mkStr' (sha256 pkg)) ] 

nixify :: Package -> NExpr
nixify pkg = mkFunction args body
  where args = mkFormalSet $ zip ("stdenv" : deps pkg) (repeat Nothing)
        kv k v = NamedVar (mkSelector k) v
        body = mkApp (mkSym "stdenv.mkDerivation") $ mkNonRecSet
               [ kv "name" (mkStr' (view localName pkg))
               , kv "version" (mkStr' (view version pkg))
               , kv "src" (fetchSrc pkg)
               , kv "buildInputs" (mkList deps') ]
        deps' = map mkSym ("cmake" : "pkgconfig" : deps pkg)

prefetch :: RosPackage -> IO Package
prefetch pkg = do (_,sha,path) <- readCreateProcessWithExitCode cp ""
                  putStrLn $ "Prefetched "++ T.unpack (view localName pkg)
                  -- path is printed as: path is '/nix/store/....'
                  Package pkg (T.pack (init sha))
                    <$> extractDeps (init (init (drop 9 path)))
  where url = T.unpack (view uri pkg)
        cp = proc "nix-prefetch-url" [url]
        oops path NoPackageXML = error $ "Couldn't find package.xml in "++path
        oops path NotGzip = error $ "Couldn't decompress "++path++" (not gzip)"
        extractDeps path = either (oops path) id <$> extractDependencies path

firstM :: Monad m => (a -> m Bool) -> [a] -> m (Maybe a)
firstM f = go
  where go [] = return Nothing
        go (x:xs) = f x >>= bool (go xs) (return (Just x))

data PackageError = NoPackageXML | NotGzip deriving (Eq,Show)

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

parseDependencies :: String -> IOSLA (XIOState s) a String
parseDependencies f = readDocument [] f
                    >>> getChildren
                    >>> hasName "package"
                    >>> getChildren
                    >>> hasName "build_depend" <+> hasName "run_depend"
                    >>> getChildren
                    >>> getText

getDependencies :: FilePath -> IO [Text]
getDependencies = fmap (map T.pack . S.toList . S.fromList)
                . runX . parseDependencies

main :: IO ()
main = do getDependencies testFile >>= mapM_ print
          getPackages testDistro >>= mapM_ print
