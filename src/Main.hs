{-# LANGUAGE FlexibleContexts, OverloadedStrings, TemplateHaskell #-}
import Control.Applicative ((<|>), liftA2)
import Control.Lens
import Data.Maybe (catMaybes)
import qualified Data.Set as S
import Data.Text (Text)
import qualified Data.Text as T
import Data.Text.Encoding
import Data.Yaml.YamlLight.Lens
import Data.Yaml.YamlLight
import Nix.Types
import System.Process (readCreateProcessWithExitCode, proc)
-- import System.Process (readProcess)
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
                       , sha256     :: Text }
               deriving Show
makeLenses ''Package

instance HasRosPackage Package where
  rosPackage = lens rosPackage' (\(Package _ s) -> flip Package s)

mkStr' :: Text -> NExpr
mkStr' = mkStr DoubleQuoted

fetchSrc :: Package -> NExpr
fetchSrc pkg = mkApp (mkSym "fetchurl") $ mkNonRecSet
               [ NamedVar (mkSelector "url") (mkStr' (view uri pkg))
               , NamedVar (mkSelector "sha256") (mkStr' (sha256 pkg)) ] 

nixify :: Package -> [String] -> NExpr
nixify pkg deps = mkFunction args body
  where args = mkFormalSet $ zip ("stdenv" : map T.pack deps) (repeat Nothing)
        kv k v = NamedVar (mkSelector k) v
        body = mkApp (mkSym "stdenv.mkDerivation") $ mkNonRecSet
               [ kv "name" (mkStr' (view localName pkg))
               , kv "version" (mkStr' (view version pkg))
               , kv "src" (fetchSrc pkg)
               , kv "buildInputs" (mkList deps') ]
        deps' = map mkSym ["cmake", "pkgconfig"] ++ map (mkSym . T.pack) deps

prefetch :: RosPackage -> IO Package
prefetch pkg = do sha <- T.pack . init . view _2 <$>
                         readCreateProcessWithExitCode cp ""
                  putStrLn $ "Prefetched "++ T.unpack (view localName pkg)
                  return $ Package pkg sha
  where url = T.unpack (view uri pkg)
        cp = proc "nix-prefetch-url" [url]

-- | Parse a @.rosinstall@ file extracting package information
getPackages :: FilePath -> IO [RosPackage]
getPackages f = aux <$> parseYamlFile f -- >>= mapM prefetch
  where aux y = catMaybes $ y ^.. each . key "tar" . to pkg
        pkg :: YamlLight -> Maybe RosPackage
        pkg y = RosPackage 
              <$> y ^? key "local-name" . _Yaml . to decodeUtf8
              <*> y ^? key "uri" . _Yaml . to decodeUtf8
              <*> y ^? key "version" . _Yaml . to decodeUtf8

getDependencies :: String -> IOSLA (XIOState s) a String
getDependencies f = readDocument [] f
                    >>> getChildren
                    >>> hasName "package"
                    >>> getChildren
                    >>> hasName "build_depend" <+> hasName "run_depend"
                    >>> getChildren
                    >>> getText

main :: IO ()
main = do runX (getDependencies testFile)
           >>= mapM_ putStrLn . S.toList . S.fromList
          getPackages testDistro >>= mapM_ print
