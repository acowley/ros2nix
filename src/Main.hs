{-# LANGUAGE Arrows, FlexibleContexts, OverloadedStrings #-}
import Control.Applicative ((<|>), liftA2)
import Control.Lens
import Data.Maybe (catMaybes)
import qualified Data.Set as S
import Data.Text (Text)
import Data.Text.Encoding
import Data.Yaml.YamlLight.Lens
import Data.Yaml.YamlLight
import Text.XML.HXT.Core

testFile :: FilePath
testFile = "/Users/acowley/Documents/Projects/Nix/Ros/indigo_perception_ws/src/actionlib/package.xml"

testDistro :: FilePath
testDistro = "/Users/acowley/Documents/Projects/Nix/Ros/indigo_perception_ws/indigo_perception_ws.rosinstall"

data Package = Package { localName :: Text
                       , uri       :: Text
                       , version   :: Text }
             deriving Show

-- | Parse a @.rosinstall@ file extracting package information
getPackages :: FilePath -> IO [Package]
getPackages f = aux <$> parseYamlFile f
  where aux y = catMaybes $ y ^.. each . key "tar" . to pkg
        pkg :: YamlLight -> Maybe Package
        pkg y = Package 
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
