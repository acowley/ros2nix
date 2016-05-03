{-# LANGUAGE OverloadedStrings, ScopedTypeVariables #-}
{-# OPTIONS_GHC -fdefer-type-errors -Wall #-}
-- | Once we have a Nix expression for a ROS package set, pull out
-- each (URL,sha256) pair so that we can regenerate the Nix expression
-- without having to download anything if it is already in the store.
module HashCache where
import Control.Exception (catch, SomeException)
import Data.Map (Map)
import qualified Data.Map as M
import Data.Text (Text)
import qualified Data.Text as T
import qualified Data.Text.IO as T
import Data.Char (isSpace)

type HashCache = Map Text Text

buildCache :: FilePath -> IO (Map Text Text)
buildCache = fmap (M.fromList . aux . T.lines) . T.readFile
  where extractHash = T.dropEnd 2
                    . T.drop (T.length "sha256 = \"")
                    . T.dropWhile isSpace
        aux [] = []
        aux [_] = []
        aux (x:xs'@(y:xs))
          | T.isPrefixOf "url =" x'
            = (T.dropEnd 2 (T.drop (T.length "url = \"") x'), extractHash y)
              : aux xs
          | otherwise = aux xs'
          where x' = T.dropWhile isSpace x

saveCache :: FilePath -> Map Text Text -> IO ()
saveCache f = T.writeFile f . foldMap (\(u,h) -> T.unlines [u,h]) . M.toList

loadCache :: FilePath -> IO (Maybe (Map Text Text))
loadCache f = fmap Just (loadCacheUnsafe f)
              `catch` (\(_::SomeException) -> return Nothing)

loadCacheUnsafe :: FilePath -> IO (Map Text Text)
loadCacheUnsafe = fmap (M.fromList . aux . T.lines) . T.readFile
  where aux [] = []
        aux (x:y:xs) = (x,y) : aux xs
        aux _ = error "Bad cache file"

cacheLookup :: Text -> HashCache -> Maybe Text
cacheLookup = M.lookup
