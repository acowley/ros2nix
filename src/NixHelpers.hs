module NixHelpers where
import Data.Fix (Fix(Fix))
import Data.Text (Text)
import Nix.Types

-- | Helper for producing a plain double-quoted Nix string.
mkStr' :: Text -> NExpr
mkStr' = mkStr DoubleQuoted

-- | @mkApp2 f x y@ applies function @f@ to two arguments.
mkApp2 :: NExpr -> NExpr -> NExpr -> NExpr
mkApp2 f x = mkApp (mkApp f x)

-- | @mkApp3 f x y z@ applies function @f@ to three arguments.
mkApp3 :: NExpr -> NExpr -> NExpr -> NExpr -> NExpr
mkApp3 f x = mkApp . mkApp2 f x

-- | Make an string literal with plain and antiquoted
-- components.
mkString :: StringKind -> [Antiquoted Text NExpr] -> NExpr
mkString k = Fix . NStr . NString k

-- | Make a (usually multi-line) indented string literal with plain
-- and antiquoted components.
mkIndented :: [Antiquoted Text NExpr] -> NExpr
mkIndented = mkString Indented

-- | Make a double-quoted string literal with plain and antiquoted
-- components.
mkDoubleQ :: [Antiquoted Text NExpr] -> NExpr
mkDoubleQ = mkString DoubleQuoted
