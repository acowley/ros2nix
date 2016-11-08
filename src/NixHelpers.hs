module NixHelpers where
import Data.Fix (Fix(Fix))
import Data.Text (Text)
import Nix.Expr

-- | @mkApp2 f x y@ applies function @f@ to two arguments.
mkApp2 :: NExpr -> NExpr -> NExpr -> NExpr
mkApp2 f x = mkApp (mkApp f x)

-- | @mkApp3 f x y z@ applies function @f@ to three arguments.
mkApp3 :: NExpr -> NExpr -> NExpr -> NExpr -> NExpr
mkApp3 f x = mkApp . mkApp2 f x

-- | Make a (usually multi-line) indented string literal with plain
-- and antiquoted components.
mkIndented :: [Antiquoted Text NExpr] -> NExpr
mkIndented = Fix . NStr . Indented

-- | Make a double-quoted string literal with plain and antiquoted
-- components.
mkDoubleQ :: [Antiquoted Text NExpr] -> NExpr
mkDoubleQ = Fix . NStr . DoubleQuoted
