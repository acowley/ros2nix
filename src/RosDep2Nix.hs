{-# LANGUAGE OverloadedStrings, TupleSections #-}
module RosDep2Nix (rosDep2Nix, rosPyDeps) where
import Data.Map (Map, fromList, lookup)
import Data.Monoid ((<>))
import Data.Text (Text)
import qualified Data.Text as T
import Prelude hiding (lookup)

rosPyDeps :: [Text]
rosPyDeps = [ "numpy", "setuptools", "sphinx", "#six", "dateutil", "docutils"
            , "argparse", "pyyaml", "nose", "rosdep", "rospkg", "rosdistro"
            , "rosinstall-generator", "wstool", "rosinstall", "catkin_tools"
            , "catkin_pkg", "bloom", "empy", "matplotlib", "pillow", "pydot"
            , "paramiko", "coverage", "netifaces", "mock", "psutil", "pyqt4"
            , "vcstools"
            , "#pyside", "defusedxml", "(pygraphviz.override { doCheck = false; })"
            , "#(callPackage ./sip.nix { inherit fetchurl python buildPythonPackage; })"]

-- | Mapping of catkin system dependency names to the corresponding
-- nixpkgs value. Note that Python dependencies are removed as they
-- are provided by a separate nix Python environment.
renamings :: Map Text Text
renamings = aliases <> fromEnv <> notUsed
  where aliases = fromList [
                    ("pkg-config", "pkgconfig")
                  , ("libopencv-dev", "opencv3")
                  , ("libconsole-bridge-dev", "console-bridge")
                  , ("libpcl-all", "pcl")
                  , ("libpcl-all-dev", "pcl")
                  , ("libpoco-dev", "poco")
                  , ("libvtk-java", "vtkWithQt4")
                  , ("libpng12-dev", "libpng12")
                  , ("libtiff-dev", "libtiff")
                  , ("libv4l-dev", "libv4l")
                  , ("libvtk-qt", "vtkWithQt4")
                  , ("protobuf-dev", "protobuf")
                  , ("yaml-cpp", "libyamlcpp") ]

        -- Handled by the pyEnv used for ROS builds
        fromEnv = fromList $ map (,"")
                  [ "python-catkin-pkg"
                  , "python-defusedxml"
                  , "python-empy"
                  , "python-netifaces"
                  , "python-numpy"
                  , "python-opencv"
                  , "python-paramiko"
                  , "python-rosdep"
                  , "python-sip"
                  , "python-vtk"
                  , "python-yaml"
                  , "python"
                  , "cmake" ]

        notUsed = fromList $ map (,"")
                  [ "python-imaging"
                  , "python-rospkg" ]

rosDep2Nix :: Text -> Maybe Text
rosDep2Nix rosName = maybe (Just rosName) check $ lookup rosName renamings
  where check n | T.null n = Nothing
                | otherwise = Just n
