{-# LANGUAGE OverloadedStrings #-}
module RosDep2Nix (rosDep2Nix) where
import Prelude hiding (lookup)
import Data.Map (Map, fromList, lookup)
import Data.Text (Text)

rosPyDeps :: [Text]
rosPyDeps = [ "numpy", "setuptools", "sphinx", "six", "dateutil", "docutils"
            , "argparse", "pyyaml", "nose", "rosdep", "rosinstall-generator"
            , "wstool", "rosinstall", "catkin-tools", "catkin-pkg", "bloom"
            , "empy", "matplotlib", "pillow", "pydot", "paramiko", "coverage"
            , "netifaces", "mock", "psutil", "pyqt4", "pyside", "sip"
            , "pygraphviz" ]

-- | Mapping of catkin system dependency names to the corresponding
-- nixpkgs value. Note that Python dependencies are removed as they
-- are provided by a separate nix Python environment.
aliases :: Map Text Text
aliases = fromList [
            ("pkg-config", "pkgconfig")
          , ("libopencv-dev", "opencv3")
          , ("libconsole-bridge-dev", "console-bridge")
          , ("libpcl-all", "pcl")
          , ("libpcl-all-dev", "pcl")
          , ("libpoco-dev", "poco")
          , ("libvtk-java", "vtkWithQt4")
          , ("yaml-cpp", "libyamlcpp")
          , ("python-catkin-pkg", "")
          , ("python-empy", "")
          , ("python-netifaces", "")
          , ("python-numpy", "")
          , ("python-opencv", "")
          , ("python-paramiko", "")
          , ("python-rosdep", "")
          , ("python-sip", "") 
          , ("python-vtk", "")
          , ("python-yaml", "") ]

rosDep2Nix :: Text -> Maybe Text
rosDep2Nix = flip lookup aliases
