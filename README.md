# !!! This is a Work-in-Progress !!!

# Quick Start

```shell
$ stack exec ros2nix -- $(nix-build --no-out-link ./ros-distro.nix -A kinetic.perception-src)/kinetic_perception.rosinstall -o kinetic_perception.nix

$ nix-shell ros.nix
```

(Note that building everything takes some time!)

# What it's doing

- The `ros-distro.nix` file uses ROS tooling to obtain a `.rosinstall` file for the desired variant (in the above example, the `perception` variant of the ROS `kinetic` distribution).

- The `ros2nix` program (called with `stack exec` above) generates derivations for all the packages that make up the ROS distribution. In the example, we output those definitions to the file `kinetic_perception.nix`.

- The `ros.nix` file identifies that we want to load `kinetic_perception.nix` and extend it with, for the sake of a satisfying example, `turtlesim.nix`. The full environment is constructed by mixing fixed ROS environment components specified in `ros-build-env.nix` with the specific package set identified by, for example, `kinetic_perception.nix`.

- Entering a `nix-shell ros.nix` sets up the `ROS_PACKAGE_PATH`,  `ROS_MASTER_URI`, and the `PYTHONPATH` such that each package can be found.

To see if things are working, try starting a master node!

## Example usage
In one shell,

```shell
$ nix-shell ros.nix
$ roscore
```

In another,

```shell
$ nix-shell ros.nix
$ rosrun turtlesim turtlesim_node
```

and in yet another,

```shell
$ nix-shell ros.nix
$ rosrun turtlesim turtle_teleop_key
```

# Ugly Gotchas to be Fixed

## Hash Caching
The `ros2nix` program uses a cache mapping package names to Nix hashes of their source. The way that I generate the cache is by loading `src/Main.hs` in a GHCi REPL, and calling `generateCache "/nix/store/..../kinetic_perception.rosinstall" "perception_hash_cache.txt"`. The `ros2nix` program looks for that file, and, if it exists, uses it as a cache. If the file does not exist, it will download each ROS package. The use of this cache file was primarily intended for the development of `ros2nix` when it was being repeatedly run against the same `.rosinstall` files. It might not be necessary as the program stabilizes and is used less frequently.

## Pinned nixpkgs
The infrastructure for using a pinned version of `nixpkgs` is included but not currently in use. I am awaiting the merging of a couple Pull Requests critical to having the `turtlesim` example work on macOS.

