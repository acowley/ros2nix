# !!! This is a Work-in-Progress !!!

# Quick Start
To enter a `nix-shell` with the [ROS Kinetic](http://wiki.ros.org/kinetic) Comm variant available, run,

```shell
$ nix-shell ros.nix
```
(Note that building everything takes some time!). This is actually a slight extension of the stock `comm` variant so that the [turtlesim](http://wiki.ros.org/turtlesim) tutorial can run.

## Regenerating a ROS Distribution

If you want to regenerate the `kinetic_perception.nix` file that defines all the ROS packages that make up the distribution, you will use the `ros2nix` program:

```shell
$ stack exec ros2nix -- $(nix-build --no-out-link ./ros-distro.nix -A kinetic.perception-src)/kinetic_perception.rosinstall -o kinetic_perception.nix
```

## Other ROS Versions and Variants

If you want to use a different ROS distribution or variant, change the
`-A kinetic.perception-src` part of the `ros2nix` incantation. If you
want to use a minimal set of core packages, consider using
`kinetic.ros_comm-src` for the `comm` ROS configuration.

# What it's doing

- The `ros-distro.nix` file uses ROS tooling to obtain a `.rosinstall` file for the desired metapackage/variant (in the above examples, we referred to the `perception` and `comm` variants of the ROS `kinetic` distribution).

- The `ros2nix` program (called with `stack exec` above) generates derivations for all the packages that make up the ROS distribution. In the example, we output those definitions to the file `kinetic_perception.nix`.

- The `ros.nix` file identifies that we want to load `kinetic_comm.nix` and extend it with, for the sake of a satisfying example, `turtlesim.nix`. The full environment is constructed by mixing fixed ROS environment components specified in `ros-build-env.nix` with the specific package set identified by, for example, `kinetic_comm.nix`.

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

![turtlesim demo](/images/TurtleDemo.jpg?raw=true "turtlesim demo")

# Ugly Gotchas to be Fixed

## Hash Caching
The `ros2nix` program uses a cache mapping package names to Nix hashes of their source. The way that I generate the cache is by loading `src/Main.hs` in a GHCi REPL, and calling `generateCache "/nix/store/..../kinetic_perception.rosinstall" "perception_hash_cache.txt"`. The `ros2nix` program looks for that file, and, if it exists, uses it as a cache. If the file does not exist, it will download each ROS package. The use of this cache file was primarily intended for the development of `ros2nix` when it was being repeatedly run against the same `.rosinstall` files. It might not be necessary as the program stabilizes and is used less frequently.

## Pinned nixpkgs
The infrastructure for using a pinned version of `nixpkgs` is included but not currently in use. I am awaiting the merging of a couple Pull Requests critical to having the `turtlesim` example work on macOS.
