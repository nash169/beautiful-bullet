# Beautiful Bullet
Dedicated to whom still likes C++...

Nowadays there are many physics engine. Among the open-source ones Bullet is still one of the best in terms of performances and features availability. Nevertheless, despite having a core written in C/C++, if you really want to use Bullet you have to go for the python wrapper. Trying to use the "core" version is a bit of a nightmare. The code is very messy and almost all the features you would need for basic robotic application are very confusingly coded in the examples. Features like URDF importing, Rigid Body algorithms and even the graphical interface are at the moment available only as examples. In order to make everything works outside from the Bullet you might easily carry over hundreds of sources files dependent on each other.

This repository intends to be a wrapper around ONLY the core of Bullet. It allows to quickly setup your robotic simulation in C++ and test your controllers.

In order to achieve that depends on the following:
- eigen (https://gitlab.com/libeigen/eigen), template library for linear algebra. It is used as standard for all the math objects;
- urdfdom (https://github.com/ros/urdfdom), standard ROS XML parser. It is used to import any urdf file into Bullet btMultiBody object;
- assimp  (https://github.com/assimp/assimp), library to import various 3D model formats. It used to load 3D meshed and create the collision objects in Bullet;
- pinocchio (https://github.com/stack-of-tasks/pinocchio), state-of-the-art implementation of many Rigid Body Algorithms. At the moment it is used to provide functionalities as Jacobian computation, inverse kinematics solution and frame position derivation;
- (optional) graphics-lib (https://github.com/nash169/graphics-lib), my wrapper around magnum. Magnum is a fast, lightweight OpenGL library. It is used to provide a graphical interface to the simulator.

The library is still in its early stages and many other Bullet features remain to be implemented.

## ToDo
- Fix Franka URDF (problem with prismatic links hand)
- Check memory allocation
- Add visualization collision objects (implement imgui in graphics-lib)
- Add flags when loading multibody collision shapes
- Move to shared ptr for bodies internal controllers as well

## Usage
Work in progress. Check the examples to get started.

## Dependencies
See the corresponding installation instructions for the libraries listed above.

In addition, in order to compile the project, install my [waf-tools](https://github.com/nash169/waf-tools.git).

## Installation
Compile and install using waf commands
```sh
waf configure build
```
or
```sh
waf configure && waf
```
Install the library (optional)
```sh
(sudo) waf install
```
If you want to make a clean installation
```sh
(sudo) waf distclean configure build install
```

### Compilation options
In order to set the desired compiler define the environment variable CXX=<g++,clang++,icpc> (gnu, clang and intel compiler respectively).

Compile in debug mode (without AVX support)
```sh
waf configure --debug
```
Compile static library (default option)
```sh
waf configure --static
```
Compile shared library
```sh
waf configure --shared
```
Define a specific installation path
```sh
waf configure --prefix=/path/to/install/folder
```

## Finding the library
In case you want to use this library in your project the waf tool necessary to automatically detect it is already included in my **waf-tools** repository.

## Examples
Once the library is compiled all the examples can be run with
```sh
./build/src/examples/<name_example>
```
Certain examples depend on **utils-lib**, **control-lib** and **graphics-lib**.