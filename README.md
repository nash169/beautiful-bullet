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

### ToDo
- Fix Franka URDF (problem with prismatic links hand)
- Check memory allocation
- Add visualization collision objects (implement imgui in graphics-lib)
- Add flags when loading multibody collision shapes

### Dependencies
See the corresponding installation instructions for the libraries listed above.

### Installation
Clone the repository including the submodules
```sh
git clone --recursive https://github.com/nash169/beautiful-bullet.git (git@github.com:nash169/beautiful-bullet.git)
```
**beautiful-bullet** relies on WAF compilation tool.
Arch provides an updated version of WAF exec in the standard repo
```sh
sudo pacman -S waf
```
For other distros it is better to download the latest version from the official website and move the executable in the library repo
```sh
wget 'https://waf.io/waf-2.0.23'
mv waf-2.0.23 waf && mv waf /path/to/beautiful-bullet
cd /path/to/kernel-lib
chmod +x waf
```
Compile and install using waf commands
```sh
waf (./waf) configure build
```
or
```sh
waf (./waf) configure && waf (./waf)
```
Install the library (optional)
```sh
(sudo) waf (./waf) install
```
If you want to make a clean installation
```sh
(sudo) waf (./waf) distclean configure build install
```

#### Compilation options
In order to set the desired compiler define the environment variable CXX=<g++,clang++,icpc> (gnu, clang and intel compiler respectively).

Compile in debug mode (without AVX support)
```sh
waf (./waf) configure --debug
```
Compile static library (default option)
```sh
waf (./waf) configure --static
```
Compile shared library
```sh
waf (./waf) configure --shared
```
Define a specific installation path
```sh
waf (./waf) configure --prefix=/path/to/install/folder
```

### Finding the library
In order to find and link the lib to other projects copy and paste the following file into the waf tools
```sh
scripts/beautifulbullet.py
```

### Examples
Once the library is compiled all the examples can be found in
```sh
./build/src/examples/
```
Certain examples depend on **utils-lib**, **control-lib** and **graphics-lib**
```sh
cd /path/to/library
waf (./waf) configure --release build install
```