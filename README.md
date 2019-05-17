# Game Engine
## Project Structure
The Game Engine has three main components: the Mediation Layer (ML), the Physics
Simulator (PS), and the Autonomy Protocol (AP). These three programs interact
together to form the Game Engine (GE).

The PS and the AP are co-dependent --- neither can accomplish its task without
the other. The AP maps the current quadcopters' states to an intended
trajectory. The PS forward-simulates the AP's intended trajectories, injecting
distrubances, and returns the quadcopter's state at a future time point.

The ML simply displays the data that the PS and the AP are passing back and
forth.

As it stands, the Mediation Layer is a vestige of a previous iteration of the
Game Engine. Originally, the ML was supposed to forward-simulate the intended
trajectories output by the AP and inject disturbances that would force the
trajectories away from other static and dynamic objects. Since the Machine Games
rules changes, this integration is no longer necessary.

## Installation

### Quick Start

Run the following to install the prerequisites, install the simulator, and run an example autonomy protocol. 

```bash
chmod 744 install_prereqs.sh install.sh run_example.sh
sudo ./install_prereqs.sh
source ~/.bashrc
./install.sh
./run_example.sh
```

If the install fails, manual install instructions are below.

### Prerequisites 
[Eigen](bitbucket.org/eigen/eigen/get/3.3.7.tar.gz)

Download the latest stable release (3.3.7) from the link above. Untar it and move it to your home folder 

```bash
tar -xvf ~/Downloads/$WHATEVER_THE_EIGEN_TAR_FILENAME_IS
mv $WHATEVER_THE_EIGEN_FOLDER_IS ~/eigen
```

Install Eigen

```bash
cd ~/eigen
mkdir build
cd build
cmake ..
make install
```

Install ROS Melodic.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt update

sudo apt install ros-melodic-desktop-full
```

Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

Add ROS environment variables to bashrc

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install these ROS dependencies

```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Install gnuplot and boost

```bash
sudo apt install libboost-all-dev gnuplot
```

Install OSQP

```bash
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build . --target install
```

Install [P4](https://github.com/tuckerhaydon/P4.git)

```bash
git clone https://github.com/tuckerhaydon/P4.git
cd P4
mkdir build
cd build
cmake ..
make -j4
```

Here it might complain that it cant find the OSQP cmake config file. You'll have to set the osqp dir environment variable to the path where the osqp-config.cmake resides before you run cmake. This cmake config file should been in ~/osqp/build/ and can be set with. 

```bash
export osqp_DIR=~/osqp/build
```

Install fftw3

```bash
 sudo apt-get install libfftw3-dev libfftw3-doc
```

### Clone
```bash
git clone https://github.com/rdmelzer/MediationLayerStudent.git GameEngine
cd GameEngine
git submodule update --init --recursive
```

### Build
```bash
mkdir build 
cd build
cmake ..
make -j4
```

## Running the Mediation Layer
The ML is comprised of a couple of executables. After building, you must ensure
that the following programs are running. It may be helpful to use a terminal
multiplexer like tmux or terminator and start each program in a separate pane, or run each process in the backround with &. 

### ROS Core
```bash
roscore
```

### Load ROS params
ROS params need only be loaded once. This must be run after roscore has been
started or re-run if any of the parameters have been changed. **Change the paths in the params.yaml to your paths.**
```bash

cd GameEngine/run
rosparam load params.yaml /mediation_layer/
```

### ROS Visualizer
The ROS visualizer manages a 3D environment that the simulation will provides
displays for.
```
cd GameEngine/run
rosrun rviz rviz -d config.rviz
```

### Mediation Layer
The mediation layer mediates proposed trajectories and ensures that they won't
cause a quadcopter to crash.
```
cd GameEngine/bin
./mediation_layer
```

### Physics Simulator
The physics simulator forward-integrates proposed quadcopter trajectories a
small period in the future and publishes the resulting state.
```
cd GameEngine/bin
./physics_simulator
```

### Autonomy Protocol
The autonomy protocol takes the current quadcopter state and publishes a
proposed trajectory for the quadcopter to follow.
```
cd GameEngine/bin
./example_autonomy_protocol
```

### Unit Tests
```bash
cd build/test
./{$any_unit_test_executable}
```

## TODO
- Disentagle the visualization parts of the ML from the logical parts of the ML.
  Break them into two separate binaries 
- Complete the ML's intended purpose: forward-integrating the intended
  trajectory to create a mediated trajectory
- Visualize the intended trajectories of the quadcopters
- Create StateWatchdog: determine if a quadcopter has flown into the no-fly-zone
  around obstacles. If so, end the game
- Create StateConstraints structure that lists the constraints of a quadcopter's
  actions. Pass an instance of this to the AutonomyProtocol.
