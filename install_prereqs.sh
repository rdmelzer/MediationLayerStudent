if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

CLONE_DIR=`pwd`

cd ~
wget bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
mv 3.3.7.tar.gz eigen.tar.gz
mkdir eigen
tar -xvf eigen.tar.gz -C ~/eigen --strip-components=1

cd ~/eigen
mkdir build
cd build
cmake ..
make install

cd ~
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
apt update
apt install ros-melodic-desktop-full

rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
#source ~/.bashrc

apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
apt install libboost-all-dev gnuplot
apt install libfftw3-dev libfftw3-doc

git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build . --target install
export osqp_DIR=~/osqp/build

cd ~
git clone https://github.com/tuckerhaydon/P4.git
cd P4
mkdir build
cd build
cmake ..
make -j4

cd $CLONE_DIR