git submodule update --init --recursive
mkdir build 
cd build
cmake ..
make -j4
cd ..