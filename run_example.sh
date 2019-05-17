roscore &
sleep 5

cd ./run
rosparam load params.yaml /mediation_layer/
rosrun rviz rviz -d config.rviz &

sleep 5

cd ../bin
./mediation_layer & 
./physics_simulator &
./example_autonomy_protocol &