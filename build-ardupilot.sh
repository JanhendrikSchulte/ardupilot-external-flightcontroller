#!/opt/homebrew/bin/fish
cd $HOME/study/master/ardupilot-external-flightcontroller
./waf distclean
./waf configure --board sitl
./waf copter
./Tools/autotest/sim_vehicle.py -N -v ArduCopter -f gazebo-iris --model JSON --map --console
