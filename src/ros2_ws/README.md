# To build:
* Install ROS2 - Iron
* Install cfclient
* Run:
	````
    pip3 install rowan
	pip3 install cflib transforms3d
	````
* Install ros-iron-tf-transformations
* Set up crazyradio
* Run this in another folder (NOT our repo folder): 
	````
	git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
	````
* Go to the resulting folder and follow the instructions at [link](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#build-python-bindings)
* Run:
	````
	export PYTHONPATH=<replace-with-path-to>/crazyflie-firmware/build:$PYTHONPATH
	````
* Go to this folder
* Run:
	colcon build --symlink-install

# To run:
* Run:
	````
	source install/local_setup.bash
	export PYTHONPATH="<replace-with-path-to>/crazyflie-firmware/build:$PYTHONPATH"
	ros2 launch frtn70 launch.py backend:=sim
	````
* Run in another terminal:
	````
	source install/local_setup.bash
	ros2 run frtn70 swc
	````
	
