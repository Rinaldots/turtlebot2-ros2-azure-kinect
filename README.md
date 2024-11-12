
# turtlebot2 ros2 com azure kinect

### Tutorial de instalação

### Crie um workspace ou use um já criado:

    mkdir turtle_ws
    cd turtle_ws/
    mkdir src
    cd src/
    cd ..
    colcon build

#### Repositório do Libfreenect para o kinect ROS2:
    git clone https://github.com/OpenKinect/libfreenect
    cd libfreenect
    mkdir build
    cd build
    cmake -L .. # -L lists all the project options
    sudo ldconfig -v
    sudo make install
#### Kinect ROS2:

    cd src/
    git clone https://github.com/fadlio/kinect_ros2
    cd ..
    rosdep install --from-paths src --ignore-src -r
#### Pacote Turtlebot2 Azure:
	cd src/
	git clone https://github.com/Rinaldots/turtlebot2-ros2-azure-kinect
	## Dependencias ##
	git clone https://github.com/kobuki-base/kobuki_ros
	git clone https://github.com/kobuki-base/cmd_vel_mux
	cd ..
	rosdep install --from-paths src --ignore-src -r
	colcon build
	
