# Maze Solver

## Install Dependencies

### Dubins

Install [```pydubins```](https://github.com/AndrewWalker/pydubins) through pip.

    pip install dubins
    
### Cartographer

For the latest instructions check the [```cartographer_ros``` docs](https://google-cartographer-ros.readthedocs.io/en/latest/).

    # Install ninja
    sudo apt-get install ninja-build
    
    # Fetch cartographer_ros
    cd ~/racecar_ws
    wstool init src
    wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
    wstool update -t src
    
    # Install proto3
    src/cartographer/scripts/install_proto3.sh
    
    # Install deb dependencies
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    
    # Build and install.
    catkin_make_isolated --install --use-ninja
    source install_isolated/setup.bash
