# Maze Solver

## Install Dependencies

### Dubins

Install [```pydubins```](https://github.com/AndrewWalker/pydubins) through pip.

    pip install git+git://github.com/sportdeath/pydubins.git

### RTree

Install [```libspatialindex```](http://libspatialindex.github.io/index.html):

    git clone https://github.com/libspatialindex/libspatialindex.git
    sudo apt-get install automake
    cd libspatialindex
    ./autogen.sh
    ./configure
    # ./configure --prefix=/home/racecar-ws/devel
    make
    sudo make install
    sudo ldconfig

Install [```rtree```](http://toblerity.org/rtree/) through pip.

    pip install rtree
    
### Cartographer

[Install cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/):

    # Install ninja
    sudo apt-get install ninja-build
    
    # Make a workspace for cartographer
    mkdir ~/cartographer_ws
    cd ~/cartographer_ws
    wstool init src
    
    # Fetch cartographer_ros
    wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
    wstool update -t src
    
    # Install proto3
    src/cartographer/scripts/install_proto3.sh
    
    # Install deb dependencies
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    
    # Build and install.
    catkin_make_isolated --install --use-ninja
    source install_isolated/setup.bash

Add this to your ~/.bashrc

    source ~/cartographer_ws/install_isolated/setup.bash
