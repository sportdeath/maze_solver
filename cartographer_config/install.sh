#!/bin/sh

if [ "$#" -ne 1 ]; then
  echo "Usage: ./install.sh PATH_TO_CARTOGRAPHER_WS"
  exit 1
fi

DIR="$(dirname "$0")"
echo $DIR/racecar_2d.lua

ln -s $DIR/racecar_2d.lua $1/src/cartographer_ros/cartographer_ros/configuration_files/
ln -s $DIR/racecar_2d.launch $1/src/cartographer_ros/cartographer_ros/launch/
cp -r $DIR/racecar_description $1/src/
cd $1
catkin_make_isolated --install --use-ninja
source $1/devel_isolated/setup.bash
