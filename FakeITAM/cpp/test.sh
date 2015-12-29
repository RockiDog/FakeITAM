#!/bin/bash

if [ ! -d build ]
then
  mkdir build
  cd build
  cmake ..
  make -j8
fi
./Test ~/Desktop/Teddy/calib.txt ~/Desktop/Teddy/Frames/%04i.ppm ~/Desktop/Teddy/Frames/%04i.pgm
