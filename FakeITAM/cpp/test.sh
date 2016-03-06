#!/bin/bash
HIL="\033[32;1;4m==> "
END="\n\033[0m"

echo

if [ $1 ] && [ $1 = "clean" ]; then
  echo -e "\033[32;1;4m==> Clean\n\033[0m"
  rm -rf build/*
  rm -rf bin/*
  echo -e "\033[32;1;4m==> Exit\n\033[0m"
  exit
fi

if [ ! -d "bin" ]; then mkdir bin; fi
if [ ! -d "build" ]; then mkdir build; fi

echo -e "${HIL}Change dir to 'bin'${END}"
cd bin

run=true
build_type=$1
if [ ! $build_type ]; then
  if [ -f Test ]; then
    build_type="Release"
  elif [ -f Test_d ]; then
    build_type="Debug"
  else
    build_type="Debug"
  fi
elif [ ${build_type:0:1} = "M" ] || [ ${build_type:0:1} = "m" ]; then
  if [ -f Test ]; then
    build_type="Release"
  elif [ -f Test_d ]; then
    build_type="Debug"
  else
    build_type="Debug"
  fi
  run=false
elif [ ${build_type:0:1} = "R" ] || [ ${build_type:0:1} = "r" ]; then
  build_type="Release"
elif [ ${build_type:0:1} = "D" ] || [ ${build_type:0:1} = "d" ]; then
  build_type="Debug"
fi

bin_name=""
rerun_cmake=false
if [ $build_type = "Release" ]; then
  bin_name="Test"
  if [ -f "Test_d" ]; then rm "Test_d"; fi
elif [ $build_type = "Debug" ]; then
  bin_name="Test_d"
  if [ -f "Test" ]; then rm "Test"; fi
fi

if [ ! -f $bin_name ]; then rerun_cmake=true; fi

if [ $rerun_cmake = true ]; then
  echo -e "${HIL}Change dir to 'build'${END}"
  cd ../build

  echo -e "${HIL}cmake ../ -DCMAKE_BUILD_TYPE=${build_type}${END}"
  cmake ../ -DCMAKE_BUILD_TYPE=$build_type
  echo

  echo -e "${HIL}Change dir to 'bin'${END}"
  cd ../bin
fi

echo -e "${HIL}make -j 8 -C ../build/${END}"
make -j 8 -C ../build/
if [ ! $bin_name = "Test" ]; then
  mv Test $bin_name
fi
echo

if [ $run = true ]; then
  echo -e "${HIL}Press <Enter> to run ${bin_name}${END}"
  read
  echo -e "${HIL}./${bin_name} ~/Desktop/Teddy/calib.txt ~/Desktop/Teddy/Frames/%04i.ppm ~/Desktop/Teddy/Frames/%04i.pgm${END}"
  ./${bin_name} ~/Desktop/Teddy/calib.txt ~/Desktop/Teddy/Frames/%04i.ppm ~/Desktop/Teddy/Frames/%04i.pgm
fi

echo -e "\033[32;1;4m==> Exit\n\033[0m"
