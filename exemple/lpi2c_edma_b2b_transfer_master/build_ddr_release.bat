cmake -DCMAKE_TOOLCHAIN_FILE="armgcc.cmake" -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=ddr_release  .
mingw32-make -j
IF "%1" == "" ( pause ) 
