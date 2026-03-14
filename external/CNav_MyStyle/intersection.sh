#!/bin/bash

# Command 1
cd /Users/tamurashuntarou/laboratory_code/CNav_MyStyle/build
make

# Command 2
cd /Users/tamurashuntarou/laboratory_code/CNav_MyStyle/executable
g++ -o intersection /Users/tamurashuntarou/laboratory_code/CNav_MyStyle/simulations/intersection.cpp -I/Users/tamurashuntarou/laboratory_code/CNav_MyStyle/src -L/Users/tamurashuntarou/laboratory_code/CNav_MyStyle/build -lRVO

# Command 3
/Users/tamurashuntarou/laboratory_code/CNav_MyStyle/executable/intersection