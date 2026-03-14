#!/bin/bash

# Command 1
cd /Users/tamurashuntarou/laboratory_code/CNav_MyStyle/build
make

# Command 2
cd /Users/tamurashuntarou/laboratory_code/CNav_MyStyle/executable
g++ -o compromiseLoop /Users/tamurashuntarou/laboratory_code/CNav_MyStyle/simulations/compromiseLoop.cpp -I/Users/tamurashuntarou/laboratory_code/CNav_MyStyle/src -L/Users/tamurashuntarou/laboratory_code/CNav_MyStyle/build -lRVO

/Users/tamurashuntarou/laboratory_code/CNav_MyStyle/executable/compromiseLoop