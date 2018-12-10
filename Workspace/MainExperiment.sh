#!/bin/bash

# Define timestamp
#timestamp=$date "+%Y%m%d-%H%M%S"
timestamp() {
  date +"%Y%m%d-%H%M%S"
}

../build/robogen-server 8001 &
../build/robogen-evolver 1 ./Results/"$(timestamp)" ./evolConfFile_Maze.txt 
