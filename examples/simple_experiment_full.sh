#!/bin/bash

./robogen-server 8001 &
./robogen-evolver 1 results/simple_experiment_full ../examples/evolConf-full.txt
