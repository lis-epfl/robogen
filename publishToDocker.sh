#!/bin/bash

rm -rf ./dist_linux
wget https://github.com/cloudimmunity/docker-slim/releases/download/1.14/dist_linux.zip
unzip dist_linux
./dist_linux/docker-slim build --continue-after 10 --show-clogs=true $DOCKER_IMAGE_NAME
docker tag --force $DOCKER_IMAGE_NAME.slim robogen/robogen
docker push robogen/robogen

