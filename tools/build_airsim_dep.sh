#!/bin/bash

cd ~/FlyingChameleons/FlyChams-Cosys-AirSim || { echo "Directory ~/FlyingChameleons/FlyChams-Cosys-AirSim was not found."; exit 1; }

echo "Building AirSim dependencies..."

./clean.sh
./setup.sh
./build.sh

echo "AirSim dependencies built successfully"