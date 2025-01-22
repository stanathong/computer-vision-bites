#!/bin/sh
# You may need to change the file permission to be able to run the script using the following command.
# chmod +x build.sh
rm -rf build && mkdir -p build && cd build && cmake ../ && make VERBOSE=1