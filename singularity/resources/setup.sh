#!/bin/bash

./waf configure --exp HTE --dart /workspace --kdtree /workspace/include --robot_dart /workspace --magnum_install_dir /workspace --magnum_integration_install_dir /workspace --magnum_plugins_install_dir /workspace --corrade_install_dir /workspace
./waf --exp HTE -j 10