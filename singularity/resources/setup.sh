#!/bin/bash

./waf configure --exp hte --dart /workspace --kdtree /workspace/include --robot_dart /workspace --magnum_install_dir /workspace --magnum_integration_install_dir /workspace --magnum_plugins_install_dir /workspace --corrade_install_dir /workspace
./waf --exp hte -j 10