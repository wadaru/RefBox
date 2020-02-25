#!/bin/bash
pushd ~
git clone https://github.com/robocup-logistics/rcll-refbox
popd
patch -u -p1 -d ~/rcll-refbox < rcll-refbox_ubuntu16.04.patch
patch -u -p1 -d ~/rcll-refbox < rcll-refbox_view3.patch
pushd ~/rcll-refbox
make clean -j3
make -j3
cd src/tools.view3
make clean
make
popd
