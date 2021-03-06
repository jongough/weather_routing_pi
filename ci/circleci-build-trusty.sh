#!/usr/bin/env bash

#
# Build the Debian artifacts
#  from squiddio
set -xe
sudo apt-get -qq update
sudo apt-get install devscripts equivs

rm -rf build && mkdir build && cd build

mk-build-deps ../ci/control
sudo dpkg -i ./*all.deb   || :
sudo apt-get --allow-unauthenticated install -f

sudo apt-get install libglu1-mesa-dev

if [ -n "$BUILD_GTK3" ]; then
    sudo update-alternatives --set wx-config \
        /usr/lib/*-linux-*/wx/config/gtk3-unicode-3.0
fi

cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -sj2
make package


#
# Build the Trusty Ubuntu artifacts
#  from testplugin

#set -xe
#sudo apt-get -qq update
#sudo apt-get install devscripts equivs gdebi
#
#rm -rf build && mkdir build && cd build
#mk-build-deps ../ci/control
#sudo gdebi -n ./*all.deb  || :
#sudo apt-get --allow-unauthenticated install -f
#rm -f ./*all.deb
#
#tag=$(git tag --contains HEAD)
#
#if [ -n "$BUILD_GTK3" ]; then
#  sudo update-alternatives --set wx-config /usr/lib/*-linux-*/wx/config/gtk3-unicode-3.0
#fi
#
#if [ -n "$tag" ]; then
#  cmake -DCMAKE_BUILD_TYPE=Release ..
#else
#  cmake -DCMAKE_BUILD_TYPE=Debug ..
#fi
#
#make -j2
#make package
#ls -l
#