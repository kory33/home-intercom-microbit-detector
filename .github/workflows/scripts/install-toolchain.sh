#!/bin/bash
set -ex

# Install cmake and ninja
sudo add-apt-repository -y ppa:adrozdoff/cmake && sudo apt-get update -qq
sudo apt-get install -y cmake ninja-build

toolchain_name=gcc-arm-none-eabi-10-2020-q4-major

export PATH=$PATH:/usr/local/bin/$toolchain_name/bin

# install arm-gcc toolchain using the official distribution
if ! command -v arm-none-eabi-gcc; then
  # https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
  tarball_filename=$toolchain_name-x86_64-linux.tar.bz2
  expected_md5=8312c4c91799885f222f663fc81f9a31

  wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10-2020q4/$tarball_filename
  echo "$expected_md5 $tarball_filename" | md5sum -c -

  tar -jxvf $tarball_filename && rm $tarball_filename
  sudo mv $toolchain_name /usr/local/bin
fi
