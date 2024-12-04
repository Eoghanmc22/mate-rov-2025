#!/bin/bash

export PATH=$PATH:$PWD/toolchain-pi4/bin/
export CC="arm-none-linux-gnueabihf-gcc"
export CXX="arm-none-linux-gnueabihf-g++"
cargo run --package robot --bin robot --release --target armv7-unknown-linux-gnueabihf # --features tracy
