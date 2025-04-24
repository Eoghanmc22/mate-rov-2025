#!/bin/bash

export PATH=$PATH:$PWD/toolchain-pi5/bin/
export CC="aarch64-none-linux-gnu-gcc"
export CXX="aarch64-none-linux-gnu-g++"
cargo run --package robot --bin robot --target aarch64-unknown-linux-gnu # --features tracy
