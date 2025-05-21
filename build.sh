#!/bin/bash
zig build
cp -a zig-out/bin/zigics.wasm webdemo/public
cp zig-out/bin/zigics.wasm webdemo/dist
