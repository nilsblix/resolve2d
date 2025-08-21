#!/usr/bin/env bash
set -ex

cd examples/native
echo "TEST: Testing examples/native"
zig test src/Units.zig
echo "BUILD: Compiling examples/native"
zig build
cd -

echo "BUILD: Compiling src/core to wasm:"
zig build

echo "BUILD: Copying zig-out/bin/zigics.wasm to examples/web/public and examples/web/dist"
cp -a zig-out/bin/zigics.wasm examples/web/public
cp zig-out/bin/zigics.wasm examples/web/dist

echo "BUILD: Compiling web example"
cd examples/web
npm run build
