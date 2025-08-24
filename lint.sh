#!/usr/bin/env bash
set -ex

echo "BUILD: Compiling src/core to wasm:"
zig fmt src
zig build

echo "BUILD: Copying zig-out/bin/zigics.wasm to demos/web/public and demos/web/dist"
cp -a zig-out/bin/zigics.wasm demos/web/public
cp zig-out/bin/zigics.wasm demos/web/dist

echo "BUILD: Compiling demos/native"
cd demos/native
zig fmt src
zig build
cd -

echo "BUILD: Compiling web example"
cd demos/web
npm run build
