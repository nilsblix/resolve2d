#!/bin/bash
echo "BUILD: Compiling zig..:"
zig build
cp -a zig-out/bin/zigics.wasm webdemo/public
cp zig-out/bin/zigics.wasm webdemo/dist
cd webdemo
echo "BUILD: Compiling typescript/webdemo"
npm run build
