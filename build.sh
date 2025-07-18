#!/bin/bash
set -e

# Default target is "web"
TARGET="wasm"
SHOULD_RUN=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --target=wasm)
            TARGET="wasm"
            ;;
        --target=native)
            TARGET="native"
            ;;
        run)
            SHOULD_RUN=true
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [--Dplatform=wasm|--Dplatform=native]"
            exit 1
            ;;
    esac
done

if [ "$TARGET" == "wasm" ]; then
    echo "=== BUILD ===: Compiling Zig for WebAssembly..."
    zig build -Dplatform=wasm

    echo "=== BUILD ===: Copying .wasm output..."
    cp -a zig-out/bin/zigics.wasm src/demos/web/public
    cp zig-out/bin/zigics.wasm src/demos/web/dist

    echo "=== BUILD ===: Compiling TypeScript/Webdemo..."
    cd src/demos/web
    npm run build
    cd ..

elif [ "$TARGET" == "native" ]; then
    echo "=== BUILD ===: Compiling Zig for native..."
    zig build

    echo "=== BUILD ===: Native binary is at zig-out/bin/zigics"
    if [ "$SHOULD_RUN" == true ]; then
        echo "=== RUNNING ==="
        zig build run
    fi
else
    echo "Unknown target: $TARGET"
    exit 1
fi

# echo "BUILD: Compiling zig..:"
# zig build -Dtarget=wasm32-freestanding
# cp -a zig-out/bin/zigics.wasm webdemo/public
# cp zig-out/bin/zigics.wasm webdemo/dist
# cd webdemo
# echo "BUILD: Compiling typescript/webdemo"
# npm run build
