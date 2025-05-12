import * as bridge from "./wasm_bridge.ts";

function sleepMicroseconds(microseconds: number): Promise<void> {
    return new Promise((resolve) => {
        const start = performance.now();
        const end = start + microseconds / 1000; // Convert to milliseconds
        const check = () => {
            if (performance.now() >= end) {
                resolve();
            } else {
                queueMicrotask(check); // Minimal delay mechanism
            }
        };
        check();
    });
}

export interface WasmModule {
    instance: WebAssembly.Instance | undefined;
    init: (obj: WebAssembly.WebAssemblyInstantiatedSource) => void;
}

const wasm: WasmModule = {
    instance: undefined,

    init(obj) {
        this.instance = obj.instance;
    },
};

async function bootstrap(): Promise<void> {
    try {
        const wasmModule = await WebAssembly.instantiateStreaming(fetch("../public/zig-out/bin/zigics.wasm"));
        wasm.init(wasmModule);

        if (wasm.instance == undefined) {
            console.error("Error: BAD! `wasm.instance` is undefined");
            return;
        }

        const fns = wasm.instance.exports as any;

        fns.solverInit();
        fns.setupDemo1();
    } catch (error) {
        console.error("Failed to bootstrap WebAssembly module:", error);
    }
}

const TARGET_FPS = 60;
const DT = 1 / TARGET_FPS;

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
function updateLoop(update: () => void) {
    let outer_dt = 0;

    const loop = async () => {
        const st = performance.now();
        update();
        const et = performance.now();

        outer_dt = et - st;

        const left = 1e3 * DT - outer_dt;
        if (left > 0) {
            await sleep(left);
        } else {
            console.log("EXCEEDED TIME = " + outer_dt + " TARGET DT = " + DT);
        }
        const e = performance.now();
        console.log("measured dt = " + (e - st));
        requestAnimationFrame(loop);
    }
    loop();
}

await bootstrap();
updateLoop(() => {
    if (wasm.instance == undefined) {
        console.error("Error: BAD! `wasm.instance` is undefined");
        return;
    }

    const fns = wasm.instance.exports as any;
    fns.solverProcess(DT, 2, 4);

    console.log(new bridge.RigidBody(wasm, 10n));
});
