console.log("hello world");

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

        const fns = wasm.instance.exports;

        console.log("Hello after wasm.init!");

        fns.solverInit();
        fns.setupDemo1();

        const ptr = fns.getRigidBodyPtrFromId(2n);
        const x0 = fns.getRigidBodyPosX(ptr);

        const STEPS = 100;
        const DT = 1 / 60;
        const total = STEPS * DT;

        let totalComptime = 0;

        for (let i = 0; i < STEPS; i++) {
            const startTime = performance.now();
            if (!fns.solverProcess(DT, 2, 4)) {
                console.error("Error: solverProcess failed.");
            }
            const endTime = performance.now();
            totalComptime += endTime - startTime;
        }

        const x1 = fns.getRigidBodyPosX(ptr);

        console.log(`Total time = ${total}, Time spent calc = ${1e-3 * totalComptime}`);
        console.log(`x0 => ${x0}`);
        console.log(`x1 => ${x1}`);

        fns.solverDeinit();

        console.log("Hello after init and deinit");
    } catch (error) {
        console.error("Failed to bootstrap WebAssembly module:", error);
    }
}

bootstrap();
