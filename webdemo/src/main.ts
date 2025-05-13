import * as bridge from "./wasm_bridge.ts";
import * as rendmod from "./renderer.ts";
import * as unitmod from "./units.ts";

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
        const wasmModule = await WebAssembly.instantiateStreaming(fetch("/zig-out/bin/zigics.wasm"));
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

await bootstrap();

const app = (() => {
    const canvas = document.getElementById("demo") as HTMLCanvasElement;
    const context = canvas.getContext("2d") as CanvasRenderingContext2D;

    const renderer = new rendmod.Renderer(context, 16 / 9, 100);
    renderer.units.camera.pos.x = -10;

    if (wasm.instance == undefined) {
        console.error("Error: BAD! `wasm.instance` is undefined");
        return;
    }

    // const fns = wasm.instance.exports as any;
    // const num = fns.solverGetNumBodies();
    //
    // for (let i = 0n; i < num; i++) {
    //     renderer.addStandardRigidBodyTex(i % 2n == 0n ? rendmod.IMAGE_PATHS.wheel : rendmod.IMAGE_PATHS.red_truck, i);
    // }

    return {
        c: context,
        renderer: renderer
    };
})();

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
        console.log("\nUpdate time = " + outer_dt);

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

updateLoop(() => {
// (() => {
    if (wasm.instance == undefined) {
        console.error("Error: BAD! `wasm.instance` is undefined");
        return;
    }

    const fns = wasm.instance.exports as any;
    fns.solverProcess(DT, 2, 4);

    app?.renderer.render(fns, app.c);

    // const num = fns.solverGetNumBodies();
    // console.log("num = " + num);
    // for (let i = 0; i < num; i++) {
    //     const id = fns.solverGetBodyIdBasedOnIter(i);
    //     console.log(new bridge.RigidBody(wasm, id).zig.props.pos);
    // }
// })();
});
