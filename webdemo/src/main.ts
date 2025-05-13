import * as rendmod from "./renderer.ts";
import * as wasmmod from "./wasm_module.ts";
import * as gui from "./nvb-imgui/src/gui/gui.ts";
const wasm = wasmmod.wasm;

await wasmmod.bootstrap();

const app = (() => {
    const canvas = document.getElementById("demo") as HTMLCanvasElement;
    const context = canvas.getContext("2d") as CanvasRenderingContext2D;

    const renderer = new rendmod.Renderer(context, 16 / 9, 25);
    renderer.units.camera.pos.x = 5;

    if (wasm.instance == undefined) {
        console.error("Error: BAD! `wasm.instance` is undefined");
        return;
    }

    const fns = wasm.instance.exports as any;
    const num = fns.solverGetNumBodies();

    for (let i = 0n; i < num; i++) {
        renderer.addStandardRigidBodyTex(i % 2n == 0n ? rendmod.IMAGE_PATHS.wheel : rendmod.IMAGE_PATHS.red_truck, i);
    }

    return {
        c: context,
        renderer: renderer
    };
})();

const TARGET_FPS = 60;
const DT = 1 / TARGET_FPS;

var comp_dt = 0;

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
function updateLoop(update: () => void) {
    let outer_dt = 0;

    const loop = async () => {
        const st = performance.now();
        update();
        const et = performance.now();

        outer_dt = et - st;
        console.log("\nUpdate time = " + outer_dt);
        comp_dt = outer_dt;

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

enum Action {
    true,
}

updateLoop(() => {
    if (wasm.instance == undefined) {
        console.error("Error: BAD! `wasm.instance` is undefined");
        return;
    }

    gui.updateCanvasSizing();
    const stack = new gui.Stack<gui.N<Action>>();
    const win = stack.makeWindow(gui.c, gui.input_state,
        // window is if win is moveable, header is if win is closeable and the rest are self explanatory.
        // Here we see the difference between Action.true and null.
        { window: Action.true, header: Action.true, resizeable: Action.true, close_btn: null, },
        { title: "Window 1", x: 0, y: 0, width: 600, height: 290 },
    );

    win.makeLabel(gui.c, null, "Hello world");
    win.makeLabel(gui.c, null, "Comptime = " + Math.floor(10 * comp_dt) / 10 + "ms");

    const fns = wasm.instance.exports as any;
    fns.solverProcess(DT, 2, 4);

    const req = stack.requestAction(gui.input_state);
    const action = req.action;

    gui.c.clearRect(0, 0, gui.canvas.width, gui.canvas.height);
    stack.stack_render(gui.c, gui.input_state);
    gui.input_state.end();
    app?.renderer.render(fns, app.c);
});
