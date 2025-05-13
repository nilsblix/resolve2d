import * as rendmod from "./renderer.ts";
import * as wasmmod from "./wasm_module.ts";
import * as gui from "./nvb-imgui/src/gui/gui.ts";
import * as bridge from "./wasm_bridge.ts";
import * as nmath from "./nmath.ts";
import { Vector2 } from "./nmath.ts";
const wasm = wasmmod.wasm;

setTimeout(() => { }, 100);

await wasmmod.bootstrap();

const app = (() => {
    const canvas = document.getElementById("demo") as HTMLCanvasElement;
    const context = canvas.getContext("2d") as CanvasRenderingContext2D;

    const renderer = new rendmod.Renderer(context, 16 / 9, 30);
    renderer.units.camera.pos.x = 5;

    if (wasm.instance == undefined) {
        console.error("Error: BAD! `wasm.instance` is undefined");
        return;
    }

    return {
        c: context,
        renderer: renderer,
        simulating: false,
        steps: 0,
    };
})();

const TARGET_FPS = 60;
const DT = 1 / TARGET_FPS;

var comp_dt = 0;

function init() {
    if (wasm.instance == undefined) return;
    const fns = wasm.instance.exports as any;

    fns.solverInit(2, 4);
    fns.setupDemo1();

    // const num = fns.solverGetNumBodies();
    // for (let i = 0n; i < num; i++) {
    //     const path = i % 2n == 0n ? "/red_truck.png" : "/wheel.png";
    //     app?.renderer.addStandardRigidBodyTex(path, i);
    // }
}

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
function updateLoop(update: () => void) {
    let outer_dt = 0;

    const loop = async () => {
        const st = performance.now();
        update();
        const et = performance.now();

        outer_dt = et - st;
        comp_dt = outer_dt;

        const left = 1e3 / TARGET_FPS - outer_dt;
        if (left > 0) {
            await sleep(left);
        } else {
            console.log("EXCEEDED TIME = " + outer_dt + " TARGET DT = " + DT);
        }
        requestAnimationFrame(loop);
    }
    loop();
}

enum Action {
    true,
    toggle_sim,
    process_sim,
}

init();

updateLoop(() => {
    if (wasm.instance == undefined) {
        console.error("Error: BAD! `wasm.instance` is undefined");
        return;
    }

    if (app == undefined) return;

    gui.updateCanvasSizing();
    const stack = new gui.Stack<gui.N<Action>>();
    const win = stack.makeWindow(gui.c, gui.input_state,
        // window is if win is moveable, header is if win is closeable and the rest are self explanatory.
        // Here we see the difference between Action.true and null.
        { window: Action.true, header: Action.true, resizeable: Action.true, close_btn: null, },
        { title: "nvb-zigics | wasm demo", x: 0, y: 0, width: 600, height: 290 },
    );

    const fns = wasm.instance.exports as any;

    win.makeLabel(gui.c, null, "=== DEBUGS ===");
    win.setMode("two columns");

    win.makeLabel(gui.c, null, "Comptime: ");
    win.makeLabel(gui.c, null, Math.floor(10 * comp_dt) / 10 + " ms");

    win.makeLabel(gui.c, null, "Step: ");
    win.makeLabel(gui.c, null, "" + app.steps);

    win.makeLabel(gui.c, null, "Num bodies: ");
    win.makeLabel(gui.c, null, "" + fns.solverGetNumBodies());

    win.setMode("normal");
    win.makeLabel(gui.c, null, "");
    win.makeLabel(gui.c, null, "=== SIMULATION ===");
    win.setMode("two columns");

    win.makeLabel(gui.c, null, "Toggle simulating:");
    win.makeButton(gui.c, Action.toggle_sim, "" + app.simulating);

    win.makeLabel(gui.c, null, "Process:");
    win.makeButton(gui.c, Action.process_sim, "Process");

    // const b10 = new bridge.RigidBody(fns, 61n);
    // if (b10.zig == undefined) return;
    // const view = new Vector2(app.renderer.units.camera.viewport.width, app.renderer.units.camera.viewport.height);
    // app.renderer.units.camera.pos = nmath.sub2(b10.zig.props.pos, nmath.scale2(view, 0.5));

    const req = stack.requestAction(gui.input_state);
    const action = req.action;

    switch (action) {
        case Action.toggle_sim:
            app.simulating = !app.simulating;
            break;
        case Action.process_sim:
            fns.solverProcess(DT, 4, 8);
            app.steps += 1;
            break;
    }

    if (app.simulating) {
        fns.solverProcess(DT, 4, 8);
        app.steps += 1;
    }
    app?.renderer.render(fns, app.c);

    gui.c.clearRect(0, 0, gui.canvas.width, gui.canvas.height);
    stack.stack_render(gui.c, gui.input_state);
    gui.input_state.end();
});
