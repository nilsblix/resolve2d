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

    const renderer = new rendmod.Renderer(context, 16 / 9, 40);
    renderer.units.camera.pos = new Vector2(-25, -10);

    if (wasm.instance == undefined) {
        console.error("Error: BAD! `wasm.instance` is undefined");
        return;
    }

    return {
        c: context,
        renderer: renderer,
        simulating: false,
        steps: 0,
        process_dt: 0,
        snap_to_body: true,
        span_to_body_id: 3n,
    };
})();

const TARGET_FPS = 60;
const DT = 1 / TARGET_FPS;

var comp_dt = 0;

function initBridgeScene() {
    if (wasm.instance == undefined) return;
    const fns = wasm.instance.exports as any;

    fns.solverInit(2, 4);
    fns.setupBridgeStressTestScene();
    app?.renderer.textures.clear();
    app?.renderer.addStandardRigidBodyTex("/red_truck.png", 3n, 6.0);
    app?.renderer.addStandardRigidBodyTex("/wheel.png", 4n, 2.05);
    app?.renderer.addStandardRigidBodyTex("/wheel.png", 5n, 2.05);
    app?.renderer.textures.set(6n, null);
}

function initCarScene() {
    if (wasm.instance == undefined) return;
    const fns = wasm.instance.exports as any;

    fns.solverInit(2, 4);
    fns.setupCarScene();
    app?.renderer.textures.clear();
    app?.renderer.addStandardRigidBodyTex("/red_truck.png", 3n, 6.0);
    app?.renderer.addStandardRigidBodyTex("/wheel.png", 4n, 2.05);
    app?.renderer.addStandardRigidBodyTex("/wheel.png", 5n, 2.05);
    app?.renderer.textures.set(6n, null);
}

window.addEventListener("keydown", (e) => {
    if (wasm.instance == undefined) return;
    const fns = wasm.instance.exports as any;

    const body = fns.getRigidBodyPtrFromId(3n);
    const wl = fns.getRigidBodyPtrFromId(4n);
    const wr = fns.getRigidBodyPtrFromId(5n);

    const w_ang = 30;
    const b_ang = 390;

    if (e.key == "d") {
        fns.setRigidBodyAngularMomentum(wl, -w_ang);
        fns.setRigidBodyAngularMomentum(wr, -w_ang);
        fns.setRigidBodyTorque(body, b_ang);
    }

    if (e.key == "a") {
        fns.setRigidBodyAngularMomentum(wl, w_ang);
        fns.setRigidBodyAngularMomentum(wr, w_ang);
        fns.setRigidBodyTorque(body, -b_ang);
    }

    const val = 10;

    if (e.key == "ArrowUp") { fns.setRigidBodyMomentumY(body, val); }
    if (e.key == "ArrowLeft") { fns.setRigidBodyMomentumX(body, -val); }
    if (e.key == "ArrowDown") { fns.setRigidBodyMomentumY(body, -val); }
    if (e.key == "ArrowRight") { fns.setRigidBodyMomentumX(body, val); }


    if (e.key == " ") {
        if (app == undefined) return;
        app.simulating = !app?.simulating;
    }
});

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
    toggle_snap_to_body,
    change_snap_to_body_id,
    init_car_scene,
    init_bridge_scene,
}


// initBridgeStressTestScene();
initCarScene();

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
        { title: "nvb-zigics | wasm demo", x: 0, y: 0, width: 500, height: 400 },
    );

    const fns = wasm.instance.exports as any;

    win.makeLabel(gui.c, null, "=== DEBUGS ===");
    win.setMode("two columns");

    win.makeLabel(gui.c, null, "Comptime: ");
    win.makeLabel(gui.c, null, Math.floor(10 * comp_dt) / 10 + " ms");

    win.makeLabel(gui.c, null, "Process dt: ");
    win.makeLabel(gui.c, null, Math.floor(10 * app.process_dt) / 10 + " ms");

    win.makeLabel(gui.c, null, "Step: ");
    win.makeLabel(gui.c, null, "" + app.steps);

    win.makeLabel(gui.c, null, "Num bodies: ");
    win.makeLabel(gui.c, null, "" + fns.solverGetNumBodies());

    win.makeLabel(gui.c, null, "Mouse world pos: ");
    var mpw = new Vector2(gui.input_state.mouse_position.x, gui.input_state.mouse_position.y);
    const rect = app.c.canvas.getBoundingClientRect();
    mpw.x -= rect.left;
    mpw.y -= rect.top;
    mpw = app.renderer.units.s2w(mpw);
    win.makeLabel(gui.c, null, "(" + mpw.x + ", " + mpw.y + ")");

    win.setMode("normal");
    win.makeLabel(gui.c, null, "");
    win.makeLabel(gui.c, null, "=== SIMULATION ===");
    win.setMode("two columns");

    win.makeLabel(gui.c, null, "Toggle simulating:");
    win.makeButton(gui.c, Action.toggle_sim, "" + app.simulating);

    win.makeLabel(gui.c, null, "Process once:");
    win.makeButton(gui.c, Action.process_sim, "Once");

    win.makeLabel(gui.c, null, "Process while held:");
    win.makeDraggable(gui.c, Action.process_sim, "Hold");

    win.setMode("normal");
    win.makeLabel(gui.c, null, "");
    win.makeLabel(gui.c, null, "=== RENDER ===");
    win.setMode("two columns");

    win.makeLabel(gui.c, null, "Toggle focus camera pos to body: ");
    win.makeButton(gui.c, Action.toggle_snap_to_body, "" + app.snap_to_body);

    win.makeLabel(gui.c, null, "Change focused body id: ");
    win.makeDraggable(gui.c, Action.change_snap_to_body_id, "Id: " + app.span_to_body_id);

    const demowin = stack.makeWindow(gui.c, gui.input_state,
        { window: Action.true, header: Action.true, resizeable: Action.true, close_btn: null, },
        { title: "init demo scenes", x: 900, y: 0, width: 500, height: 200 },
    );

    demowin.setMode("two columns")

    demowin.makeLabel(gui.c, null, "Car Scene: ");
    demowin.makeButton(gui.c, Action.init_car_scene, "Init");

    demowin.makeLabel(gui.c, null, "Bridge Scene: ");
    demowin.makeButton(gui.c, Action.init_bridge_scene, "Init");

    if (app.snap_to_body) {
        const body = new bridge.RigidBody(fns, app.span_to_body_id);
        if (body.zig == undefined) return;
        const view = new Vector2(app.renderer.units.camera.viewport.width, app.renderer.units.camera.viewport.height);
        app.renderer.units.camera.pos = nmath.sub2(body.zig.props.pos, nmath.scale2(view, 0.5));
    }

    const req = stack.requestAction(gui.input_state);
    const action = req.action;

    switch (action) {
        case Action.init_car_scene:
            fns.solverDeinit();
            initCarScene();
            app.steps = 0;
            break;
        case Action.init_bridge_scene:
            fns.solverDeinit();
            initBridgeScene();
            app.steps = 0;
            break;
        case Action.toggle_sim:
            app.simulating = !app.simulating;
            break;
        case Action.process_sim:
            fns.solverProcess(DT, 8, 16);
            app.steps += 1;
            break;
        case Action.toggle_snap_to_body:
            app.snap_to_body = !app.snap_to_body;
            break;
        case Action.change_snap_to_body_id:
            const num = fns.solverGetNumBodies();
            app.span_to_body_id = BigInt(gui.updateDraggableValue(Number(app.span_to_body_id), gui.input_state, 2, { min: 0, max: num - 1 }));
            break;
    }

    const st = performance.now();
    if (app.simulating) {
        fns.solverProcess(DT, 8, 16);
        app.steps += 1;
    }
    const et = performance.now();
    app.process_dt = et - st;

    app?.renderer.render(fns, app.c);

    gui.c.clearRect(0, 0, gui.canvas.width, gui.canvas.height);
    stack.stack_render(gui.c, gui.input_state);
    gui.input_state.end();
});
