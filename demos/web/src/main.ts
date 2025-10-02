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
        span_to_body_id: 3,
    };
})();

const TARGET_FPS = 60;
const DT = 1 / TARGET_FPS;

const PROCESS_CONFIG = {
    sub_steps: 4,
    collision_iters: 4,
};

function initBridgeScene() {
    if (wasm.instance == undefined) return;
    const fns = wasm.instance.exports as any;

    fns.solverInit(2, 4);
    fns.setup_0_2_bridge_stress();
    app?.renderer.textures.clear();
    app?.renderer.addStandardRigidBodyTex(rendmod.IMAGE_PATHS.red_truck, 3, 6.0);
    app?.renderer.addStandardRigidBodyTex(rendmod.IMAGE_PATHS.wheel, 4, 2.05);
    app?.renderer.addStandardRigidBodyTex(rendmod.IMAGE_PATHS.wheel, 5, 2.05);
    app?.renderer.textures.set(6, null);
}

function initCarScene() {
    if (wasm.instance == undefined) return;
    const fns = wasm.instance.exports as any;

    fns.solverInit(2, 4);
    fns.setup_0_1_car_platformer();
    app?.renderer.textures.clear();
    app?.renderer.addStandardRigidBodyTex(rendmod.IMAGE_PATHS.red_truck, 3, 6.0);
    app?.renderer.addStandardRigidBodyTex(rendmod.IMAGE_PATHS.wheel, 4, 2.05);
    app?.renderer.addStandardRigidBodyTex(rendmod.IMAGE_PATHS.wheel, 5, 2.05);
    app?.renderer.textures.set(6, null);
}

window.addEventListener("keydown", (e) => {
    if (wasm.instance == undefined) return;
    const fns = wasm.instance.exports as any;

    const car = fns.getRigidBodyPtrFromId(3);
    const wl = fns.getRigidBodyPtrFromId(4);
    const wr = fns.getRigidBodyPtrFromId(5);

    const max_angular_vel = 22;

    const torque = (v: number): number => {
        return max_angular_vel - 0.2 * Math.sqrt(Math.abs(v));
    }

    const applyWheel = (body: any, mult: number = 1): void => {
        fns.setRigidBodyAngularMomentum(body, mult * torque(fns.getRigidBodyAngularMomentum(body) / fns.getRigidBodyInertia(body)));
    }

    if (e.key == "d") {
        applyWheel(wl, -1);
        applyWheel(wr, -1);
        fns.setRigidBodyTorque(car, 500);
    }

    if (e.key == "a") {
        applyWheel(wl);
        applyWheel(wr);
        fns.setRigidBodyTorque(car, -500);
    }

    const val = 30;

    if (e.key == "ArrowUp") { fns.setRigidBodyMomentumY(car, val); }
    if (e.key == "ArrowLeft") { fns.setRigidBodyMomentumX(car, -val); }
    if (e.key == "ArrowDown") { fns.setRigidBodyMomentumY(car, -val); }
    if (e.key == "ArrowRight") { fns.setRigidBodyMomentumX(car, val); }


    if (e.key == " ") {
        if (app == undefined) return;
        app.simulating = !app?.simulating;
    }
});

function startFixedTicker() {
    const dt_ms = 1000 / TARGET_FPS;
    const start = performance.now();
    let tick_idx = 0; // number of completed fixed updates since start
    let running = true;

    function onTick() {
        if (!running) return;
        const now = performance.now();

        const expected_idx = Math.floor((now - start) / dt_ms);

        if (expected_idx >= tick_idx + 1) {
            const fns = (wasm.instance?.exports as any);
            while (tick_idx < expected_idx) {
                const st = performance.now();
                if (app?.simulating) {
                    fns.solverProcess(DT, PROCESS_CONFIG.sub_steps, PROCESS_CONFIG.collision_iters);
                    if (app) app.steps += 1;
                }
                const et = performance.now();
                if (app) app.process_dt = et - st;
                tick_idx += 1;
            }
        }

        scheduleNext();
    }

    function scheduleNext() {
        if (!running) return;
        const next_time = start + (tick_idx + 1) * dt_ms;
        const delay = Math.max(0, next_time - performance.now());
        setTimeout(onTick, delay);
    }

    scheduleNext();
    return () => { running = false; };
}

function startRenderLoop(renderFrame: () => void) {
    const loop = () => {
        renderFrame();
        requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
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


initCarScene();

startFixedTicker();

startRenderLoop(() => {
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
        { title: "Resolve2D | WASM Demo", x: 0, y: 20, width: 420, height: 500 },
    );

    // We do not care about the return value of this.
    stack.makeWindow(gui.c, gui.input_state,
        { window: null, header: null, resizeable: null, close_btn: null, },
        { title: "Tip: Double click the window-headers to expand/minimize a window", x: 0, y: 0, width: 470, height: 50, },
    );

    gui.input_state.window_minimised[1] = true

    const fns = wasm.instance.exports as any;

    win.makeLabel(gui.c, null, "Simulation values:");
    win.setMode("two columns")

    win.makeLabel(gui.c, null, "Simulation compute time: ");
    const process_dt = Math.floor(10 * app.process_dt) / 10;
    win.makeLabel(gui.c, null, (process_dt == 0) ? "Not simulating" : (process_dt + " ms"));

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
    mpw.x = Math.round(mpw.x * 1000) / 1000
    mpw.y = Math.round(mpw.y * 1000) / 1000
    win.makeLabel(gui.c, null, "(" + mpw.x + ", " + mpw.y + ")");

    win.setMode("normal")
    const half_width = gui.MBBox.calcWidth(win.bbox) / 2;

    win.makeLabel(gui.c, null, "");
    win.makeLabel(gui.c, null, "Interact with the simulation:");
    win.setMode("two columns")

    win.makeText(gui.c, null, "Toggle the simulation:", half_width);
    win.makeButton(gui.c, Action.toggle_sim, "" + app.simulating);

    win.makeText(gui.c, null, "Step one time frame forwards: ", half_width);
    win.makeButton(gui.c, Action.process_sim, "Step");

    win.makeText(gui.c, null, "Process the simulation while holding: ", half_width);
    win.makeDraggable(gui.c, Action.process_sim, "Hold");
    const text = win.makeText(gui.c, null, "Note: The option above will run at the frequency of the UI-update, which will be more frequently than the dedicated physics pipeline. The results of the engine will be the same hover, as the engine is deterministic with the same inputs.");

    win.setMode("normal");
    win.makeLabel(gui.c, null, "");
    win.cursor.x = text.bbox.left;
    win.cursor.y = text.bbox.bottom;
    win.makeLabel(gui.c, null, "Change the rendering options:");
    win.setMode("two columns");

    win.makeText(gui.c, null, "Focus camera position to a RigidBody: ", half_width);
    win.makeButton(gui.c, Action.toggle_snap_to_body, "" + app.snap_to_body);

    win.makeText(gui.c, null, "The index of the focused RigidBody. Hold and drag the button: ", half_width);
    win.makeDraggable(gui.c, Action.change_snap_to_body_id, "Index = " + app.span_to_body_id);

    const demo_win = stack.makeWindow(gui.c, gui.input_state,
        { window: Action.true, header: Action.true, resizeable: Action.true, close_btn: null, },
        { title: "Press the buttons to test the different scenes!", x: 0, y: win.bbox.bottom, width: gui.MBBox.calcWidth(win.bbox), height: 100 },
    );

    demo_win.setMode("two columns")

    const demo_half_width = gui.MBBox.calcWidth(demo_win.bbox) / 2;

    demo_win.makeText(gui.c, null, "Car-Platformer Scene (default): ", demo_half_width);
    demo_win.makeButton(gui.c, Action.init_car_scene, "Init");

    demo_win.makeText(gui.c, null, "Car-Bridge Scene: ", demo_half_width);
    demo_win.makeButton(gui.c, Action.init_bridge_scene, "Init");

    const key_win = stack.makeWindow(gui.c, gui.input_state,
        { window: Action.true, header: Action.true, resizeable: Action.true, close_btn: null, },
        { title: "List of keybindings", x: 0, y: demo_win.bbox.bottom, width: gui.MBBox.calcWidth(win.bbox), height: 110 },
    );

    key_win.setMode("two columns");

    key_win.makeLabel(gui.c, null, "Space");
    key_win.makeLabel(gui.c, null, "Toggle the simulation")

    key_win.makeLabel(gui.c, null, "WASD");
    key_win.makeLabel(gui.c, null, "Drive the car")

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
            // Manual single step (independent of fixed ticker)
            const pst = performance.now();
            fns.solverProcess(DT, PROCESS_CONFIG.sub_steps, PROCESS_CONFIG.collision_iters);
            app.steps += 1;
            app.process_dt = performance.now() - pst;
            break;
        case Action.toggle_snap_to_body:
            app.snap_to_body = !app.snap_to_body;
            break;
        case Action.change_snap_to_body_id:
            const num = fns.solverGetNumBodies();
            app.span_to_body_id = Number(gui.updateDraggableValue(Number(app.span_to_body_id), gui.input_state, 2, { min: 0, max: num - 1 }));
            break;
    }

    app?.renderer.render(fns, app.c);

    gui.c.clearRect(0, 0, gui.canvas.width, gui.canvas.height);
    stack.stack_render(gui.c, gui.input_state);
    gui.input_state.end();
});
