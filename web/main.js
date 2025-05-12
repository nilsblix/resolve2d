const text_decoder = new TextDecoder();
let console_log_buffer = "";

let wasm = {
    instance: undefined,

    init: function (obj) {
        this.instance = obj.instance;
    },
    getString: function (ptr, len) {
        const memory = this.instance.exports.memory;
        return text_decoder.decode(new Uint8Array(memory.buffer, ptr, len));
    },
};

let importObject = {
    env: {
        jsConsoleLogWrite: function (ptr, len) {
            console_log_buffer += wasm.getString(ptr, len);
        },
        jsConsoleLogFlush: function () {
            console.log(console_log_buffer);
            console_log_buffer = "";
        }
    }
};

async function bootstrap() {
    wasm.init(await WebAssembly.instantiateStreaming(fetch("../zig-out/bin/zigics.wasm"), importObject));
    const fns = wasm.instance.exports;

    console.log("Hello after wasm.init!");

    fns.solverInit();

    fns.setupDemo1();

    const ptr = fns.getRigidBodyPtrFromId(2n);
    const x0 = fns.getRigidBodyPosX(ptr);

    const STEPS = 100;
    const DT = 1/60;

    const total = STEPS * DT;
    var total_comptime = 0;

    for (let i = 0; i < STEPS; i++) {
        const st = performance.now();
        if (!fns.solverProcess(DT, 2, 4)) console.log("Error.Bad");
        const et = performance.now();
        total_comptime += et - st;
    }

    const x1 = fns.getRigidBodyPosX(ptr);

    console.log("totatime = " + total, "time spent calc = " + 1e-3 * total_comptime);
    console.log("x0 => " + x0);
    console.log("x1 => " + x1);

    fns.solverDeinit();

    console.log("Hello after init and deinit");
}

bootstrap();
