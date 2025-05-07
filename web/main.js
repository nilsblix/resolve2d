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
    fns.solverDeinit();

    console.log("Hello after init and deinit");
}

bootstrap();
