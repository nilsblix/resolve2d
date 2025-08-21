export interface WasmModule {
    instance: WebAssembly.Instance | undefined;
    init: (obj: WebAssembly.WebAssemblyInstantiatedSource) => void;
}

export const wasm: WasmModule = {
    instance: undefined,

    init(obj) {
        this.instance = obj.instance;
    },
};

export async function bootstrap(): Promise<void> {
    try {
        const wasmModule = await WebAssembly.instantiateStreaming(fetch("zigics.wasm"));
        wasm.init(wasmModule);

        if (wasm.instance == undefined) {
            console.error("Error: BAD! `wasm.instance` is undefined");
            return;
        }

    } catch (error) {
        console.error("Failed to bootstrap WebAssembly module:", error);
    }
}
