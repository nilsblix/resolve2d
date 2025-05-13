import * as nmath from "./nmath.ts";
import { Vector2 } from "./nmath.ts";
import * as unitsmod from "./units.ts";
import { Units } from "./units.ts";
import * as bridge from "./wasm_bridge.ts";

export const IMAGE_PATHS = {
    wheel: "/wheel.png",
    red_truck: "/red_truck.png",
};

export type RigidBodyTex = {
    image: HTMLImageElement;
    id: bigint;
    render: (c: CanvasRenderingContext2D, body: bridge.RigidBody) => void;
};

export class Renderer {
    units: Units;
    textures: Map<bigint, RigidBodyTex>;

    constructor(c: CanvasRenderingContext2D, desired_aspect: number, start_world_width: number) {
        this.units = new Units(c, desired_aspect, start_world_width);
        this.textures = new Map();
    }

    /*
    * fns is wasm.instance.exports
    */
    render(fns: any, c: CanvasRenderingContext2D): void {
        c.clearRect(0, 0, c.canvas.width, c.canvas.height);

        this.units.update(c);

        const num = fns.solverGetNumBodies();
        console.log("num = " + num);
        for (let i = 0; i < num; i++) {
            const id = fns.solverGetBodyIdBasedOnIter(i);
            if (this.textures.has(id)) continue;
            // Render the rigidbody if there isn't a defined texture.
            // FIXME:
        }

        this.textures.forEach((value, _1, _2) => {
            const body = new bridge.RigidBody(fns, value.id);
            value.render(c, body);
        });
    }

    addStandardRigidBodyTex(path: string, id: bigint): void {
        const self = this;

        const tex: RigidBodyTex = {
            image: new Image(),
            id: id,
            render: function(c: CanvasRenderingContext2D, body: bridge.RigidBody): void {
                const zig = body.zig;
                if (zig == undefined) return;
                const pos = zig.props.pos;
                const screen_pos = self.units.w2s(pos);

                const m = self.units.mult.w2s;

                const aspect = tex.image.height / tex.image.width;

                const scaledWidth = m * 0.8;
                const scaledHeight = m * 0.8 * aspect;

                c.save();
                c.translate(screen_pos.x, screen_pos.y);
                c.rotate(zig.props.angle);

                c.drawImage(tex.image, -scaledWidth / 2, -scaledHeight / 2, scaledWidth, scaledHeight);
                c.restore();
            },
        };

        tex.image.src = path;

        this.textures.set(id, tex);
    }
}
