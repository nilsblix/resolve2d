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
            // FIXME: uncomment
            // if (this.textures.has(id)) continue;
            const body = new bridge.RigidBody(fns, id);
            const imp = body.getImplementation(fns);

            if (body.zig == undefined) continue;
            const pos = body.zig.props.pos;
            const scr = this.units.w2s(pos);

            switch (body.zig.type) {
                case bridge.RigidBodies.disc:
                    const disc = imp as bridge.DiscBodyImplementation;

                    c.fillStyle = "#FF9011";
                    if (body.zig.static) c.fillStyle = "#fa6c90";
                    c.beginPath();
                    c.arc(scr.x, scr.y, this.units.mult.w2s * disc.radius, 0, 2 * Math.PI);
                    c.fill();
                    c.strokeStyle = "#aa4100";  // Slightly darker green for better contrast
                    if (body.zig.static) c.strokeStyle = "#b14d64";  // Darker red for static ones
                    c.lineWidth = 0.04 * this.units.mult.w2s;  // Adjust the border thickness if needed
                    c.stroke();
                    c.closePath();

                    break;
                case bridge.RigidBodies.rectangle:
                    const rect = imp as bridge.RectangleBodyImplementation;
                    const w = rect.width / 2;
                    const h = rect.height / 2;

                    var dp1 = new Vector2(-w, -h);
                    var dp2 = new Vector2(w, -h);
                    var dp3 = new Vector2(w, h);
                    var dp4 = new Vector2(-w, h);

                    const a = body.zig.props.angle;

                    dp1 = nmath.rotate2(dp1, a);
                    dp2 = nmath.rotate2(dp2, a);
                    dp3 = nmath.rotate2(dp3, a);
                    dp4 = nmath.rotate2(dp4, a);

                    const p1 = nmath.add2(dp1, pos);
                    const p2 = nmath.add2(dp2, pos);
                    const p3 = nmath.add2(dp3, pos);
                    const p4 = nmath.add2(dp4, pos);

                    const s1 = this.units.w2s(p1);
                    const s2 = this.units.w2s(p2);
                    const s3 = this.units.w2s(p3);
                    const s4 = this.units.w2s(p4);

                    c.fillStyle = "#90FF11";
                    if (body.zig.static) c.fillStyle = "#fa6c90";
                    c.beginPath();
                    c.moveTo(s1.x, s1.y);
                    c.lineTo(s2.x, s2.y);
                    c.lineTo(s3.x, s3.y);
                    c.lineTo(s4.x, s4.y);
                    c.closePath();

                    c.fill();
                    c.strokeStyle = "#406d0e";  // Slightly darker green for better contrast
                    if (body.zig.static) c.strokeStyle = "#b14d64";  // Darker red for static ones
                    c.lineWidth = 0.04 * this.units.mult.w2s;  // Adjust the border thickness if needed
                    c.stroke();
                    break;
            }
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
