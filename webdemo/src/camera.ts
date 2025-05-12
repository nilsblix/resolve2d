import * as nmath from "./nmath.ts";
import { Vector2 } from "./nmath.ts";

export type Size = {
    width: number;
    height: number;
}

export type TransformMult = {
    w2s: number;
    s2w: number;
}

export type Camera = {
    pos: Vector2,
    zoom: number,
    viewport: Size,
}

export class Units {
    camera: Camera;
    mult: TransformMult;
    // In custom unit space.
    default_world_size: Size;
    // Pixels in this case.
    screen_size: Size;

    constructor(screen_size: Size, default_world_width: number) {
        const aspect_ratio = screen_size.height / screen_size.width;

        const def_world_size: Size = {
            width: default_world_width,
            height: default_world_width * aspect_ratio
        };

        const camera: Camera = {
            pos: new Vector2(),
            zoom: 1.0,
            viewport: def_world_size
        };

        const mult: TransformMult = {
            w2s: screen_size.width / default_world_width,
            s2w: default_world_width / screen_size.width,
        };

        this.camera = camera;
        this.mult = mult;
        this.default_world_size = def_world_size;
        this.screen_size = screen_size;
    }

    updateViewPort(): void {
        const dims = new Vector2(this.screen_size.width, this.screen_size.height);
        const world_view = nmath.scale2(dims, this.mult.s2w);
        this.camera.viewport = {
            width: world_view.x,
            height: world_view.y
        };
    }

    adjustCameraZoom(factor: number, screen_pos: Vector2): void {
        const old_wpos = this.s2w(screen_pos);
        this.camera.zoom /= factor;

        this.mult.w2s = (this.screen_size.width / this.default_world_size.width) / this.camera.zoom;
        this.mult.s2w = (this.default_world_size.width / this.screen_size.width) * this.camera.zoom;

        const new_wpos = this.s2w(screen_pos);
        const delta_world = nmath.sub2(new_wpos, old_wpos);
        this.camera.pos.sub(delta_world);

        this.updateViewPort();
    }

    adjustCameraPos(delta: Vector2): void {
        this.camera.pos.sub(delta);
    }

    w2s(pos: Vector2): Vector2 {
        const x = this.mult.w2s * (pos.x - this.camera.pos.x);
        const y = this.screen_size.height - this.mult.w2s * (pos.y - this.camera.pos.y);
        return new Vector2(x, y);
    }

    s2w(pos: Vector2): Vector2 {
        const x = pos.x * this.mult.s2w + this.camera.pos.x;
        const y = (this.screen_size.height - pos.y) * this.mult.s2w + this.camera.pos.y;
        return new Vector2(x, y);
    }

    map(x: number, omin: number, omax: number, nmin: number, nmax: number): number {
        return nmin + ((x - omin) * (nmin - nmax)) / (omin - omax);
    }
}
