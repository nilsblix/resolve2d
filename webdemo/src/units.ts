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
    start_world_size: Size;
    desired_aspect: number;
    // Pixels in this case.
    screen_size: Size;

    constructor(c: CanvasRenderingContext2D, desired_aspect: number, start_world_width: number) {
        const screen_size: Size = {
            width: c.canvas.width,
            height: c.canvas.height,
        };

        const mult: TransformMult = {
            w2s: c.canvas.width / start_world_width,
            s2w: start_world_width / c.canvas.width,
        };

        const camera: Camera = {
            pos: new Vector2(),
            zoom: 1.0,
            viewport: {
                width: start_world_width,
                height: start_world_width / desired_aspect,
            },
        };

        this.camera = camera;
        this.mult = mult;
        this.start_world_size = {
            width: start_world_width,
            height: start_world_width / desired_aspect,
        };
        this.desired_aspect = desired_aspect;
        this.screen_size = screen_size;
    }

    update(c: CanvasRenderingContext2D): void {
        const window_ratio = window.innerWidth / window.innerHeight;
        if (window_ratio > this.desired_aspect) {
            c.canvas.height = window.innerHeight;
            c.canvas.width = c.canvas.height * this.desired_aspect;
        } else {
            c.canvas.width = window.innerWidth;
            c.canvas.height = c.canvas.width / this.desired_aspect;
        }

        this.screen_size.width = c.canvas.width;
        this.screen_size.height = c.canvas.height;

        this.mult.s2w = (this.start_world_size.width / this.screen_size.width) / this.camera.zoom;
        this.mult.w2s = (this.screen_size.width / this.start_world_size.width) * this.camera.zoom;
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
