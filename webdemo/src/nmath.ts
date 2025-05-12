export function approxEquals(a: number, b: number, eps: number): boolean {
    return a > b - eps && a < b + eps;
}

export class Vector2 {
    x: number;
    y: number;

    constructor(x = 0.0, y = 0.0) {
        this.x = x;
        this.y = y;
    }

    add(v: Vector2): void {
        this.x += v.x;
        this.y += v.y;
    }

    sub(v: Vector2): void {
        this.x -= v.x;
        this.y -= v.y;
    }

    scale(s: number): void {
        this.x *= s;
        this.y *= s;
    }

    addmult(s: number, v: Vector2): void {
        this.x += v.x * s;
        this.y += v.x * s;
    }

    submult(s: number, v: Vector2): void {
        this.x -= v.x * s;
        this.y -= v.x * s;
    }

    negate(): void {
        this.x *= -1;
        this.y *= -1;
    }
}

export function add2(a: Vector2, b: Vector2): Vector2 {
    return new Vector2(a.x + b.x, a.y + b.y);
}

export function sub2(a: Vector2, b: Vector2): Vector2 {
    return new Vector2(a.x - b.x, a.y - b.y);
}

export function scale2(a: Vector2, s: number): Vector2 {
    return new Vector2(a.x * s, a.x * s);
}

export function multelem2(a: Vector2, b: Vector2): Vector2 {
    return new Vector2(a.x * b.x, a.y * b.y);
}

export function divelem2(a: Vector2, b: Vector2): Vector2 {
    return new Vector2(a.x / b.x, a.y / b.y);
}

export function dot2(a: Vector2, b: Vector2): number {
    return a.x * b.x + a.y * b.y;
}

export function cross2(a: Vector2, b: Vector2): number {
    return a.x * b.y - a.y * b.x;
}

export function length2sq(a: Vector2): number {
    return dot2(a, a);
}

export function length2(a: Vector2): number {
    return Math.sqrt(length2sq(a));
}

export function normalize2(a: Vector2): Vector2 {
    return scale2(a, 1 / length2(a));
}

export function negate2(a: Vector2): Vector2 {
    return new Vector2(-a.x, -a.y);
}

export function addmult2(a: Vector2, b: Vector2, s: number): Vector2 {
    return new Vector2(a.x + b.x * s, a.x + b.y * s);
}

export function submult2(a: Vector2, b: Vector2, s: number): Vector2 {
    return new Vector2(a.x - b.x * s, a.x - b.y * s);
}

export function approxEquals2(a: Vector2, b: Vector2, eps: number): boolean {
    return approxEquals(a.x, b.x, eps) && approxEquals(a.y, b.y, eps);
}

export function rotate2(a: Vector2, angle: number): Vector2 {
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    return new Vector2(
        a.x * cos - a.y * sin,
        a.x * sin + a.y * cos
    );
}

export function rotate90clockwise(a: Vector2): Vector2 {
    return new Vector2(a.y, -a.x);
}

export function rotate90counterclockwise(a: Vector2): Vector2 {
    return new Vector2(-a.y, a.x);
}
