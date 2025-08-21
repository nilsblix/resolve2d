// import * as nmath from "./nmath.ts";
import { Vector2 } from "./nmath.ts";

export const enum RigidBodies {
    disc,
    rectangle,
}

export type Props = {
    pos: Vector2;
    momentum: Vector2;
    force: Vector2;
    mass: number;
    angle: number;
    ang_momentum: number;
    torque: number;
    inertia: number;
    mu: number;
}

export interface AABB {
    pos: Vector2;
    half_width: number;
    half_height: number;
}

export class RigidBody {
    static readonly ZigTranslation: {
        id: number;
        aabb: AABB,
        static: boolean;
        num_normals: number;
        type: RigidBodies;
        props: Props;
        // The implementation of the zig-interface.
        implementation_ptr: number;
    };

    // The ptr to the rigidbody in wasm-memory.
    ptr: number;
    zig: (typeof RigidBody.ZigTranslation) | null;

    constructor(fns: any, id: number) {
        const ptr = fns.getRigidBodyPtrFromId(id);

        this.ptr = ptr;
        this.zig = {
            id: id,
            aabb: {
                pos: new Vector2(
                    fns.getRigidBodyAABBPosX(ptr),
                    fns.getRigidBodyAABBPosY(ptr)
                ),
                half_width: fns.getRigidBodyAABBHalfWidth(ptr),
                half_height: fns.getRigidBodyAABBHalfHeight(ptr),
            },
            static: fns.isRigidBodyStatic(ptr),
            num_normals: fns.getRigidBodyNumNormals(ptr),
            type: fns.getRigidBodyType(ptr) as RigidBodies,
            props: {
                pos: new Vector2(
                    fns.getRigidBodyPosX(ptr),
                    fns.getRigidBodyPosY(ptr)
                ),
                momentum: new Vector2(
                    fns.getRigidBodyMomentumX(ptr),
                    fns.getRigidBodyMomentumY(ptr)
                ),
                force: new Vector2(
                    fns.getRigidBodyForceX(ptr),
                    fns.getRigidBodyForceY(ptr)
                ),
                mass: fns.getRigidBodyMass(ptr),
                angle: fns.getRigidBodyAngle(ptr),
                ang_momentum: fns.getRigidBodyAngularMomentum(ptr),
                torque: fns.getRigidBodyTorque(ptr),
                inertia: fns.getRigidBodyInertia(ptr),
                mu: fns.getRigidBodyFrictionCoeff(ptr),
            },
            implementation_ptr: fns.getRigidBodyImplementation(ptr),
        }
    }

    getImplementationDiscBody(fns: any): DiscBodyImplementation {
        return {
            radius: fns.getDiscBodyRadiusAssumeType(this.zig?.implementation_ptr),
        };
    }

    getImplementationRectangleBody(fns: any): RectangleBodyImplementation {
        return {
            width: fns.getRectangleBodyWidthAssumeType(this.zig?.implementation_ptr),
            height: fns.getRectangleBodyHeightAssumeType(this.zig?.implementation_ptr),
        };
    }

    getImplementation(fns: any): Implementation {
        switch (this.zig?.type) {
            case RigidBodies.disc:
                return this.getImplementationDiscBody(fns);
            case RigidBodies.rectangle:
                return this.getImplementationRectangleBody(fns);
            default:
                throw new Error("Undefined type. Could not return implementation");
        }
    }
}

type Implementation = DiscBodyImplementation | RectangleBodyImplementation;

export type DiscBodyImplementation = {
    radius: number,
};

export type RectangleBodyImplementation = {
    width: number,
    height: number,
};
