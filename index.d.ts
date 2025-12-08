import * as THREE from 'three';

// MARK: - Core
declare namespace Physijs {
  export type PhysijsScripts = {
    worker?: string;
    ammo?: string;
  }

  export const scripts: PhysijsScripts;

  export interface Material extends THREE.Material {
    _physijs: {
      id: number;
      friction: number;
      restitution: number;
    };
  }

  export function createMaterial(
    material: THREE.Material,
    friction?: number,
    restitution?: number
  ): Material;

  export interface Eventable {
    addEventListener(event: string, callback: CallableFunction): void;
    removeEventListener(event: string, callback: CallableFunction): void;
    dispatchEvent<TMap extends {}, T extends Extract<keyof TMap, string>>(event: THREE.BaseEvent<T> & TMap[T]): void;
  }

  export class Scene extends THREE.Scene implements Eventable {
    constructor(parameters?: {
      fixedTimeStep?: number;
      reportSize?: number;
      broadphase?: string;
      gravity?: THREE.Vector3;
    });

    addConstraint(constraint: Constraint, show_marker?: boolean): void;
    onSimulationResume(): void;
    removeConstraint(constraint: Constraint): void;
    add(...object: THREE.Object3D[]): this;
    remove(...object: THREE.Object3D[]): this;
    setFixedTimeStep(timeStep?: number): void;
    setGravity(gravity: THREE.Vector3): void;
    simulate(timeStep?: number, maxSubSteps?: number): void;

    addEventListener(event: string, callback: CallableFunction): void;
    removeEventListener(event: string, callback: CallableFunction): void;
    dispatchEvent<TMap extends {}, T extends Extract<keyof TMap, string>>(event: THREE.BaseEvent<T> & TMap[T]): void;
  }
}

// MARK: - Meshes
declare namespace Physijs {
  export class Mesh extends THREE.Mesh implements Eventable {
    mass:number;

    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);

    applyCentralImpulse(impulse: THREE.Vector3): void;
    applyImpulse(impulse: THREE.Vector3, offset: THREE.Vector3): void;
    applyTorque(torque: THREE.Vector3): void;
    applyCentralForce(force: THREE.Vector3): void;
    applyForce(force: THREE.Vector3, offset: THREE.Vector3): void;
    getAngularVelocity(): THREE.Vector3;
    setAngularVelocity(velocity: THREE.Vector3): void;
    getLinearVelocity(): THREE.Vector3;
    setLinearVelocity(velocity: THREE.Vector3): void;
    setAngularFactor(factor: THREE.Vector3): void;
    setLinearFactor(factor: THREE.Vector3): void;
    setDamping(linear: number, angular: number): void;
    setCcdMotionThreshold(threshold: number): void;
    setCcdSweptSphereRadius(radius: number): void;

    addEventListener(event: string, callback: CallableFunction): void;
    removeEventListener(event: string, callback: CallableFunction): void;
    dispatchEvent<TMap extends {}, T extends Extract<keyof TMap, string>>(event: THREE.BaseEvent<T> & TMap[T]): void;
  }

  export class PlaneMesh extends Mesh {
    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);
  }

  export class HeightfieldMesh extends Mesh {
    constructor(
      geometry: THREE.BufferGeometry,
      material: THREE.Material | THREE.Material[],
      mass?: number,
      xdiv?: number,
      ydiv?: number
    );
  }

  export class BoxMesh extends Mesh {
    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);
  }

  export class SphereMesh extends Mesh {
    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);
  }

  export class CylinderMesh extends Mesh {
    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);
  }

  export class CapsuleMesh extends Mesh {
    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);
  }

  export class ConeMesh extends Mesh {
    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);
  }

  export class ConcaveMesh extends Mesh {
    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);
  }

  export class ConvexMesh extends Mesh {
    constructor(geometry: THREE.BufferGeometry, material: THREE.Material | THREE.Material[], mass?: number);
  }
}

// MARK: - Constraints
declare namespace Physijs {
  export class Constraint {
    constructor(objecta: Mesh, objectb?: Mesh, options?: any);
    getDefinition(): {};
  }

  export class PointConstraint extends Constraint {
    constructor(objecta: Mesh, position: THREE.Vector3);
    constructor(objecta: Mesh, objectb: Mesh, position: THREE.Vector3);
  }

  export class HingeConstraint extends Constraint {
    constructor(objecta: Mesh, position: THREE.Vector3, axis: THREE.Vector3);
    constructor(objecta: Mesh, objectb: Mesh, position: THREE.Vector3, axis: THREE.Vector3);

    setLimits(low: number, high: number, bias_factor?: number, relaxation_factor?: number): void;
    enableAngularMotor(velocity: number, acceleration: number): void;
    disableMotor(): void;
  }

  export class SliderConstraint extends Constraint {
    constructor(objecta: Mesh, position: THREE.Vector3, axis: THREE.Vector3);
    constructor(objecta: Mesh, objectb: Mesh, position: THREE.Vector3, axis: THREE.Vector3);

    setLimits(lin_lower: number, lin_upper: number, ang_lower: number, ang_upper: number): void;
    setRestitution(linear: number, angular: number): void;
    enableLinearMotor(velocity: number, acceleration: number): void;
    disableLinearMotor(): void;
    enableAngularMotor(velocity: number, acceleration: number): void;
    disableAngularMotor(): void;
  }

  export class ConeTwistConstraint extends Constraint {
    constructor(objecta: Mesh, objectb: Mesh, position: THREE.Vector3);

    setLimit(x: number, y: number, z: number): void;
    enableMotor(): void;
    setMaxMotorImpulse(max_impulse: number): void;
    setMotorTarget(target: THREE.Quaternion | THREE.Vector3 | THREE.Euler | THREE.Matrix4): void;
    disableMotor(): void;
  }

  export class DOFConstraint extends Constraint {
    constructor(objecta: Mesh, position: THREE.Vector3);
    constructor(objecta: Mesh, objectb: Mesh, position: THREE.Vector3);

    setLinearLowerLimit(limit: THREE.Vector3): void;
    setLinearUpperLimit(limit: THREE.Vector3): void;
    setAngularLowerLimit(limit: THREE.Vector3): void;
    setAngularUpperLimit(limit: THREE.Vector3): void;
    enableAngularMotor(which: number): void;
    configureAngularMotor(
      which: number,
      low_angle: THREE.Vector3,
      high_angle: THREE.Vector3,
      velocity: THREE.Vector3,
      max_force: THREE.Vector3
    ): void;
    disableAngularMotor(which: number): void;
  }
}

// MARK: - Vehicle
declare namespace Physijs {
  export class Vehicle {
    constructor(mesh: Mesh, tuning?: VehicleTuning);

    addWheel(
      wheel_geometry: THREE.BufferGeometry,
      wheel_material: THREE.Material | THREE.Material[],
      connection_point: THREE.Vector3,
      wheel_direction: THREE.Vector3,
      wheel_axle: THREE.Vector3,
      suspension_rest_length: number,
      wheel_radius: number,
      is_front_wheel: boolean,
      tuning?: VehicleTuning
    ): void
    setSteering(amount: number, wheel_index?: number): void
    setBrake(amount: number, wheel_index?: number): void
    applyEngineForce(amount: number, wheel_index?: number): void
  }

  export class VehicleTuning {
    constructor(
      suspension_stiffness?: number,
      suspension_compression?: number,
      suspension_damping?: number,
      max_suspension_travel?: number,
      friction_slip?: number,
      max_suspension_force?: number
    );

    suspension_stiffness: number;
    suspension_compression: number;
    suspension_damping: number;
    max_suspension_travel: number;
    friction_slip: number;
    max_suspension_force: number;
  }
}

declare global {
  interface Window {
    Physijs: typeof Physijs;
  }
}

export = Physijs;
export as namespace Physijs;
