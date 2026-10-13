/** Scene helpers: scale references, CG marker, force arrows, wind field, replay trail, camera rig. */
import { Html, useGLTF } from '@react-three/drei';
import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import {
  BufferGeometry, Color, Float32BufferAttribute, InstancedMesh, Line, LineBasicMaterial, MeshBasicMaterial, Object3D, Quaternion,
  Vector3,
} from 'three';
import type { Product } from '../data/schema';
import { foldQuaternion } from '../kinematics/hinge';
import type { CameraPreset } from '../store/app';

export function HumanSilhouette({ position }: { position: [number, number, number] }) {
  const mat = useMemo(() => new MeshBasicMaterial({ color: '#334155' }), []);
  return (
    <group position={position}>
      {[-0.1, 0.1].map((z) => (
        <mesh key={z} position={[0, 0.43, z]} material={mat}><capsuleGeometry args={[0.065, 0.74, 4, 8]} /></mesh>
      ))}
      <mesh position={[0, 1.18, 0]} material={mat}><capsuleGeometry args={[0.16, 0.42, 4, 12]} /></mesh>
      {[-0.24, 0.24].map((z) => (
        <mesh key={z} position={[0, 1.12, z]} rotation={[z > 0 ? -0.08 : 0.08, 0, 0]} material={mat}><capsuleGeometry args={[0.05, 0.56, 4, 8]} /></mesh>
      ))}
      <mesh position={[0, 1.69, 0]} material={mat}><sphereGeometry args={[0.11, 16, 12]} /></mesh>
      <Html position={[0, 1.95, 0]} center className="pointer-events-none select-none">
        <div className="whitespace-nowrap rounded bg-slate-900/70 px-1.5 py-0.5 text-[10px] text-slate-200">1,80 m</div>
      </Html>
    </group>
  );
}

export function CGMarker({ position, label }: { position: Vector3; label: string }) {
  const size = 0.07;
  return (
    <group position={position}>
      <mesh renderOrder={10}>
        <sphereGeometry args={[size, 20, 14]} />
        <meshBasicMaterial color="#facc15" depthTest={false} transparent opacity={0.95} />
      </mesh>
      {[0, 1, 2].map((i) => (
        <mesh key={i} rotation={i === 0 ? [0, 0, Math.PI / 2] : i === 2 ? [Math.PI / 2, 0, 0] : [0, 0, 0]} renderOrder={10}>
          <cylinderGeometry args={[size * 0.12, size * 0.12, size * 5, 6]} />
          <meshBasicMaterial color="#0f172a" depthTest={false} transparent opacity={0.8} />
        </mesh>
      ))}
      <Html position={[0, size * 3.2, 0]} center className="pointer-events-none select-none">
        <div className="whitespace-nowrap rounded bg-yellow-400/90 px-1.5 py-0.5 text-[10px] font-semibold text-slate-900">{label}</div>
      </Html>
    </group>
  );
}

export interface ForceVector {
  id: string;
  label: string;
  vectorN: Vector3;
  color: string;
  origin: Vector3;
}

/**
 * Arrow length = refLengthM · √(|F| / refForceN): square-root scaling keeps small forces (drag)
 * readable next to the weight while preserving ordering. Labels show the true magnitude.
 */
export function ForceArrows({ forces, refForceN, refLengthM }: { forces: ForceVector[]; refForceN: number; refLengthM: number }) {
  return (
    <group>
      {forces.map((f) => {
        const mag = f.vectorN.length();
        if (mag < 1e-3) return null;
        const len = Math.max(0.08, refLengthM * Math.sqrt(mag / refForceN));
        const dir = f.vectorN.clone().normalize();
        const q = new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), dir);
        const head = Math.min(0.25, len * 0.25);
        const r = Math.max(0.012, len * 0.018);
        return (
          <group key={f.id} position={f.origin} quaternion={q}>
            <mesh position={[0, (len - head) / 2, 0]} renderOrder={5}>
              <cylinderGeometry args={[r, r, len - head, 8]} />
              <meshBasicMaterial color={f.color} depthTest={false} transparent opacity={0.9} />
            </mesh>
            <mesh position={[0, len - head / 2, 0]} renderOrder={5}>
              <coneGeometry args={[r * 2.8, head, 12]} />
              <meshBasicMaterial color={f.color} depthTest={false} transparent opacity={0.9} />
            </mesh>
            <Html position={[0, len + 0.08, 0]} center className="pointer-events-none select-none">
              <div className="whitespace-nowrap rounded px-1.5 py-0.5 text-[10px] font-medium text-white" style={{ background: f.color + 'cc' }}>
                {f.label}: {mag >= 1000 ? `${(mag / 1000).toFixed(2)} kN` : `${mag.toFixed(0)} N`}
              </div>
            </Html>
          </group>
        );
      })}
    </group>
  );
}

/** Instanced streak field advected by the relative air velocity; jitter grows with turbulence. */
export function WindField({ flowRef, extentM, turbulence, center }: {
  flowRef: React.MutableRefObject<Vector3>;
  extentM: number;
  turbulence: number;
  center: Vector3;
}) {
  const N = 450;
  const ref = useRef<InstancedMesh>(null);
  const seeds = useMemo(() => {
    let s = 12345;
    const r = () => ((s = (s * 16807) % 2147483647) / 2147483647);
    return Array.from({ length: N }, () => ({ p: new Vector3((r() - 0.5) * 2, r() * 0.9, (r() - 0.5) * 2), ph: r() * 100 }));
  }, []);
  const tmp = useMemo(() => new Object3D(), []);
  const up = useMemo(() => new Vector3(1, 0, 0), []);
  const mat = useMemo(() => new MeshBasicMaterial({ color: new Color('#7dd3fc'), transparent: true, opacity: 0.38, depthWrite: false }), []);
  useFrame(({ clock }, dt) => {
    const mesh = ref.current;
    if (!mesh) return;
    const v = flowRef.current;
    const speed = v.length();
    mesh.visible = speed > 0.3;
    if (!mesh.visible) return;
    const dir = v.clone().divideScalar(speed);
    const q = new Quaternion().setFromUnitVectors(up, dir);
    const visual = Math.min(speed, 40) * 0.25; // slowed down for readability
    const len = 0.12 + Math.min(0.6, speed * 0.025);
    const t = clock.elapsedTime;
    seeds.forEach((s, i) => {
      s.p.addScaledVector(dir, (visual * dt) / extentM);
      for (const k of ['x', 'z'] as const) {
        if (s.p[k] > 1) s.p[k] -= 2;
        if (s.p[k] < -1) s.p[k] += 2;
      }
      if (s.p.y > 0.95) s.p.y -= 0.9;
      if (s.p.y < 0.05) s.p.y += 0.9;
      const j = turbulence * 0.04 * extentM;
      tmp.position.set(
        center.x + s.p.x * extentM + Math.sin(t * 3.1 + s.ph) * j,
        center.y + s.p.y * extentM * 0.6 + Math.sin(t * 2.3 + s.ph * 1.7) * j,
        center.z + s.p.z * extentM + Math.cos(t * 2.7 + s.ph) * j,
      );
      tmp.quaternion.copy(q);
      tmp.scale.set(len, 1, 1);
      tmp.updateMatrix();
      mesh.setMatrixAt(i, tmp.matrix);
    });
    mesh.instanceMatrix.needsUpdate = true;
  });
  return (
    <instancedMesh ref={ref} args={[undefined, undefined, N]} material={mat} frustumCulled={false}>
      <boxGeometry args={[1, 0.012, 0.012]} />
    </instancedMesh>
  );
}

/** Replay trail line; points are provided already in scene coordinates. */
export function Trail({ points, color = '#f472b6' }: { points: Float32Array; color?: string }) {
  const line = useMemo(() => new Line(new BufferGeometry(), new LineBasicMaterial({ color, transparent: true, opacity: 0.9 })), [color]);
  useEffect(() => {
    line.geometry.setAttribute('position', new Float32BufferAttribute(points, 3));
    line.geometry.computeBoundingSphere();
  }, [points, line]);
  useEffect(() => () => { line.geometry.dispose(); (line.material as LineBasicMaterial).dispose(); }, [line]);
  return <primitive object={line} frustumCulled={false} />;
}

export const CAMERA_DIRECTIONS: Record<CameraPreset, [number, number, number]> = {
  front: [1, 0.12, 0],
  side: [0, 0.12, 1],
  top: [0.001, 1, 0],
  iso: [0.8, 0.6, 0.9],
};

/** Applies camera presets; optional shared camera state for synchronised views. */
export function CameraRig({ request, distance, target, sync }: {
  request: { preset: CameraPreset; n: number } | null;
  distance: number;
  target: Vector3;
  sync?: { role: 'master' | 'slave'; state: { pos: Vector3; target: Vector3; valid: boolean } };
}) {
  const { camera, controls } = useThree() as unknown as { camera: import('three').PerspectiveCamera; controls: { target: Vector3; update: () => void } | null };
  useEffect(() => {
    if (!request) return;
    const d = new Vector3(...CAMERA_DIRECTIONS[request.preset]).normalize().multiplyScalar(distance);
    camera.position.copy(target.clone().add(d));
    if (controls) {
      controls.target.copy(target);
      controls.update();
    } else camera.lookAt(target);
  }, [request?.n]); // eslint-disable-line react-hooks/exhaustive-deps
  useFrame(() => {
    if (!sync) return;
    if (sync.role === 'master') {
      sync.state.pos.copy(camera.position);
      if (controls) sync.state.target.copy(controls.target);
      sync.state.valid = true;
    } else if (sync.state.valid) {
      camera.position.copy(sync.state.pos);
      if (controls) {
        controls.target.copy(sync.state.target);
        controls.update();
      } else camera.lookAt(sync.state.target);
    }
  });
  return null;
}

/**
 * GLB model path: node names must match parts[].id; hinge nodes `hinge_left` / `hinge_right`
 * are rotated about the data-defined hinge axis. Parts get their data record in userData.
 */
export function GltfAircraft({ product, url, foldDeg }: { product: Product; url: string; foldDeg: number }) {
  const gltf = useGLTF(url);
  const scene = useMemo(() => gltf.scene.clone(true), [gltf]);
  const base = useRef(new Map<string, Quaternion>());
  useEffect(() => {
    scene.traverse((o) => {
      const part = product.parts.find((p) => p.id === o.name);
      if (part) o.userData.part = part;
      if (o.name === 'hinge_left' || o.name === 'hinge_right') base.current.set(o.name, o.quaternion.clone());
    });
  }, [scene, product]);
  useFrame(() => {
    for (const [name, side] of [['hinge_left', 'L'], ['hinge_right', 'R']] as const) {
      const o = scene.getObjectByName(name);
      const q0 = base.current.get(name);
      if (o && q0) o.quaternion.copy(foldQuaternion(product.geometry.hinge, side, foldDeg)).multiply(q0);
    }
  });
  return <primitive object={scene} />;
}

/** Reference ring showing the commanded hover position during replay. */
export function TargetMarker({ position, radius }: { position: Vector3; radius: number }) {
  return (
    <mesh position={position} rotation={[-Math.PI / 2, 0, 0]}>
      <ringGeometry args={[radius * 0.92, radius, 48]} />
      <meshBasicMaterial color="#22d3ee" transparent opacity={0.6} side={2} />
    </mesh>
  );
}

