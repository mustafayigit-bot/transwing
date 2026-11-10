/**
 * 3-D replay of a 6-DOF mission. Floating origin: the aircraft stays at the scene origin and the
 * world (ground, route, flown path) is shifted by −position, so precision holds over tens of km.
 * Scene axes = simulation world axes: x north, y up, z east.
 */
import { Grid, Html, OrbitControls } from '@react-three/drei';
import { Canvas, useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import { BufferGeometry, Float32BufferAttribute, Group, Line, LineBasicMaterial } from 'three';
import type { PayloadOption, Product } from '../data/schema';
import type { MissionTrace } from '../physics/fdm/mission';
import { Aircraft, groundClearance } from './Aircraft';
import { StageLights } from './Viewer';

const DEG = Math.PI / 180;

function PathLine({ pts, color, opacity = 1 }: { pts: Float32Array; color: string; opacity?: number }) {
  const line = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute('position', new Float32BufferAttribute(pts, 3));
    const l = new Line(g, new LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
    l.frustumCulled = false;
    return l;
  }, [pts, color, opacity]);
  return <primitive object={line} />;
}

function World({ trace, index, clearance }: { trace: MissionTrace; index: number; clearance: number }) {
  const ref = useRef<Group>(null);
  const gridRef = useRef<Group>(null);
  const c = trace.ch;
  const flown = useMemo(() => {
    const n = c.t.length;
    const step = Math.max(1, Math.floor(n / 6000));
    const a: number[] = [];
    for (let i = 0; i < n; i += step) a.push(c.pn[i], c.alt[i], c.pe[i]);
    return new Float32Array(a);
  }, [c]);
  const shadow = useMemo(() => {
    const a = new Float32Array(flown.length);
    for (let i = 0; i < flown.length; i += 3) { a[i] = flown[i]; a[i + 1] = 0.05; a[i + 2] = flown[i + 2]; }
    return a;
  }, [flown]);
  const route = useMemo(() => {
    const w = trace.config.waypointsM;
    const h = trace.config.cruiseAltitudeM;
    const a = [0, h, 0];
    for (const p of w) a.push(p[0], h, p[1]);
    a.push(0, h, 0);
    return new Float32Array(a);
  }, [trace]);
  useFrame(() => {
    const pn = c.pn[index] ?? 0, pe = c.pe[index] ?? 0, alt = c.alt[index] ?? 0;
    if (ref.current) ref.current.position.set(-pn, -alt - clearance, -pe);
    // keep the grid pattern anchored to the ground while following the aircraft
    if (gridRef.current) gridRef.current.position.set(pn - (pn % 10), 0, pe - (pe % 10));
  });
  return (
    <group ref={ref}>
      <group ref={gridRef}>
        <Grid args={[400, 400]} cellSize={10} cellThickness={0.7} cellColor="#334155" sectionSize={100} sectionThickness={1.2} sectionColor="#475569" fadeDistance={900} infiniteGrid />
      </group>
      <PathLine pts={route} color="#22d3ee" opacity={0.5} />
      <PathLine pts={flown} color="#f472b6" />
      <PathLine pts={shadow} color="#64748b" opacity={0.6} />
      {/* launch / landing pad */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.03, 0]}>
        <ringGeometry args={[2.6, 3, 48]} />
        <meshBasicMaterial color="#facc15" />
      </mesh>
      {trace.config.waypointsM.map((p, i) => (
        <group key={i} position={[p[0], 0, p[1]]}>
          <mesh position={[0, trace.config.cruiseAltitudeM / 2, 0]}>
            <cylinderGeometry args={[0.4, 0.4, trace.config.cruiseAltitudeM, 6]} />
            <meshBasicMaterial color="#22d3ee" transparent opacity={0.35} />
          </mesh>
          <Html position={[0, trace.config.cruiseAltitudeM + 8, 0]} center className="pointer-events-none select-none">
            <div className="rounded bg-cyan-500/80 px-1.5 text-[10px] font-semibold text-slate-900">WP{i + 1}</div>
          </Html>
        </group>
      ))}
    </group>
  );
}

export function MissionViewer({ product, payload, trace, index }: { product: Product; payload: PayloadOption | null; trace: MissionTrace; index: number }) {
  const c = trace.ch;
  const i = Math.min(index, c.t.length - 1);
  const span = product.geometry.spanM;
  const clearance = groundClearance(product);
  const nominal = (c.thrust1.length ? (product.mass.emptyKg + product.mass.batteryKg + (payload?.massKg ?? 0)) * 9.80665 : 1) / trace.nRotors;
  const load: number[] = [], folded: boolean[] = [];
  for (let k = 0; k < trace.nRotors; k++) {
    const rpm = c[`rpm${k + 1}`][i] ?? 0;
    load.push(Math.max(0, (c[`thrust${k + 1}`][i] ?? 0) / nominal));
    folded.push(rpm < 150 && (c.fold[i] ?? 0) < 2 && (c.ias[i] ?? 0) > 10);
    if (rpm > 150 && load[k] < 0.03) load[k] = 0.03;
  }
  const rot: [number, number, number, 'YZX'] = [(c.roll[i] ?? 0) * DEG, -(c.yaw[i] ?? 0) * DEG, (c.pitch[i] ?? 0) * DEG, 'YZX'];
  return (
    <Canvas gl={{ antialias: true, preserveDrawingBuffer: true }} camera={{ position: [-span * 1.6, span * 0.7, span * 1.2], fov: 45, near: 0.1, far: 60000 }}>
      <color attach="background" args={['#0b1220']} />
      <fog attach="fog" args={['#0b1220', 600, 6000]} />
      <StageLights />
      <World trace={trace} index={i} clearance={clearance} />
      <group rotation={rot}>
        <Aircraft product={product} payload={payload} foldDeg={c.fold[i] ?? 0} smoothFold={false} rotorLoad={load} propsFolded={folded} rotorSaturated={c.sat[i] ? load.map((l) => l > 1.2) : undefined} />
      </group>
      <OrbitControls makeDefault target={[0, 0, 0]} enableDamping maxDistance={2500} />
    </Canvas>
  );
}
