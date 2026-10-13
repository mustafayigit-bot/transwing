/** Main configurator 3D view (store-driven) and a light-weight reusable viewer. */
import { Grid, OrbitControls } from '@react-three/drei';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { Suspense, useEffect, useMemo, useRef, type ReactNode } from 'react';
import { Plane, Vector3 } from 'three';
import type { PayloadOption, Product } from '../data/schema';
import { thrustDirection } from '../kinematics/hinge';
import { projectedArea, windToDirection } from '../physics/airframe';
import { atmosphere } from '../physics/atmosphere';
import { cruisePoint } from '../physics/cruise';
import { hoverPerformance } from '../physics/hover';
import { massProperties } from '../physics/massProps';
import type { FlightMode } from '../physics/types';
import { windAtHeight } from '../physics/wind';
import { activeTrace, frameIndex, useApp, type CameraPreset } from '../store/app';
import { useCurrentProduct, usePayload } from '../store/results';
import { Aircraft, groundClearance } from './Aircraft';
import { CameraRig, CGMarker, ForceArrows, GltfAircraft, HumanSilhouette, TargetMarker, Trail, WindField, type ForceVector } from './Helpers';

// ------------------------------------------------------------------------------------------------
// Screenshot bridge
let captureFn: (() => string) | null = null;
export function captureScreenshot(): string | null {
  return captureFn ? captureFn() : null;
}
function ScreenshotBridge() {
  const { gl, scene, camera } = useThree();
  useEffect(() => {
    captureFn = () => {
      gl.render(scene, camera);
      return gl.domElement.toDataURL('image/png');
    };
    return () => { captureFn = null; };
  }, [gl, scene, camera]);
  return null;
}

function ClippingEnabler() {
  const { gl } = useThree();
  useEffect(() => { gl.localClippingEnabled = true; }, [gl]);
  return null;
}

export function StageLights() {
  return (
    <>
      <ambientLight intensity={0.55} />
      <hemisphereLight args={['#dbeafe', '#1e293b', 0.6]} />
      <directionalLight position={[6, 10, 5]} intensity={1.6} castShadow shadow-mapSize={[2048, 2048]} shadow-camera-left={-8} shadow-camera-right={8} shadow-camera-top={8} shadow-camera-bottom={-8} />
      <directionalLight position={[-6, 4, -6]} intensity={0.5} />
    </>
  );
}

/** Rotor load for spin speed from a static evaluation of the flight mode. */
export function staticRotorState(product: Product, payload: PayloadOption | null, mode: FlightMode, foldDeg: number, cruiseMotors: 2 | 4, altitudeM: number) {
  const hover = hoverPerformance({ product, payload, altitudeM, isaDeltaK: 0 }).value;
  const nominal = hover.weightN / product.geometry.rotors.length;
  const inCruise = foldDeg < 1;
  const folded = product.geometry.rotors.map((r) => inCruise && cruiseMotors === 2 && r.foldableInCruise);
  let load: number[];
  if (mode === 'cruise' || inCruise) {
    const v = 25;
    const pt = cruisePoint({ product, payload, altitudeM, isaDeltaK: 0, motors: cruiseMotors }, v);
    const nAct = folded.filter((f) => !f).length;
    load = folded.map((f) => (f ? 0 : pt.dragN / nAct / nominal));
  } else {
    load = hover.rotorThrustN.map((t) => t / nominal);
  }
  return { load, folded };
}

// ------------------------------------------------------------------------------------------------
function ConfigScene() {
  const product = useCurrentProduct()!;
  const s = useApp();
  const payload = usePayload(product, s.payloadId);
  const trace = activeTrace(s);
  const replaying = !!trace && trace.t.length > 0;
  const i = replaying ? frameIndex(trace!, s.replayT) : 0;
  const frameFold = replaying && Number.isFinite(trace!.foldDeg[i]) ? trace!.foldDeg[i] : null;
  const fold = frameFold ?? s.foldDeg;
  const clearance = groundClearance(product);
  const span = product.geometry.spanM;

  // ---- replay offsets
  const offset = useMemo(() => {
    if (!replaying) return new Vector3();
    const p = trace!.pos[i], r = trace!.refPos;
    if (trace!.mode !== 'hover' && trace!.source === 'internal') return new Vector3(0, p[1] - r[1], 0);
    return new Vector3(p[0] - r[0], p[1] - r[1], p[2] - r[2]);
  }, [replaying, trace, i]);
  const att = replaying ? trace!.att[i] : ([0, 0, 0] as [number, number, number]);
  const basePos = new Vector3(0, clearance, 0).add(offset);

  // ---- rotors
  const rotor = useMemo(() => {
    if (replaying && trace!.thrustN[i]?.length) {
      const hover = hoverPerformance({ product, payload, altitudeM: s.env.altitudeM, isaDeltaK: s.env.isaDeltaK }).value;
      const nominal = hover.weightN / product.geometry.rotors.length;
      const th = trace!.thrustN[i];
      const folded = product.geometry.rotors.map((r, k) => fold < 1 && r.foldableInCruise && (th[k] ?? 0) === 0);
      return { load: product.geometry.rotors.map((_, k) => (th[k] ?? 0) / nominal), folded, saturated: trace!.saturated[i] ?? [] };
    }
    const st = staticRotorState(product, payload, s.flightMode, fold, s.cruiseMotors, s.env.altitudeM);
    return { ...st, saturated: [] as boolean[] };
  }, [replaying, trace, i, product, payload, fold, s.flightMode, s.cruiseMotors, s.env.altitudeM, s.env.isaDeltaK]);

  // ---- CG
  const foldKey = Math.round(fold * 2) / 2;
  const mp = useMemo(() => massProperties(product, payload, foldKey).value, [product, payload, foldKey]);
  const cgLocal = new Vector3(...mp.cgM);

  // ---- forces (scene frame, unrotated body)
  const forces = useMemo<ForceVector[]>(() => {
    const W = mp.totalKg * 9.80665;
    const rho = atmosphere(s.env.altitudeM, s.env.isaDeltaK).value.densityKgM3;
    const tdir = new Vector3();
    product.geometry.rotors.forEach((r) => tdir.add(thrustDirection(product.geometry.hinge, r.wing, fold)));
    tdir.normalize();
    const out: ForceVector[] = [{ id: 'w', label: 'Ağırlık', vectorN: new Vector3(0, -W, 0), color: '#ef4444', origin: cgLocal.clone() }];
    let thrust = 0;
    let windVec = new Vector3();
    if (replaying) {
      thrust = (trace!.thrustN[i] ?? []).reduce((a, b) => a + b, 0);
      const w = trace!.windMs[i];
      windVec = new Vector3(w[0], w[1], w[2]);
    } else {
      const meanW = windAtHeight(s.env.windSpeedMs, 30, s.env.logProfile);
      windVec = windToDirection(s.env.windFromDeg).multiplyScalar(meanW);
    }
    const wmag = windVec.length();
    // wind load is shown for the (near-)hover configuration; in wing-borne flight it is part of the drag
    if (fold > 0.6 * product.geometry.hinge.maxFoldDeg) {
      const area = wmag > 0.01 ? projectedArea(product, fold, windVec.clone().normalize()).areaM2 : 0;
      const Dw = 0.5 * rho * wmag * wmag * product.aero.sideCd * area;
      if (!replaying) thrust = Math.hypot(W, Dw);
      if (Dw > 1) out.push({ id: 'wind', label: 'Rüzgâr yükü', vectorN: windVec.clone().normalize().multiplyScalar(Dw), color: '#38bdf8', origin: cgLocal.clone() });
    }
    if (fold < product.geometry.hinge.maxFoldDeg - 1) {
      const v = replaying && Number.isFinite(trace!.airspeedMs[i]) ? trace!.airspeedMs[i] : 25;
      const pt = cruisePoint({ product, payload, altitudeM: s.env.altitudeM, isaDeltaK: s.env.isaDeltaK, motors: s.cruiseMotors }, Math.max(v, 1));
      const frac = 1 - fold / product.geometry.hinge.maxFoldDeg;
      out.push({ id: 'drag', label: 'Sürüklenme', vectorN: new Vector3(-pt.dragN * frac, 0, 0), color: '#a855f7', origin: cgLocal.clone() });
      if (!replaying) thrust = fold < 1 ? pt.dragN : Math.max(thrust, pt.dragN);
      if (fold < 1) out.push({ id: 'lift', label: 'Kaldırma', vectorN: new Vector3(0, W, 0), color: '#22c55e', origin: cgLocal.clone().add(new Vector3(0.02, 0, 0)) });
    }
    out.push({ id: 't', label: 'Toplam itki', vectorN: tdir.clone().multiplyScalar(thrust), color: '#f97316', origin: cgLocal.clone().add(new Vector3(0, 0.02, 0)) });
    return out;
  }, [mp, product, payload, fold, replaying, trace, i, s.env, s.cruiseMotors]); // eslint-disable-line react-hooks/exhaustive-deps

  // ---- wind flow (air velocity relative to the vehicle)
  const flowRef = useRef(new Vector3());
  useFrame(() => {
    if (replaying) {
      const w = trace!.windMs[i];
      const flow = new Vector3(w[0], w[1], w[2]);
      if (trace!.mode !== 'hover' && i > 0) {
        const p0 = trace!.pos[i - 1], p1 = trace!.pos[i];
        const dt = Math.max(1e-3, trace!.t[i] - trace!.t[i - 1]);
        flow.sub(new Vector3((p1[0] - p0[0]) / dt, 0, (p1[2] - p0[2]) / dt));
      }
      flowRef.current.copy(flow);
    } else {
      const meanW = windAtHeight(s.env.windSpeedMs, 30, s.env.logProfile);
      const flow = windToDirection(s.env.windFromDeg).multiplyScalar(meanW);
      if (s.flightMode === 'cruise') flow.add(new Vector3(-25, 0, 0));
      flowRef.current.copy(flow);
    }
  });
  const turbLevel = { none: 0, light: 0.6, moderate: 1.2, severe: 2 }[s.env.turbulence];

  // ---- trail
  const trail = useMemo(() => {
    if (!replaying) return null;
    const pts: number[] = [];
    const r = trace!.refPos;
    const fwd = trace!.mode !== 'hover' && trace!.source === 'internal';
    const xi = trace!.pos[i][0];
    for (let k = 0; k <= i; k += 1) {
      const p = trace!.pos[k];
      if (fwd) pts.push(p[0] - xi, p[1] - r[1] + clearance, 0);
      else pts.push(p[0] - r[0], p[1] - r[1] + clearance, p[2] - r[2]);
    }
    return new Float32Array(pts);
  }, [replaying, trace, i, clearance]);

  // ---- section plane
  const clipping = useMemo(() => (s.sectionEnabled ? [new Plane(new Vector3(-1, 0, 0), s.sectionOffset)] : undefined), [s.sectionEnabled, s.sectionOffset]);

  // ---- attitude: roll about x, pitch about z, yaw about -y (order YZX)
  const DEG = Math.PI / 180;
  const rot: [number, number, number] = [att[0] * DEG, -att[2] * DEG, att[1] * DEG];
  const glb = product.modelUrl;

  return (
    <>
      <group position={basePos}>
        <group rotation={[rot[0], rot[1], rot[2], 'YZX']}>
          {glb ? (
            <Suspense fallback={null}>
              <GltfAircraft product={product} url={glb} foldDeg={fold} />
            </Suspense>
          ) : (
            <Aircraft
              product={product}
              payload={payload}
              foldDeg={fold}
              smoothFold={!replaying && !s.transitionPlaying}
              explode={s.explode}
              wireframe={s.wireframe}
              clipping={clipping}
              hiddenParts={s.hiddenParts}
              isolatedPart={s.isolatedPart}
              selectedPart={s.selectedPart}
              hoveredPart={s.hoveredPart}
              showConnectors={s.showConnectors}
              interactive
              rotorLoad={rotor.load}
              rotorSaturated={rotor.saturated}
              propsFolded={rotor.folded}
              onHover={(id) => useApp.setState({ hoveredPart: id })}
              onSelect={(id) => useApp.setState({ selectedPart: id })}
            />
          )}
          {s.showCG && <CGMarker position={cgLocal} label={`CG ${mp.totalKg.toFixed(1)} kg`} />}
          {s.showForces && <ForceArrows forces={forces} refForceN={mp.totalKg * 9.80665} refLengthM={0.3 * span} />}
        </group>
      </group>
      {replaying && <TargetMarker position={new Vector3(0, clearance, 0)} radius={0.25 * span} />}
      {trail && <Trail points={trail} />}
      {s.showWind && <WindField flowRef={flowRef} extentM={span * 1.1} turbulence={turbLevel} center={new Vector3(0, 0.1, 0)} />}
      {s.showHuman && <HumanSilhouette position={[0, 0, -(span / 2 + 1.2)]} />}
    </>
  );
}

function Drivers() {
  // transition animation hover → cruise → hover (ping-pong) and replay clock
  const t0 = useRef(0);
  useFrame((_, dt) => {
    const s = useApp.getState();
    const product = s.specs?.products.find((p) => p.id === s.productId);
    if (s.transitionPlaying && product) {
      t0.current += dt;
      const T = Math.max(0.5, s.transitionDurationS);
      const ph = (t0.current % (2 * T)) / T; // 0..2
      const u = ph <= 1 ? ph : 2 - ph;
      const ease = u * u * (3 - 2 * u);
      const fold = product.geometry.hinge.maxFoldDeg * (1 - ease);
      useApp.setState({ foldDeg: fold });
      if (!s.presentation && t0.current >= 2 * T) {
        t0.current = 0;
        useApp.setState({ transitionPlaying: false, foldDeg: product.geometry.hinge.maxFoldDeg });
      }
    } else t0.current = 0;
    const tr = activeTrace(s);
    if (s.replayPlaying && tr && tr.t.length) {
      const end = tr.t[tr.t.length - 1];
      let t = s.replayT + dt * s.replaySpeed;
      if (t > end) {
        if (s.presentation) t = 0;
        else {
          useApp.setState({ replayT: end, replayPlaying: false });
          return;
        }
      }
      useApp.setState({ replayT: t });
    }
  });
  return null;
}

export function ConfigViewer() {
  const product = useCurrentProduct();
  const cameraRequest = useApp((s) => s.cameraRequest);
  const presentation = useApp((s) => s.presentation);
  if (!product) return null;
  const span = product.geometry.spanM;
  const target = new Vector3(0, groundClearance(product), 0);
  return (
    <Canvas
      shadows
      gl={{ preserveDrawingBuffer: true, antialias: true }}
      camera={{ position: [span * 0.9, span * 0.6, span * 1.0], fov: 40, near: 0.05, far: 2000 }}
      onPointerMissed={() => useApp.setState({ selectedPart: null })}
    >
      <color attach="background" args={['#0b1220']} />
      <fog attach="fog" args={['#0b1220', span * 4, span * 12]} />
      <StageLights />
      <ClippingEnabler />
      <ScreenshotBridge />
      <Drivers />
      <Grid args={[60, 60]} cellSize={1} cellThickness={0.6} cellColor="#334155" sectionSize={5} sectionThickness={1.1} sectionColor="#475569" fadeDistance={span * 8} infiniteGrid position={[0, 0, 0]} />
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.002, 0]} receiveShadow>
        <planeGeometry args={[200, 200]} />
        <shadowMaterial opacity={0.35} />
      </mesh>
      <ConfigScene />
      <OrbitControls makeDefault target={target} autoRotate={presentation} autoRotateSpeed={0.6} enableDamping maxPolarAngle={Math.PI * 0.495} />
      <CameraRig request={cameraRequest} distance={span * 1.6} target={target} />
    </Canvas>
  );
}

/** Generic viewer for compare / matrix previews. */
export function MiniViewer({ product, payload, foldDeg, sync, cameraRequest, autoRotate, interactive = true, children, frameloop = 'always' }: {
  product: Product;
  payload: PayloadOption | null;
  foldDeg: number;
  sync?: { role: 'master' | 'slave'; state: { pos: Vector3; target: Vector3; valid: boolean } };
  cameraRequest?: { preset: CameraPreset; n: number } | null;
  autoRotate?: boolean;
  interactive?: boolean;
  children?: ReactNode;
  frameloop?: 'always' | 'demand';
}) {
  const span = product.geometry.spanM;
  const target = new Vector3(0, groundClearance(product), 0);
  const rotor = useMemo(() => staticRotorState(product, payload, foldDeg > 1 ? 'hover' : 'cruise', foldDeg, 2, 0), [product, payload, foldDeg]);
  return (
    <Canvas frameloop={frameloop} gl={{ preserveDrawingBuffer: true, antialias: true }} camera={{ position: [span * 0.9, span * 0.6, span], fov: 40, near: 0.05, far: 1000 }}>
      <color attach="background" args={['#0b1220']} />
      <StageLights />
      <Grid args={[40, 40]} cellSize={1} cellThickness={0.5} cellColor="#334155" sectionSize={5} sectionColor="#475569" fadeDistance={span * 6} infiniteGrid />
      <group position={target}>
        <Aircraft product={product} payload={payload} foldDeg={foldDeg} rotorLoad={rotor.load} propsFolded={rotor.folded} />
      </group>
      <HumanSilhouette position={[0, 0, -(span / 2 + 1.2)]} />
      {children}
      <OrbitControls makeDefault target={target} enabled={interactive && sync?.role !== 'slave'} autoRotate={autoRotate} autoRotateSpeed={1} />
      <CameraRig request={cameraRequest ?? null} distance={span * 1.6} target={target} sync={sync} />
    </Canvas>
  );
}
