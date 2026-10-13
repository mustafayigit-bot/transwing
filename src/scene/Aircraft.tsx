/**
 * Procedural Specter aircraft built from product data. Every part is its own group carrying its
 * data record in `userData`. Wing-attached parts sit below the inclined hinge pivot nodes
 * (`hinge_left` / `hinge_right`) so a single rotation folds wing, nacelles and rotors together.
 */
import { Outlines, RoundedBox } from '@react-three/drei';
import { useFrame, type ThreeEvent } from '@react-three/fiber';
import { memo, useEffect, useMemo, useRef, type ReactNode } from 'react';
import {
  BufferGeometry, CylinderGeometry, DoubleSide, Float32BufferAttribute, Group, Line, LineBasicMaterial, Material,
  MeshStandardMaterial, Plane, Quaternion, SphereGeometry, Vector3, type Mesh,
} from 'three';
import type { Part, PayloadOption, Product } from '../data/schema';
import { exposedSemiSpanM, foldQuaternion, hingeAxis, hingePivot, rotorHubCruise } from '../kinematics/hinge';
import { bladeGeometry, fuselageGeometry, loftAirfoil, noseGeometry } from './geometry';
import { GROUP_COLORS, KIND_COLORS } from './materials';

export interface AircraftProps {
  product: Product;
  payload: PayloadOption | null;
  foldDeg: number;
  explode?: number;
  wireframe?: boolean;
  clipping?: Plane[];
  hiddenParts?: string[];
  isolatedPart?: string | null;
  selectedPart?: string | null;
  hoveredPart?: string | null;
  interactive?: boolean;
  showConnectors?: boolean;
  /** relative rotor load (thrust / nominal hover thrust) for spin speed; 0 = stopped */
  rotorLoad?: number[];
  rotorSaturated?: boolean[];
  /** foldable props fold against the nacelle (2-motor cruise) */
  propsFolded?: boolean[];
  onHover?: (id: string | null) => void;
  onSelect?: (id: string | null) => void;
  /** smooth (damped) fold animation; disable for replay */
  smoothFold?: boolean;
}

export function noseLength(p: Product) {
  return 0.12 * p.geometry.fuselageLengthM;
}

export function groundClearance(p: Product) {
  return p.geometry.fuselageDiameterM / 2 + p.geometry.landingGearHeightM;
}

/** Geometric centre of a part in cruise frame (used for explode direction and labels). */
export function partCenter(product: Product, part: Part): Vector3 {
  if (part.kind === 'wing') {
    const side = part.attach === 'wingL' ? 'L' : 'R';
    const pv = hingePivot(product.geometry.hinge, side);
    return new Vector3(pv.x - 0.2 * product.geometry.chordRootM, pv.y, pv.z + (side === 'R' ? 1 : -1) * 0.5 * exposedSemiSpanM(product));
  }
  return new Vector3(...part.positionM);
}

/** Explode origin: body-frame origin (fuselage mid-point). */
const ASSEMBLY_CENTER = new Vector3(0, 0, 0);

// ------------------------------------------------------------------------------------------------
function usePartMaterial(part: Part, props: AircraftProps, colorOverride?: string) {
  const color = colorOverride ?? KIND_COLORS[part.kind] ?? GROUP_COLORS[part.group];
  const mat = useMemo(
    () =>
      new MeshStandardMaterial({
        color,
        metalness: part.kind === 'hinge' ? 0.7 : part.group === 'Propulsion' ? 0.45 : 0.15,
        roughness: part.kind === 'wing' || part.kind === 'fuselage' ? 0.42 : 0.55,
        side: DoubleSide,
      }),
    [color, part.kind, part.group],
  );
  const ghost = !!props.isolatedPart && props.isolatedPart !== part.id;
  useEffect(() => {
    mat.wireframe = !!props.wireframe;
    mat.clippingPlanes = props.clipping ?? null;
    mat.clipShadows = true;
    mat.transparent = ghost;
    mat.opacity = ghost ? 0.08 : 1;
    mat.depthWrite = !ghost;
    mat.emissive.set(props.selectedPart === part.id ? '#2563eb' : '#000000');
    mat.emissiveIntensity = props.selectedPart === part.id ? 0.55 : 0;
    mat.needsUpdate = true;
  }, [mat, props.wireframe, props.clipping, ghost, props.selectedPart, part.id]);
  useEffect(() => () => mat.dispose(), [mat]);
  return mat;
}

function Highlight({ active, selected }: { active: boolean; selected: boolean }) {
  if (!active && !selected) return null;
  return <Outlines thickness={selected ? 0.012 : 0.008} color={selected ? '#3b82f6' : '#facc15'} screenspace={false} angle={0} />;
}

interface PartNodeProps {
  part: Part;
  product: Product;
  ap: AircraftProps;
  children: (mat: Material, hl: ReactNode) => ReactNode;
  color?: string;
}

/** Part wrapper: data in userData, pointer events, explode offset animation, connector line. */
function PartNode({ part, product, ap, children, color }: PartNodeProps) {
  const ref = useRef<Group>(null);
  const mat = usePartMaterial(part, ap, color);
  const center = useMemo(() => partCenter(product, part), [product, part]);
  const dir = useMemo(() => {
    const d = center.clone().sub(ASSEMBLY_CENTER);
    // keep internal parts (battery, avionics ...) moving visibly: add a vertical bias
    if (part.attach === 'body' && part.kind !== 'fuselage') d.y += part.positionM[1] >= 0 ? 0.6 : -0.6;
    if (part.kind === 'fuselage') d.set(0, 0, 0);
    return d.multiplyScalar(1.3);
  }, [center, product, part]);
  const lineGeom = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute('position', new Float32BufferAttribute([center.x, center.y, center.z, center.x, center.y, center.z], 3));
    return g;
  }, [center]);
  const line = useMemo(() => {
    const l = new Line(lineGeom, new LineBasicMaterial({ color: '#94a3b8', transparent: true, opacity: 0.7 }));
    l.visible = false;
    return l;
  }, [lineGeom]);
  useEffect(() => () => { lineGeom.dispose(); (line.material as Material).dispose(); }, [lineGeom, line]);
  useFrame((_, dt) => {
    const g = ref.current;
    if (!g) return;
    const target = dir.clone().multiplyScalar(ap.explode ?? 0);
    g.position.lerp(target, 1 - Math.exp(-8 * dt));
    const a = lineGeom.getAttribute('position') as Float32BufferAttribute;
    a.setXYZ(1, center.x + g.position.x, center.y + g.position.y, center.z + g.position.z);
    a.needsUpdate = true;
    line.visible = !!ap.showConnectors && g.position.lengthSq() > 1e-4;
  });
  const hidden = ap.hiddenParts?.includes(part.id);
  const hovered = ap.hoveredPart === part.id;
  const selected = ap.selectedPart === part.id;
  const ghost = !!ap.isolatedPart && ap.isolatedPart !== part.id;
  const events = ap.interactive && !ghost
    ? {
        onPointerOver: (e: ThreeEvent<PointerEvent>) => { e.stopPropagation(); ap.onHover?.(part.id); },
        onPointerOut: (e: ThreeEvent<PointerEvent>) => { e.stopPropagation(); ap.onHover?.(null); },
        onClick: (e: ThreeEvent<MouseEvent>) => { e.stopPropagation(); ap.onSelect?.(part.id); },
      }
    : {};
  return (
    <>
      <group ref={ref} name={part.id} userData={{ part }} visible={!hidden} {...events}>
        {children(mat, <Highlight active={hovered && !ghost} selected={selected} />)}
      </group>
      <primitive object={line} />
    </>
  );
}

// ------------------------------------------------------------------------------------------------
function WingPivot({ product, side, foldDeg, smooth, children }: { product: Product; side: 'L' | 'R'; foldDeg: number; smooth: boolean; children: ReactNode }) {
  const pivot = useMemo(() => hingePivot(product.geometry.hinge, side), [product, side]);
  const ref = useRef<Group>(null);
  const current = useRef(foldDeg);
  const target = useRef(foldDeg);
  target.current = foldDeg;
  const q = useMemo(() => new Quaternion(), []);
  useFrame((_, dt) => {
    if (!ref.current) return;
    current.current = smooth ? current.current + (target.current - current.current) * (1 - Math.exp(-10 * dt)) : target.current;
    q.copy(foldQuaternion(product.geometry.hinge, side, current.current));
    ref.current.quaternion.copy(q);
  });
  return (
    <group name={side === 'L' ? 'hinge_left' : 'hinge_right'} position={pivot}>
      <group ref={ref} quaternion={foldQuaternion(product.geometry.hinge, side, foldDeg)}>
        <group position={pivot.clone().negate()}>{children}</group>
      </group>
    </group>
  );
}

// ------------------------------------------------------------------------------------------------
function WingMesh({ product, side, mat, hl }: { product: Product; side: 'L' | 'R'; mat: Material; hl: ReactNode }) {
  const geom = useMemo(() => {
    const g = product.geometry;
    const pv = hingePivot(g.hinge, side);
    const tipZ = (side === 'R' ? 1 : -1) * (g.spanM / 2);
    const sec = (z: number, c: number) => ({ le: new Vector3(pv.x + 0.25 * c, pv.y, z), chord: c });
    return loftAirfoil([sec(pv.z, g.chordRootM), sec(tipZ, g.chordTipM)], g.airfoil, { flip: side === 'L' });
  }, [product, side]);
  useEffect(() => () => geom.dispose(), [geom]);
  return (
    <mesh geometry={geom} material={mat} castShadow receiveShadow>
      {hl}
    </mesh>
  );
}

function FuselageMesh({ product, mat, hl }: { product: Product; mat: Material; hl: ReactNode }) {
  const geom = useMemo(() => fuselageGeometry(product.geometry.fuselageLengthM, product.geometry.fuselageDiameterM, noseLength(product)), [product]);
  useEffect(() => () => geom.dispose(), [geom]);
  return <mesh geometry={geom} material={mat} castShadow receiveShadow>{hl}</mesh>;
}

function NoseMesh({ product, mat, hl }: { product: Product; mat: Material; hl: ReactNode }) {
  const geom = useMemo(() => {
    const L = product.geometry.fuselageLengthM;
    return noseGeometry(L / 2 - noseLength(product), noseLength(product), product.geometry.fuselageDiameterM);
  }, [product]);
  useEffect(() => () => geom.dispose(), [geom]);
  return <mesh geometry={geom} material={mat} castShadow>{hl}</mesh>;
}

function TailMesh({ product, mat, hl }: { product: Product; mat: Material; hl: ReactNode }) {
  const geoms = useMemo(() => {
    const g = product.geometry;
    const L = g.fuselageLengthM;
    const D = g.fuselageDiameterM;
    const rootC = 0.18 * L;
    const xLe = -L / 2 + rootC + 0.02 * L;
    const finH = 0.16 * L + 0.4 * D;
    const fin = loftAirfoil(
      [
        { le: new Vector3(xLe, D * 0.3, 0), chord: rootC },
        { le: new Vector3(xLe - 0.45 * rootC, D * 0.3 + finH, 0), chord: rootC * 0.55 },
      ],
      '0010',
      { vertical: true },
    );
    const halfSpan = 0.12 * g.spanM;
    const hc = rootC * 0.85;
    const stab = (s: 1 | -1) =>
      loftAirfoil(
        [
          { le: new Vector3(xLe - 0.1 * rootC, D * 0.15, 0), chord: hc },
          { le: new Vector3(xLe - 0.35 * hc, D * 0.15, s * halfSpan), chord: hc * 0.6 },
        ],
        '0009',
        { flip: s === -1 },
      );
    return [fin, stab(1), stab(-1)];
  }, [product]);
  useEffect(() => () => geoms.forEach((g) => g.dispose()), [geoms]);
  return (
    <>
      {geoms.map((g, i) => (
        <mesh key={i} geometry={g} material={mat} castShadow>{i === 0 ? hl : null}</mesh>
      ))}
    </>
  );
}

function HingeMesh({ product, part, mat, hl }: { product: Product; part: Part; mat: Material; hl: ReactNode }) {
  const side = part.positionM[2] < 0 ? 'L' : 'R';
  const { q, pos, r, len } = useMemo(() => {
    const axis = hingeAxis(product.geometry.hinge, side);
    return {
      q: new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), axis),
      pos: hingePivot(product.geometry.hinge, side),
      r: product.geometry.fuselageDiameterM * 0.09,
      len: product.geometry.fuselageDiameterM * 0.75,
    };
  }, [product, side]);
  return (
    <mesh position={pos} quaternion={q} material={mat} castShadow>
      <cylinderGeometry args={[r, r, len, 20]} />
      {hl}
    </mesh>
  );
}

function NacelleMesh({ product, part, mat, hl }: { product: Product; part: Part; mat: Material; hl: ReactNode }) {
  const rotor = product.geometry.rotors[part.rotorIndex ?? 0];
  const { center, len, r } = useMemo(() => {
    const hub = rotorHubCruise(product, rotor);
    const chord = product.geometry.chordRootM;
    const back = hub.x - rotor.hubOffsetM - 0.55 * chord;
    const front = hub.x - 0.03 * rotor.diameterM;
    return { center: new Vector3((front + back) / 2, hub.y, hub.z), len: front - back, r: 0.05 * rotor.diameterM };
  }, [product, rotor]);
  return (
    <mesh position={center} rotation={[0, 0, -Math.PI / 2]} material={mat} castShadow>
      <capsuleGeometry args={[r, Math.max(0.01, len - 2 * r), 6, 16]} />
      {hl}
    </mesh>
  );
}

function RotorMesh({ product, part, mat, hl, load, saturated, folded }: { product: Product; part: Part; mat: Material; hl: ReactNode; load: number; saturated: boolean; folded: boolean }) {
  const rotor = product.geometry.rotors[part.rotorIndex ?? 0];
  const hub = useMemo(() => rotorHubCruise(product, rotor), [product, rotor]);
  const R = rotor.diameterM / 2;
  const blade = useMemo(() => bladeGeometry(R), [R]);
  const hubGeom = useMemo(() => new SphereGeometry(R * 0.09, 16, 12), [R]);
  const discGeom = useMemo(() => new CylinderGeometry(R, R, 0.002, 48, 1, true), [R]);
  const spin = useRef<Group>(null);
  const foldRefs = [useRef<Group>(null), useRef<Group>(null)];
  const satMat = useMemo(() => new MeshStandardMaterial({ color: '#ef4444', emissive: '#ef4444', emissiveIntensity: 1.2, side: DoubleSide }), []);
  const discMat = useMemo(() => new MeshStandardMaterial({ color: '#94a3b8', transparent: true, opacity: 0.08, side: DoubleSide, depthWrite: false }), []);
  const bladeMeshes = [useRef<Mesh>(null), useRef<Mesh>(null)];
  const omega = useRef(0);
  const foldAng = useRef(folded ? Math.PI / 2 : 0);
  useEffect(() => () => { blade.dispose(); hubGeom.dispose(); discGeom.dispose(); satMat.dispose(); discMat.dispose(); }, [blade, hubGeom, discGeom, satMat, discMat]);
  useFrame(({ clock }, dt) => {
    const targetOmega = folded ? 0 : 38 * Math.sqrt(Math.max(0, load));
    omega.current += (targetOmega - omega.current) * (1 - Math.exp(-3 * dt));
    if (spin.current) {
      spin.current.rotation.x += omega.current * dt;
      if (folded && omega.current < 0.5) {
        // park the blades horizontally before folding them back
        const a = spin.current.rotation.x % Math.PI;
        spin.current.rotation.x -= a * (1 - Math.exp(-6 * dt));
      }
    }
    const fTarget = folded && omega.current < 1 ? Math.PI / 2 - 0.08 : 0;
    foldAng.current += (fTarget - foldAng.current) * (1 - Math.exp(-4 * dt));
    foldRefs[0].current && (foldRefs[0].current.rotation.z = foldAng.current);
    foldRefs[1].current && (foldRefs[1].current.rotation.z = foldAng.current);
    const blink = saturated && Math.floor(clock.elapsedTime * 6) % 2 === 0;
    for (const b of bladeMeshes) if (b.current) b.current.material = blink ? satMat : mat;
  });
  return (
    <group position={hub}>
      <mesh geometry={hubGeom} material={saturated ? satMat : mat}>{hl}</mesh>
      <mesh geometry={discGeom} material={discMat} rotation={[0, 0, Math.PI / 2]} visible={!folded && load > 0.05} />
      <group ref={spin}>
        {[0, 1].map((k) => (
          <group key={k} rotation={[k * Math.PI, 0, 0]}>
            <group ref={foldRefs[k]}>
              <mesh ref={bladeMeshes[k]} geometry={blade} material={mat} castShadow />
            </group>
          </group>
        ))}
      </group>
    </group>
  );
}

function BoxPart({ part, mat, hl, fallback }: { part: Part; mat: Material; hl: ReactNode; fallback: [number, number, number] }) {
  const s = part.sizeM ?? fallback;
  return (
    <RoundedBox args={s} radius={Math.min(...s) * 0.15} smoothness={3} position={part.positionM} material={mat} castShadow>
      {hl}
    </RoundedBox>
  );
}

function PayloadMesh({ product, part, payload, mat, hl }: { product: Product; part: Part; payload: PayloadOption | null; mat: Material; hl: ReactNode }) {
  const D = product.geometry.fuselageDiameterM;
  const [x, y, z] = part.positionM;
  if (!payload || payload.shape === 'none') return null;
  const k = Math.cbrt(Math.max(0.2, payload.massKg) / 10) * D;
  switch (payload.shape) {
    case 'gimbal':
      return (
        <group position={[x, y, z]}>
          <mesh material={mat} position={[0, 0.25 * k, 0]}><cylinderGeometry args={[0.12 * k, 0.12 * k, 0.25 * k, 16]} /></mesh>
          <mesh material={mat} castShadow><sphereGeometry args={[0.3 * k, 24, 16]} />{hl}</mesh>
          <mesh position={[0.22 * k, 0, 0]} rotation={[0, 0, Math.PI / 2]}><cylinderGeometry args={[0.1 * k, 0.1 * k, 0.1 * k, 16]} /><meshStandardMaterial color="#0f172a" /></mesh>
        </group>
      );
    case 'lidar':
      return (
        <group position={[x, y, z]}>
          <mesh material={mat} castShadow><cylinderGeometry args={[0.3 * k, 0.3 * k, 0.45 * k, 24]} />{hl}</mesh>
          <mesh position={[0, -0.1 * k, 0]}><cylinderGeometry args={[0.31 * k, 0.31 * k, 0.12 * k, 24]} /><meshStandardMaterial color="#0f172a" metalness={0.6} roughness={0.2} /></mesh>
        </group>
      );
    case 'pod':
      return (
        <mesh position={[x, y, z]} rotation={[0, 0, Math.PI / 2]} material={mat} castShadow>
          <capsuleGeometry args={[0.28 * k, 1.4 * k, 8, 20]} />
          {hl}
        </mesh>
      );
    case 'antenna':
      return (
        <group position={[x, y, z]}>
          <RoundedBox args={[0.9 * k, 0.35 * k, 0.5 * k]} radius={0.05 * k} material={mat} castShadow>{hl}</RoundedBox>
          {[-0.3, 0, 0.3].map((dx) => (
            <mesh key={dx} position={[dx * k, -0.45 * k, 0]}><cylinderGeometry args={[0.015 * k, 0.015 * k, 0.6 * k, 6]} /><meshStandardMaterial color="#e2e8f0" /></mesh>
          ))}
        </group>
      );
    default:
      return (
        <RoundedBox args={[1.3 * k, 0.55 * k, 0.75 * k]} radius={0.04 * k} position={[x, y - 0.05 * k, z]} material={mat} castShadow>
          {hl}
        </RoundedBox>
      );
  }
}

function GearMesh({ product, mat, hl }: { product: Product; mat: Material; hl: ReactNode }) {
  const g = product.geometry;
  const D = g.fuselageDiameterM;
  const yb = -(D / 2 + g.landingGearHeightM);
  const w = D * 1.4;
  const L = g.fuselageLengthM * 0.62;
  const r = D * 0.035;
  const xs = [L * 0.32, -L * 0.42];
  return (
    <group position={[-0.05 * g.fuselageLengthM, 0, 0]}>
      {[-1, 1].map((s) => (
        <group key={s}>
          <mesh position={[0, yb, s * w]} rotation={[0, 0, Math.PI / 2]} material={mat} castShadow>
            <capsuleGeometry args={[r, L, 4, 10]} />
            {s === 1 ? hl : null}
          </mesh>
          {xs.map((x) => {
            const a = new Vector3(x, yb, s * w);
            const b = new Vector3(x, -D * 0.3, s * D * 0.3);
            const mid = a.clone().add(b).multiplyScalar(0.5);
            const q = new Quaternion().setFromUnitVectors(new Vector3(0, 1, 0), b.clone().sub(a).normalize());
            return (
              <mesh key={x} position={mid} quaternion={q} material={mat} castShadow>
                <cylinderGeometry args={[r * 0.8, r * 0.8, a.distanceTo(b), 8]} />
              </mesh>
            );
          })}
        </group>
      ))}
    </group>
  );
}

// ------------------------------------------------------------------------------------------------
function renderPart(part: Part, product: Product, ap: AircraftProps, mat: Material, hl: ReactNode) {
  switch (part.kind) {
    case 'fuselage': return <FuselageMesh product={product} mat={mat} hl={hl} />;
    case 'nose': return <NoseMesh product={product} mat={mat} hl={hl} />;
    case 'tail': return <TailMesh product={product} mat={mat} hl={hl} />;
    case 'wing': return <WingMesh product={product} side={part.attach === 'wingL' ? 'L' : 'R'} mat={mat} hl={hl} />;
    case 'hinge': return <HingeMesh product={product} part={part} mat={mat} hl={hl} />;
    case 'nacelle': return <NacelleMesh product={product} part={part} mat={mat} hl={hl} />;
    case 'rotor': {
      const i = part.rotorIndex ?? 0;
      return <RotorMesh product={product} part={part} mat={mat} hl={hl} load={ap.rotorLoad?.[i] ?? 0} saturated={!!ap.rotorSaturated?.[i]} folded={!!ap.propsFolded?.[i]} />;
    }
    case 'payload': return <PayloadMesh product={product} part={part} payload={ap.payload} mat={mat} hl={hl} />;
    case 'gear': return <GearMesh product={product} mat={mat} hl={hl} />;
    case 'battery': return <BoxPart part={part} mat={mat} hl={hl} fallback={[0.5, 0.2, 0.2]} />;
    case 'avionics': return <BoxPart part={part} mat={mat} hl={hl} fallback={[0.3, 0.15, 0.2]} />;
    default: return <BoxPart part={part} mat={mat} hl={hl} fallback={[0.2, 0.1, 0.1]} />;
  }
}

export const Aircraft = memo(function Aircraft(ap: AircraftProps) {
  const { product } = ap;
  const groups = useMemo(() => ({
    body: product.parts.filter((p) => p.attach === 'body'),
    L: product.parts.filter((p) => p.attach === 'wingL'),
    R: product.parts.filter((p) => p.attach === 'wingR'),
  }), [product]);
  const node = (part: Part) => (
    <PartNode key={part.id} part={part} product={product} ap={ap}>
      {(mat, hl) => renderPart(part, product, ap, mat, hl)}
    </PartNode>
  );
  return (
    <group name="aircraft" userData={{ productId: product.id }}>
      {groups.body.map(node)}
      <WingPivot product={product} side="L" foldDeg={ap.foldDeg} smooth={ap.smoothFold ?? true}>{groups.L.map(node)}</WingPivot>
      <WingPivot product={product} side="R" foldDeg={ap.foldDeg} smooth={ap.smoothFold ?? true}>{groups.R.map(node)}</WingPivot>
    </group>
  );
});
